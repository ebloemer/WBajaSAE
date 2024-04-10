// This file contains the code for the ecvtCode project.

#include <HardwareSerial.h>
#include <AS5600.h>
#include <Wire.h>
#include <BluetoothSerial.h>

//error codes
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// serial communication
HardwareSerial Onboard(0); // UART0
BluetoothSerial SerialBT; // Bluetooth debug

// Define an instance of the AS5600 sensor
AS5600 Encoder; // AS5600 sensor

// Pin Assignments *DO NOT CHANGE* ----------------------------------------------------------------------------

// High Power Outputs
#define motorForwardA 32  // forward + mosfet
#define motorForwardB 26  // forward - mosfet
#define motorReverseA 25  // reverse + mosfet
#define motorReverseB 2  // reverse - mosfet

int forwardA = 0;
int forwardB = 0;
int reverseA = 0;
int reverseB = 0;

// Low Power Inputs
#define linkRx 3
#define linkTx 1
#define engineSensor  12
#define batterySensor 36
#define brakeInput    27
#define launchButton  15  // can change this one

#define SDA 21
#define SCL 22

// Variable declarations ----------------------------------------------------------------

// Battery variables:
int rawBattery;
int batPercent;
int batLowLimit = 2150;
int batHighLimit = 3180;
const int batSamples = 200;
int batLogger[batSamples];
int batIndex = 0;

// Engine RPM variables:
int commandRpm;
int actualRpm = 0;

int rpmVariance = 50;

// Launch variables
int launchActive = 0; // This is (1) when the CVT needs to be open/system is in launch mode

// Brake variables
int brakePrevState = 0;
unsigned long brakeDebounce;
int brakeStatus = 0;  // This is (1) when the brake has been pressed for 1/2 second or more

// Throttle variables
int rawThrottle = 0;
int throttlePos = 0;
int throttleMin = 2910;
int throttleMax = 1820;

// Helix variables
float rawHelix = 0;
float helixPos = 0;
int helixMax = 47;
int helixMin = 0;
float helixOffset = 166;
float rawHelixDegrees = 0;

int closeSpeed = 150;
int openSpeed = 150;
int returnSpeed = 100;

// PID shit
// Define PID parameters
double Kp = 0.1;  // Proportional gain
double Ki = 0.001;  // Integral gain
double Kd = 0;  // Derivative gain

double output = 0;  // Output control signal

double previousError = 0;
double integral = 0;
int pidInterval = 100;
int pidTimer;

// Serial communication
String incomingOnboardData = "";
String incomingBluetoothData = "";

// Global variables
const int debounce = 100;
int elapsedTime;
unsigned long updateTimer = 0;
const int updateInterval = 50;
int motorState = 2;	// 0 = Forward, 1 = Reverse, 2 = Stopped, 3 = Past Min, 4 = Past Max


// Function prototypes --------------------------------------------------------------------
void openCVT(int revSpeed);
void closeCVT(int fwdSpeed);
void stopCVT();
void getCommandRPM();
void setCommandRPM();
void pastMin(int speed);
void pastMax(int speed);
int checkLimits();
void setCommandHelix(int commandHelix);
void helixRead();
void helixCalibrate(int duration);
void batRead();
void updateBatAverage(int newValue);
void brake();
void readOnboardData();
void processOnboardData(String data);
void exportOnboardData();
void readBluetoothData();
void processBluetoothData(String data);
void exportBluetoothData();

// Functions ---------------------------------------------------------------------------

void setup() {
	// Initialize the Onboard serial communication at a baud rate of 115200
	Onboard.begin(115200, SERIAL_8N1, linkRx, linkTx);

	// Initialize the Bluetooth serial communication
	SerialBT.begin("BajaECVT");

	// Initialize the AS5600 sensor
	Wire.begin();
	Encoder.begin();

	// Initialize battery samples array
	for (int i = 0; i < batSamples; i++) {
		batLogger[i] = batLowLimit;
	}

	// Set the pin modes for the motor control pins
	pinMode(motorForwardA, OUTPUT);
	//pinMode(motorForwardB, OUTPUT);
	pinMode(motorReverseA, OUTPUT);
	//pinMode(motorReverseB, OUTPUT);

	// Setup LEDC for PWM
	ledcSetup(0, 1000, 8);		//forward
	ledcSetup(1, 1000, 8);		//reverse

	// Set the pin modes for the sensor inputs
	pinMode(engineSensor, INPUT);
	pinMode(batterySensor, INPUT);

	stopCVT();

	// Set the pin modes for the brake input and launch button
	pinMode(brakeInput, INPUT);
	pinMode(launchButton, INPUT_PULLUP);
}

// Loop function
void loop() {
	readOnboardData(); // Read onboard data
	if(SerialBT.connected()) {
		readBluetoothData(); // Read bluetooth data
	}

	brake(); // Check brake conditions


	/*     digitalWrite(motorForwardA, forwardA);
    digitalWrite(motorForwardB, forwardB);
    digitalWrite(motorReverseA, reverseA);
    digitalWrite(motorReverseB, reverseB); */

  elapsedTime = millis() - pidTimer;
	
	if(launchActive == 0 && elapsedTime >= pidInterval) {
		setCommandRPM();

		exportOnboardData(); // Export onboard data
		if(SerialBT.connected()) {
			exportBluetoothData();
		}

		pidTimer = millis();
	} 
	
	else if (launchActive == 1){
		setCommandHelix(helixMin);

		Onboard.println("Launch Active..,");
		if(SerialBT.connected()) {
			SerialBT.println("Launch Active..,");
		}
	}

		

	// Perform tasks at a specific interval
	if (millis() - updateTimer >= updateInterval) {
		batRead(); // Read battery voltage

		//exportOnboardData(); // Export onboard data

		updateTimer = millis();
	}
}

// Function to open the CVT
void openCVT(int revSpeed) {
  if(motorState != 1) {
    stopCVT();
    digitalWrite(motorReverseA, HIGH); // motorReverseA ON
    ledcAttachPin(motorReverseB, 1);
    ledcWrite(1, revSpeed); // motorReverseA ON
    motorState = 1;
  }

  if(ledcRead(1) != revSpeed) {
    ledcWrite(1, revSpeed);
  }

  Onboard.println("Opening..,");
  if(SerialBT.connected()) {
    SerialBT.println("Opening..,");
  }
}

// Function to close the CVT
void closeCVT(int fwdSpeed) {

  if(motorState != 0) {
    stopCVT();
    digitalWrite(motorForwardA, HIGH);
    ledcAttachPin(motorForwardB, 0);
    ledcWrite(0, fwdSpeed); // motorForwardA ON
    motorState = 0;
  }

  if(ledcRead(0) != fwdSpeed) {
    ledcWrite(0, fwdSpeed);
  }

  Onboard.println("Closing..,");
  if(SerialBT.connected()) {
    SerialBT.println("Closing..,");
  }
}

// Function to stop the CVT
void stopCVT() {
	helixRead();
  digitalWrite(motorForwardA, LOW);
  digitalWrite(motorReverseA, LOW);
	ledcDetachPin(motorForwardB);
	ledcDetachPin(motorReverseB);
/* 	digitalWrite(motorForwardB, LOW);
	digitalWrite(motorReverseB, LOW); */
	
	ledcWrite(0, 0);
	ledcWrite(1, 0);

	delay(1);

	motorState = 2;

	Onboard.print("Stopping..,");
	if(SerialBT.connected()) {
		SerialBT.print("Stopping..,");
	}
}

// Function to set the command RPM
void getCommandRPM() {
	//throttlePos = map(rawThrottle, throttleMin, throttleMax, 0, 100);
	throttlePos = map(rawThrottle, 0, 100, 0, 100);

	throttlePos = constrain(throttlePos, 0, 100);

	//commandRpm = map(throttlePos, 0, 100, 1500, 2500);
	commandRpm = map(throttlePos, 0, 100, 0, 1000);
}

void setCommandRPM(){

  // Read the helix position
  helixRead();

  // Set the command RPM based on the throttle position
  getCommandRPM();


  // Calculate error
  double error = commandRpm - actualRpm;

  // Calculate integral term (approximate integral using trapezoidal rule)
  integral += (error + previousError) * elapsedTime / 2000.0;  // Convert milliseconds to seconds

  // Calculate derivative term
  double derivative = (error - previousError) / (elapsedTime / 1000.0);  // Convert milliseconds to seconds

  // Compute PID output
  output = Kp * error + Ki * integral + Kd * derivative;

  // Update previous values for next iteration
  previousError = error;

  // Apply output to the system (e.g., adjust motor speed)
  Serial.println(output);

  // Ensure output is within acceptable bounds (e.g., for PWM control)
  output = constrain(output, -255, 255);

	int limitCheck = checkLimits();

  // Execute PID control signal
  if (output > 0 && !limitCheck) {
    // Open CVT
    openCVT(output); // Adjust PWM duty cycle for motor control
  } else if (output < 0 && !limitCheck) {
    // Close CVT
    closeCVT(abs(output)); // Adjust PWM duty cycle for motor control
  } else if (!limitCheck){
    // Stop CVT
    stopCVT();
  }
}

void pastMin(int speed) {
	if(motorState != 3){
		stopCVT();
		digitalWrite(motorForwardA, HIGH);
		ledcAttachPin(motorForwardB, 0);
		ledcWrite(0, speed); // motorForwardA ON
		motorState = 3;
	}

	if(ledcRead(0) != speed) {
		ledcWrite(0, speed);
	}

	Onboard.println("Helix PAST min..,");
	if(SerialBT.connected()) {
		SerialBT.println("Helix PAST min..,");
	}
}

void pastMax(int speed) {
	if(motorState != 4){
		stopCVT();
		digitalWrite(motorReverseA, HIGH);
		ledcAttachPin(motorReverseB, 1);
		ledcWrite(1, speed); // motorReverseA ON
		motorState = 4;
	}

	if(ledcRead(1) != speed) {
		ledcWrite(1, speed);
	}

	Onboard.println("Helix PAST max..,");
	if(SerialBT.connected()) {
		SerialBT.println("Helix PAST max..,");
	}
}

int checkLimits() {
	helixRead();

	if (helixPos > (helixMax+5)) {
		pastMax(returnSpeed);
		return 1;
	} else if (helixPos < (helixMin-5)) {
		pastMin(returnSpeed);
		return 1;
	} else {
		return 0;
	}
}

void setCommandHelix(int commandHelix) {
	if (helixPos < commandHelix-10) {
		openCVT(openSpeed);
	} else if (helixPos > commandHelix+10) {
		closeCVT(closeSpeed);
	} else if (!checkLimits()) {
		stopCVT();

		if(SerialBT.connected()) {
			SerialBT.println("Helix at setpoint..,");
		}
		Onboard.println("Helix at setpoint..,");
	}
}

// Function to read the helix position
void helixRead() {
	// Read the helix position from the AS5600 sensor
	rawHelix = map(Encoder.readAngle(),4095,0,0,4095);

	// Convert the raw helix position to degrees
	//helixPos = ((rawHelix * AS5600_RAW_TO_DEGREES) - helixOffset);
}

// Function to read the battery voltage and calculate the battery percentage
void batRead() {

	int rawBattery = analogRead(batterySensor);

	// Calibrate the reading
	if(rawBattery > batLowLimit) {
		updateBatAverage(rawBattery);
	}
}

// Function to update the moving average of battery samples
void updateBatAverage(int newValue) {
	batLogger[batIndex] = newValue;
	batIndex = (batIndex + 1) % batSamples;
	int sum = 0;

	for (int i = 0; i < batSamples; i++) {
		sum += batLogger[i];
	}

	batPercent = map((sum / batSamples), batLowLimit, batHighLimit, 0, 99);

	if (batPercent < 0) {
		batPercent = 0;
	} else if (batPercent > 100) {
		batPercent = 100;
	}
}

// Function to handle brake input
void brake() {
	int brakeState = digitalRead(brakeInput);

	// Check if brake state has changed
	if (brakeState != brakePrevState) {
		brakeDebounce = millis();
	}

	// Check if debounce time has passed
	if (millis() - brakeDebounce >= 500) {
		// Update brake status based on current state
		if (brakeState == HIGH) {
			brakeStatus = 1; // Brake is pressed
		} else {
			brakeStatus = 0; // Brake is not pressed
		}
	}

	brakePrevState = brakeState;
}

// Function to read onboard data from UART
void readOnboardData() {
	while (Onboard.available() > 0) {
		char c = Onboard.read();

		if (c == ',') {
			processOnboardData(incomingOnboardData);
			incomingOnboardData = "";
		} else if (c != '\n') {
			incomingOnboardData += c;
		}
	}
}

void readBluetoothData() {
	while (SerialBT.available() > 0) {
		char c = SerialBT.read();

		if (c == ',') {
			processBluetoothData(incomingBluetoothData);
			incomingBluetoothData = "";
		} else if (c != '\n') {
			incomingBluetoothData += c;
		}
	}
}

// Function to process onboard data received from UART
void processOnboardData(String data) {
	int dataIndex = data.indexOf(':');

	if (dataIndex != -1) {
		String item = data.substring(0, dataIndex);

		if (item == "ForA") {
			forwardA = data.substring(dataIndex + 1).toInt();
		} else if (item == "ForB") {
			forwardB = data.substring(dataIndex + 1).toInt();
		} else if (item == "RevA"){
			reverseA = data.substring(dataIndex + 1).toInt();
		} else if (item == "RevB"){
			reverseB = data.substring(dataIndex + 1).toInt();
		} else if (item == "RPM") {
			actualRpm = data.substring(dataIndex + 1).toInt();
		} else if (item == "Throttle") {
			rawThrottle = data.substring(dataIndex + 1).toInt();
		} else if (item == "Launch"){
			launchActive = data.substring(dataIndex + 1).toInt();
		} else if (item == "Helix"){
			helixPos = data.substring(dataIndex + 1).toInt();
		} else if (item == "Return"){
			returnSpeed = data.substring(dataIndex + 1).toInt();
		} else if (item == "Kp"){
      Kp = data.substring(dataIndex + 1).toFloat();
    } else if (item == "Ki"){
      Ki = data.substring(dataIndex + 1).toFloat();
    } else if (item == "Kd"){
      Kd = data.substring(dataIndex + 1).toFloat();
    }
	}
}

void processBluetoothData(String data) {
	int dataIndex = data.indexOf(':');

	if (dataIndex != -1) {
		String item = data.substring(0, dataIndex);

		if (item == "ForA") {
			forwardA = data.substring(dataIndex + 1).toInt();
		} else if (item == "ForB") {
			forwardB = data.substring(dataIndex + 1).toInt();
		} else if (item == "RevA"){
			reverseA = data.substring(dataIndex + 1).toInt();
		} else if (item == "RevB"){
			reverseB = data.substring(dataIndex + 1).toInt();
		} else if (item == "Min"){
			helixMin = data.substring(dataIndex + 1).toInt();
		} else if (item == "Max"){
			helixMax = data.substring(dataIndex + 1).toInt();
		} else if (item == "RPM") {
			actualRpm = data.substring(dataIndex + 1).toInt();
		} else if (item == "Throttle") {
			rawThrottle = data.substring(dataIndex + 1).toInt();
		} else if (item == "Launch"){
			launchActive = data.substring(dataIndex + 1).toInt();
		} else if (item == "Helix"){
			helixPos = data.substring(dataIndex + 1).toInt();
		} else if (item == "Offset"){
			helixOffset = data.substring(dataIndex + 1).toInt();
		} else if (item == "Return"){
			returnSpeed = data.substring(dataIndex + 1).toInt();
		} else if (item == "Kp"){
      Kp = data.substring(dataIndex + 1).toFloat();
    } else if (item == "Ki"){
      Ki = data.substring(dataIndex + 1).toFloat();
    } else if (item == "Kd"){
      Kd = data.substring(dataIndex + 1).toFloat();
    }
    
	}
}

// Function to export onboard data to UART
void exportOnboardData() {
	Onboard.print("Forward A:"); // 0-1
	Onboard.print(digitalRead(motorForwardA));;
	Onboard.print(",");

	Onboard.print("Forward B:"); // 0-1
	Onboard.print(digitalRead(motorForwardB));
	Onboard.print(",");

	Onboard.print("Reverse A:"); // 0-1
	Onboard.print(digitalRead(motorReverseA));
	Onboard.print(",");

	Onboard.print("Reverse B:"); // 0-1
	Onboard.print(digitalRead(motorReverseB));
	Onboard.print(",");

/* 	Onboard.print("Battery:"); // 0-1
	Onboard.print(rawBattery);
	Onboard.print(","); */

	Onboard.print("RPM:");
	Onboard.print(actualRpm);
	Onboard.print(",");

	Onboard.print("Throttle:"); // 0-1
	Onboard.print(throttlePos);
	Onboard.print(",");

/* 	Onboard.print("Raw Throttle:"); // 0-1
	Onboard.print(rawThrottle);
	Onboard.print(","); */

	Onboard.print("Helix:"); // 0-1
	Onboard.print(helixPos);
	Onboard.print(",");

/* 	Onboard.print("Raw Helix:"); // 0-1
	Onboard.print(rawHelix);
	Onboard.print(","); */

	Onboard.print("Return Speed:"); // 0-1
	Onboard.print(returnSpeed);
	Onboard.print(",");

	Onboard.print("Kp:"); // 0-1
	Onboard.print(Kp);
	Onboard.print(",");

	Onboard.print("Ki:"); // 0-1
	Onboard.print(Ki);
	Onboard.print(",");

	Onboard.print("Kd:"); // 0-1
	Onboard.print(Kd);
	Onboard.println(",");
}

void exportBluetoothData(){
	SerialBT.print("ForA:"); // 0-1
	SerialBT.print(digitalRead(motorForwardA));;
	SerialBT.print(",");

	SerialBT.print("ForB:"); // 0-1
	SerialBT.print(digitalRead(motorForwardB));
	SerialBT.print(",");

	SerialBT.print("RevA:"); // 0-1
	SerialBT.print(digitalRead(motorReverseA));
	SerialBT.print(",");

	SerialBT.print("RevB:"); // 0-1
	SerialBT.print(digitalRead(motorReverseB));
	SerialBT.print(",");

	SerialBT.print("RPM:"); // 0-1
	SerialBT.print(actualRpm);
	SerialBT.print(",");

	SerialBT.print("Commanded:");
	SerialBT.print(commandRpm);
	SerialBT.print(",");

	SerialBT.print("Throttle:"); // 0-1
	SerialBT.print(throttlePos);
	SerialBT.print(",");

	SerialBT.print("Helix:"); // 0-1
	SerialBT.print(helixPos);
	SerialBT.print(",");

/* 	SerialBT.print("Raw Helix:"); // 0-1
	SerialBT.print(rawHelix);
	SerialBT.print(","); */

/* 	SerialBT.print("Helix min:"); // 0-1
	SerialBT.print(helixMin);
	SerialBT.print(",");

	SerialBT.print("Helix max:"); // 0-1
	SerialBT.print(helixMax);
	SerialBT.print(",");

	SerialBT.print("Offset:"); // 0-1
	SerialBT.print(helixOffset);
	SerialBT.print(","); */

	SerialBT.print("Launch:"); // 0-1
	SerialBT.print(launchActive);
	SerialBT.println(",");

	SerialBT.print("Return Speed:"); // 0-1
	SerialBT.print(returnSpeed);
	SerialBT.print(",");

	SerialBT.print("Kp:"); // 0-1
	SerialBT.print(Kp);
	SerialBT.print(",");

	SerialBT.print("Ki:"); // 0-1
	SerialBT.print(Ki);
	SerialBT.print(",");

	SerialBT.print("Kd:"); // 0-1
	SerialBT.print(Kd);
	SerialBT.println(",");
}