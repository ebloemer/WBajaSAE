// This file contains the code for the ecvtCode project.

#include <HardwareSerial.h>
#include <AS5600.h>
#include <Wire.h>
#include <BluetoothSerial.h>

//error codes
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Serial communication
HardwareSerial Onboard(0); // UART0
BluetoothSerial SerialBT; // Bluetooth debug

// Define an instance of the AS5600 sensor
AS5600 Encoder; // AS5600 sensor

// Pin Assignments *DO NOT CHANGE* ----------------------------------------------------------------------------

// High Power Outputs
#define motorForwardA 32  // forward + mosfet
#define motorForwardB 26  // forward - mosfet
#define motorReverseA 25  // reverse + mosfet
#define motorReverseB 33  // reverse - mosfet

// Low Power Inputs
#define engineSensor  12
#define batterySensor 36
#define brakeInput    27

// Communication Pins
#define linkRx 3
#define linkTx 1
#define SDA 21
#define SCL 22

// Variable declarations ----------------------------------------------------------------

// Battery variables:
int batLowLimit = 2150;
int batHighLimit = 3180;

int rawBattery = batLowLimit;
int batPercent;

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
unsigned long brakeTimer;
const int brakeDebounce = 100;

int brakePrevState = 0;
int brakeStatus = 0;  // This is (1) when the brake has been pressed for 1/2 second or more

// Throttle variables
int throttleMin = 2910;
int throttleMax = 1820;

int rawThrottle = throttleMin;
int throttlePos = 0;

// Helix variables
float rawHelix = 0;
float helixPos = 0;
int helixMax = 47;
int helixMin = 0;
float helixOffset = 166;

int helixMinVariance = 5;
int helixMaxVariance = 5;

int closeSpeed = 150;
int openSpeed = 150;
int returnSpeed = 100;

int limitCheck = 1;

// Motor variables
int forwardA = 0;
int forwardB = 0;
int reverseA = 0;
int reverseB = 0;

int motorState = 2;	// 0 = Forward, 1 = Reverse, 2 = Stopped, 3 = Past Min, 4 = Past Max

// Define PID parameters
double Kp = 0.1;  // Proportional gain
double Ki = 0.1;  // Integral gain
double Kd = 0.01;  	// Derivative gain
double Ti = 100;	// Integral time constant
double Td = 1000;	// Derivative time constant

double previousError = 0;
double integral = 0;
double output = 0;  // Output control signal

unsigned long pidPrevTime = 0;
const int pidInterval = 1;	// Controls how often the PID loop runs in milliseconds
int pidElapsedTime;

// Serial communication
String incomingOnboardData = "";
String incomingBluetoothData = "";

// Runtime variables
unsigned long updateTimer = 0;
const int updateInterval = 50;



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
	pinMode(motorReverseA, OUTPUT);

	// Setup LEDC for PWM
	ledcSetup(0, 1000, 8);		//forward
	ledcSetup(1, 1000, 8);		//reverse

	// Set the pin modes for the sensor inputs
	pinMode(engineSensor, INPUT);
	pinMode(batterySensor, INPUT);

	// Set the pin modes for the brake input and launch button
	pinMode(brakeInput, INPUT);

	// Ensure the motor is stopped
	stopCVT();
}

// Loop function
void loop() {

	limitCheck = checkLimits();		//purely incase I forget in any scenario

	// Read serial data
	readOnboardData(); // Read onboard data
	if(SerialBT.connected()) {
		readBluetoothData(); // Read bluetooth data
	}

	if(actualRpm < 1500){	// Correct for low idling/starting RPM
		actualRpm = 1500;
	}
	
	brake(); // Check brake conditions

	// This begins 'useful' code--------------------------------------------------------------------------

	// Normal operation
  pidElapsedTime = millis() - pidPrevTime;

	if(launchActive == 0 && pidElapsedTime >= pidInterval) {
		setCommandRPM();

		exportOnboardData(); // Export onboard data
		if(SerialBT.connected()) {
			exportBluetoothData();
		}

		pidPrevTime = millis();
	} 
	
	// Launch operation
	else if (launchActive == 1){
		setCommandHelix(helixMin);

		Onboard.println("Launch Active..,");
		if(SerialBT.connected()) {
			SerialBT.println("Launch Active..,");
		}
	}

	// This ends 'useful' code--------------------------------------------------------------------------

	// Update certain parameters at a fixed interval
	if (millis() - updateTimer >= updateInterval) {
		batRead(); // Read battery voltage

		//exportOnboardData(); // Export onboard data

		updateTimer = millis();
	}
}

// Function to open the CVT
void openCVT(int revSpeed) {
	// Check if the motor is already in the reverse state
	if(motorState != 1) {
		stopCVT();
		digitalWrite(motorReverseA, HIGH); // Turn on the reverse + mosfet
		ledcAttachPin(motorReverseB, 1); // Attach the reverse - mosfet to LEDC channel 1
		ledcWrite(1, revSpeed); // Set the speed of the reverse - mosfet
		motorState = 1;
	}

	// Check if the speed of the reverse - mosfet needs to be updated
	if(ledcRead(1) != revSpeed) {
		ledcWrite(1, revSpeed);
	}

	// Print the opening message to the onboard serial communication
	Onboard.print("Opening..,");
	Onboard.println(revSpeed);

	// Print the opening message to the Bluetooth serial communication if connected
	if(SerialBT.connected()) {
		SerialBT.println("Opening..,");
	}
}

// Function to close the CVT
void closeCVT(int fwdSpeed) {
	// Check if the motor is already in the forward state
	if(motorState != 0) {
		stopCVT();
		digitalWrite(motorForwardA, HIGH); // Turn on the forward + mosfet
		ledcAttachPin(motorForwardB, 0); // Attach the forward - mosfet to LEDC channel 0
		ledcWrite(0, fwdSpeed); // Set the speed of the forward - mosfet
		motorState = 0;
	}

	// Check if the speed of the forward - mosfet needs to be updated
	if(ledcRead(0) != fwdSpeed) {
		ledcWrite(0, fwdSpeed);
	}

	// Print the closing message to the onboard serial communication
	Onboard.print("Closing..,");
	Onboard.println(fwdSpeed);

	// Print the closing message to the Bluetooth serial communication if connected
	if(SerialBT.connected()) {
		SerialBT.println("Closing..,");
	}
}

// Function to stop the CVT
void stopCVT() {
	// Read the helix position
	helixRead();

	// Turn off the forward and reverse + mosfets
	digitalWrite(motorForwardA, LOW);
	digitalWrite(motorReverseA, LOW);

	// Detach the forward and reverse - mosfets from LEDC channels
	ledcDetachPin(motorForwardB);
	ledcDetachPin(motorReverseB);

	// Set the speed of the LEDC channels to idle
	ledcWrite(0, 0);
	ledcWrite(1, 0);

	// Delay for 1 millisecond to ensure the motor is disconnected
	delay(1);

	// Update the motor state to indicate that it is stopped
	motorState = 2;

	// Print the stopping message to the onboard serial communication
	Onboard.print("Stopping..,");

	// Print the stopping message to the Bluetooth serial communication if connected
	if(SerialBT.connected()) {
		SerialBT.print("Stopping..,");
	}
}

// Function to get the command RPM
void getCommandRPM() {
	throttlePos = map(rawThrottle, throttleMin, throttleMax, 0, 100);

	throttlePos = constrain(throttlePos, 0, 100);

	if(throttlePos <= 5){
		throttlePos = 0;
	}

	commandRpm = map(throttlePos, 0, 100, 1500, 3000);
}

// Function to set the command RPM
void setCommandRPM(){

  // Set the command RPM based on the throttle position
  getCommandRPM();

  // Calculate error
  double error = commandRpm - actualRpm;

  // Calculate integral term (approximate integral using trapezoidal rule)
  integral += (error + previousError) * pidElapsedTime / Ti;  // Convert milliseconds to seconds

  // Calculate derivative term
  double derivative = (error - previousError) / (pidElapsedTime / Td);  // Convert milliseconds to seconds

  // Compute PID output
  output = Kp * error + Ki * integral + Kd * derivative;

	// Print PID parameters
	Onboard.print("Error: ");
	Onboard.print(error);
	Onboard.print(", ");

	Onboard.print("Integral: ");
	Onboard.print(integral);
	Onboard.print(", ");

	Onboard.print("Derivative: ");
	Onboard.print(derivative);
	Onboard.println(", ");

  // Update previous values for next iteration
  previousError = error;

  // Ensure output is within acceptable bounds (e.g., for PWM control)
  output = constrain(output, -255, 255);

	// Read the helix position
  limitCheck = checkLimits();

  // Execute PID control signal
  if (output > 0 && limitCheck==0 && helixPos > (helixMin+helixMinVariance)) {
    // Open CVT
    openCVT(output); // Adjust PWM duty cycle for motor control
  } else if (output < 0 && limitCheck==0 && helixPos < (helixMax-helixMaxVariance)) {
    // Close CVT
    closeCVT(abs(output)); // Adjust PWM duty cycle for motor control
  } else if (limitCheck == 0){
    // Stop CVT
    stopCVT();
		
		Onboard.print("Setpoint reached..,");
		Onboard.println(output);
		if(SerialBT.connected()) {
			SerialBT.println("Setpoint reached..,");
		}
  }
}

// Function to handle when the helix position is past the minimum limit
void pastMin(int speed) {
	// Check if the motor state is not already indicating past the minimum limit
	if (motorState != 3) {
		stopCVT();
		digitalWrite(motorForwardA, HIGH); // Turn on the forward + mosfet
		ledcAttachPin(motorForwardB, 0); // Attach the forward - mosfet to LEDC channel 0
		ledcWrite(0, speed); // Set the speed of the forward - mosfet
		motorState = 3;
	}

	// Check if the speed of the forward - mosfet needs to be updated
	if (ledcRead(0) != speed) {
		ledcWrite(0, speed);
	}

	// Print the message indicating that the helix is past the minimum limit to the onboard serial communication
	Onboard.println("Helix PAST min..,");

	// Print the message indicating that the helix is past the minimum limit to the Bluetooth serial communication if connected
	if (SerialBT.connected()) {
		SerialBT.println("Helix PAST min..,");
	}
}

// Function to handle when the helix position is past the maximum limit
void pastMax(int speed) {
	// Check if the motor state is not already indicating past the maximum limit
	if (motorState != 4) {
		stopCVT();
		digitalWrite(motorReverseA, HIGH); // Turn on the reverse + mosfet
		ledcAttachPin(motorReverseB, 1); // Attach the reverse - mosfet to LEDC channel 1
		ledcWrite(1, speed); // Set the speed of the reverse - mosfet
		motorState = 4;
	}

	// Check if the speed of the reverse - mosfet needs to be updated
	if (ledcRead(1) != speed) {
		ledcWrite(1, speed);
	}

	// Print the message indicating that the helix is past the maximum limit to the onboard serial communication
	Onboard.println("Helix PAST max..,");

	// Print the message indicating that the helix is past the maximum limit to the Bluetooth serial communication if connected
	if (SerialBT.connected()) {
		SerialBT.println("Helix PAST max..,");
	}
}

// Function to check if the helix position is within the limits
int checkLimits() {
	helixRead(); // Read the helix position from the sensor

	// Check if the helix position is past the maximum limit
	if (helixPos > (helixMax + helixMaxVariance)) {
		pastMax(returnSpeed); // Handle when the helix position is past the maximum limit
		return 1;
	}
	// Check if the helix position is past the minimum limit
	else if (helixPos < (helixMin - helixMinVariance)) {
		pastMin(returnSpeed); // Handle when the helix position is past the minimum limit
		return 1;
	}
	// The helix position is within the limits
	else {
		return 0; // Return 0 to indicate that the helix position is within the limits
	}
}

// Function to set the command helix position
void setCommandHelix(int commandHelix) {

	limitCheck = checkLimits(); // Check if the helix position is within the limits

	// Check if the helix position is above the setpoint
	if (helixPos > (commandHelix + helixMinVariance) && limitCheck == 0) {
		openCVT(openSpeed); // Open the CVT
	}

	// Check if the helix position is below the setpoint
	else if (helixPos < (commandHelix - helixMaxVariance) && limitCheck == 0) {
		closeCVT(closeSpeed); // Close the CVT
	}
	// Check if the helix position is within the acceptable range
	else if (limitCheck == 0) {
		stopCVT(); // Stop the CVT

		// Print the message indicating that the helix is at the setpoint to the Bluetooth serial communication if connected
		if (SerialBT.connected()) {
			SerialBT.println("Helix at setpoint..,");
		}

		// Print the message indicating that the helix is at the setpoint to the onboard serial communication
		Onboard.println("Helix at setpoint..,");
	}
}

// Function to read the helix position
void helixRead() {
	// Read the helix position from the AS5600 sensor
	rawHelix = map(Encoder.readAngle(),4095,0,0,4095);

	// Convert the raw helix position to degrees
	helixPos = ((rawHelix * AS5600_RAW_TO_DEGREES) - helixOffset);
}

// Function to read the battery voltage and calculate the battery percentage
void batRead() {

	int rawBattery = analogRead(batterySensor);

	// Average the reading
	if(rawBattery > batLowLimit) {
		updateBatAverage(rawBattery);
	}
}

// Function to update the moving average of battery samples
void updateBatAverage(int newValue) {
	batLogger[batIndex] = newValue;
	batIndex = (batIndex + 1) % batSamples;
	int sum = 0;

	// Calculate the sum of all battery samples
	for (int i = 0; i < batSamples; i++) {
		sum += batLogger[i];
	}

	// Calculate the battery percentage based on the average value
	batPercent = map((sum / batSamples), batLowLimit, batHighLimit, 0, 99);

	// Ensure that the battery percentage is within the valid range
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
		brakeTimer = millis();
	}

	// Check if debounce time has passed
	if (millis() - brakeTimer >= brakeDebounce) {
		// Update brake status based on current state
		if (brakeState == HIGH) {
			brakeStatus = 1; // Brake is pressed
		} else {
			brakeStatus = 0; // Brake is not pressed
		}
	}

	brakePrevState = brakeState;
}

// Functions to read data from UART & Bluetooth
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

// Functions to process data from UART & Bluetooth
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

// Functions to export data to UART & Bluetooth
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

	Onboard.print("Commanded:");
	Onboard.print(commandRpm);
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