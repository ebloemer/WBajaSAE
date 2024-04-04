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
#define motorReverseB 33  // reverse - mosfet

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
int rpm = 0;

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

int returnSpeed = 50;

// Serial communication
String incomingOnboardData = "";
String incomingBluetoothData = "";

// Global variables
const int debounce = 100;
unsigned long iterationTimer = 0;
const int iterationInterval = 500;
int revDirection = 2;	// 0 = forward, 1 = reverse, 2 = stop


// Function prototypes --------------------------------------------------------------------
void openCVT(int revSpeed);
void closeCVT(int fwdSpeed);
void stopCVT();
void setCommandRPM();
void setCommandHelix();
void helixRead();
void helixCalibrate(int duration);
void potRead();
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
	pinMode(motorForwardB, OUTPUT);
	pinMode(motorReverseA, OUTPUT);
	pinMode(motorReverseB, OUTPUT);

	// Setup LEDC for PWM
	ledcSetup(0, 5000, 8);

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
	potRead(); // Read throttle position

	if(launchActive == 0) {
		setCommandRPM();
	} else if (launchActive == 1){
		setCommandHelix(0);
	}

	if(SerialBT.connected()) {
			exportBluetoothData();
		}

	// Perform tasks at a specific interval
	if (millis() - iterationTimer >= iterationInterval) {
		batRead(); // Read battery voltage


/*     digitalWrite(motorForwardA, forwardA);
    digitalWrite(motorForwardB, forwardB);
    digitalWrite(motorReverseA, reverseA);
    digitalWrite(motorReverseB, reverseB); */

		exportOnboardData(); // Export onboard data

		iterationTimer = millis();
	}
}

// Function to open the CVT
void openCVT(int revSpeed) {
	helixRead();
	
	if(helixPos > (helixMin + 5) && helixPos < (helixMax+50)) {	// Open the CVT if the helix position is within the limits
		if(revDirection != 1) {
			stopCVT();
    	digitalWrite(motorReverseA, HIGH); // motorReverseA ON
			ledcAttachPin(motorReverseB, 0);
			ledcWrite(0, revSpeed); // motorReverseA ON
			revDirection = 1;
		}	
	}	
	
	else if	(helixPos < (helixMin-5)){ // Close the CVT if the helix position is below the minimum limit
			if(revDirection != 0){
				stopCVT();
				digitalWrite(motorForwardA, HIGH);
				ledcAttachPin(motorForwardB, 0);
				ledcWrite(0, returnSpeed); // motorForwardA ON
				revDirection = 0;
			}
			Onboard.print("Helix PAST min..,");
			if(SerialBT.connected()) {
			SerialBT.print("Helix PAST min..,");
		}
	} 
	
	else { // Stop the CVT if the helix position is at the minimum limit
		Onboard.print("Helix at min..,");
		if(SerialBT.connected()) {
			SerialBT.print("Helix at min..,");
		}

		stopCVT();
	}

	if(SerialBT.connected()) {
		SerialBT.println("Opening..,");
	}
	Onboard.println("Opening..,");
}

// Function to close the CVT
void closeCVT(int fwdSpeed) {
	helixRead();
	
	if(helixPos < (helixMax-5) && helixPos > (helixMin-100)){ // Close the CVT if the helix position is within the limits

		if(revDirection != 0) {
			stopCVT();
      digitalWrite(motorForwardA, HIGH);
			ledcAttachPin(motorForwardB, 0);
			ledcWrite(0, fwdSpeed); // motorForwardA ON
			revDirection = 0;
		}
	}	
	
	else if(helixPos > (helixMax+5)){ // Open the CVT if the helix position is above the maximum limit
		if(revDirection != 1){
			stopCVT();
			digitalWrite(motorReverseA, HIGH);
			ledcAttachPin(motorReverseB, 0);
			ledcWrite(0, returnSpeed); // motorReverseA ON
			revDirection = 1;
		}
		Onboard.print("Helix PAST max..,");
		if(SerialBT.connected()) {
			SerialBT.print("Helix PAST max..,");
		}
	} 
	
	else	{ // Stop the CVT if the helix position is at the maximum limit
		Onboard.print("Helix at max..,");
		if(SerialBT.connected()) {
			SerialBT.print("Helix at max..,");
		}

		stopCVT();
	}

	Onboard.println("Closing..,");
	if(SerialBT.connected()) {
		SerialBT.println("Closing..,");
	}
}

// Function to stop the CVT
void stopCVT() {
	helixRead();
	ledcWrite(0, 0);
  digitalWrite(motorForwardA, LOW);
  digitalWrite(motorReverseA, LOW);
	ledcDetachPin(motorForwardB);
	ledcDetachPin(motorReverseB);
	digitalWrite(motorForwardB, LOW);
	digitalWrite(motorReverseB, LOW);
	delay(1);

	revDirection = 2;

	Onboard.println("Stopping..,");
	if(SerialBT.connected()) {
		SerialBT.println("Stopping..,");
	}
}

// Function to set the command RPM
void setCommandRPM(){
	potRead();

	//commandRpm = map(throttlePos, 0, 100, 1500, 2500);
	commandRpm = map(throttlePos, 0, 100, 0, 400);

	if((rpm+rpmVariance) < commandRpm) {
		openCVT(75);
	} else if((rpm-rpmVariance) > commandRpm) {
		closeCVT(75);
	} else {
		stopCVT();
	}

}

void setCommandHelix(int commandHelix) {
	if (helixPos < commandHelix-10) {
		closeCVT(100);
	} else if (helixPos > commandHelix+5) {
		openCVT(100);
	} else {
		stopCVT();
	}
}

// Function to read the helix position
void helixRead() {
	// Read the helix position from the AS5600 sensor
	rawHelix = map(Encoder.readAngle(),4095,0,0,4095);

	// Convert the raw helix position to degrees
	helixPos = ((rawHelix * AS5600_RAW_TO_DEGREES) - helixOffset);
}

void helixCalibrate(int duration) {

	int timeElapsed = 0;

	digitalWrite(motorReverseB, HIGH); // motorReverseB ON
	ledcWrite(1, 100); // motorReverseA ON

	while(millis() - timeElapsed < duration) {
		helixOffset = (Encoder.readAngle() * AS5600_RAW_TO_DEGREES);
	}

	stopCVT();

}

// Function to read the throttle position
void potRead() {

	/* if(rawThrottle > throttleMin && rawThrottle < 2950) {
		throttleMin = rawThrottle;
	} else if(rawThrottle < throttleMax) {
		throttleMax = rawThrottle;
	} */

	throttlePos = map(rawThrottle, throttleMin, throttleMax, 0, 100);

	if(throttlePos < 5) {
		throttlePos = 0;
	} else if(throttlePos > 100) {
		throttlePos = 100;
	}
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
		} else if (item == "Min"){
			helixMin = data.substring(dataIndex + 1).toInt();
		} else if (item == "RPM") {
			rpm = data.substring(dataIndex + 1).toInt();
		} else if (item == "Throttle") {
			rawThrottle = data.substring(dataIndex + 1).toInt();
		} else if (item == "Launch"){
			launchActive = data.substring(dataIndex + 1).toInt();
		} else if (item == "Helix"){
			helixPos = data.substring(dataIndex + 1).toInt();
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
			rpm = data.substring(dataIndex + 1).toInt();
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

	Onboard.print("Battery:"); // 0-1
	Onboard.print(rawBattery);
	Onboard.print(",");

	Onboard.print("RPM:"); // 0-1
	Onboard.print(rpm);
	Onboard.print(",");

	Onboard.print("Throttle:"); // 0-1
	Onboard.print(throttlePos);
	Onboard.print(",");

	Onboard.print("Raw Throttle:"); // 0-1
	Onboard.print(rawThrottle);
	Onboard.print(",");

	Onboard.print("Helix:"); // 0-1
	Onboard.print(helixPos);
	Onboard.print(",");

	Onboard.println("Raw Helix:"); // 0-1
	Onboard.print(rawHelix);
	Onboard.println(",");
}

void exportBluetoothData(){
	SerialBT.print("Forward A:"); // 0-1
	SerialBT.print(digitalRead(motorForwardA));;
	SerialBT.print(",");

	SerialBT.print("Forward B:"); // 0-1
	SerialBT.print(digitalRead(motorForwardB));
	SerialBT.print(",");

	SerialBT.print("Reverse A:"); // 0-1
	SerialBT.print(digitalRead(motorReverseA));
	SerialBT.print(",");

	SerialBT.print("Reverse B:"); // 0-1
	SerialBT.print(digitalRead(motorReverseB));
	SerialBT.print(",");

	SerialBT.print("Battery:"); // 0-1
	SerialBT.print(rawBattery);
	SerialBT.print(",");

	SerialBT.print("RPM:"); // 0-1
	SerialBT.print(rpm);
	SerialBT.print(",");

	SerialBT.print("Commanded");
	SerialBT.print(commandRpm);
	SerialBT.print(",");

	SerialBT.print("Throttle:"); // 0-1
	SerialBT.print(throttlePos);
	SerialBT.print(",");

	SerialBT.print("Helix:"); // 0-1
	SerialBT.print(helixPos);
	SerialBT.print(",");

	SerialBT.print("Raw Helix:"); // 0-1
	SerialBT.print(rawHelix);
	SerialBT.print(",");

	SerialBT.print("Helix min:"); // 0-1
	SerialBT.print(helixMin);
	SerialBT.print(",");

	SerialBT.print("Helix max:"); // 0-1
	SerialBT.print(helixMax);
	SerialBT.print(",");

	SerialBT.print("Offset:"); // 0-1
	SerialBT.print(helixOffset);
	SerialBT.println(",");
}