// This file contains the code for the ecvtCode project.

#include <HardwareSerial.h>
#include <AS5600.h>

// Define two instances of HardwareSerial for two UART interfaces
HardwareSerial Onboard(0); // UART0
AS5600 Encoder; // AS5600 sensor

// Pin Assignments *DO NOT CHANGE* ----------------------------------------------------------------------------

// High Power Outputs
#define motorForwardA 32  // forward + mosfet
#define motorForwardB 26  // forward - mosfet
#define motorReverseA 25  // reverse + mosfet
#define motorReverseB 33  // reverse - mosfet

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
unsigned long pulseTime;
unsigned long prevPulseTime;
unsigned long magFreq;
const int magnets = 3;
int rpm = 0;
unsigned long magInterval = 0;

// Launch variables
int launchPrevState;
unsigned long launchDebounce;
int launchInterval;
int launchFlag;
int launchActive = 0; // This is (1) when the CVT needs to be open/system is in launch mode

// Brake variables
int brakePrevState;
unsigned long brakeDebounce;
int brakeStatus = 0;  // This is (1) when the brake has been pressed for 1/2 second or more

// Throttle variables
int rawThrottle;
int throttlePos = 0;
int throttleMin = 4095;
int throttleMax = 0;

// Helix variables
int commandHelix;
float rawHelix;
int helixPos = 0;
int helixMax = 100;
int helixMin = 0;
int helixOffset = 0;

// Onboard communication
String incomingOnboardData = "";

// Global variables
const int debounce = 100;
unsigned long iterationTimer = 0;
const int iterationInterval = 5;
int revDirection = 0;	// 0 = forward, 1 = reverse


// Function prototypes --------------------------------------------------------------------
void openCVT(int revSpeed);
void closeCVT(int fwdSpeed);
void stopCVT();
void stopPWM(int channel);
void setCommandRPM();
void helixRead();
void potRead();
void batRead();
void updateBatAverage(int newValue);
void RPMRead();
void rpmCalculate();
void launch();
void brake();
void readOnboardData();
void processOnboardData(String data);
void exportOnboardData();

// Functions ---------------------------------------------------------------------------

void setup() {
	// Initialize the Onboard serial communication at a baud rate of 115200
	Onboard.begin(115200, SERIAL_8N1, linkRx, linkTx);

	// Initialize the AS5600 sensor
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
	ledcSetup(0, 1000, 8);
	ledcSetup(1, 1000, 8);

	ledcAttachPin(motorForwardA, 0);
	ledcAttachPin(motorReverseA, 1);

	// Set the pin modes for the sensor inputs
	pinMode(engineSensor, INPUT);
	pinMode(batterySensor, INPUT);

	// Set the pin modes for the brake input and launch button
	pinMode(brakeInput, INPUT);
	pinMode(launchButton, INPUT_PULLUP);

	// Attach an interrupt to the engine sensor pin to detect RPM changes
	attachInterrupt(digitalPinToInterrupt(engineSensor), RPMRead, FALLING);
}

// Loop function
void loop() {
	readOnboardData(); // Read onboard data

	//launch(); // Check launch conditions
	brake(); // Check brake conditions
	potRead(); // Read throttle position

	// Move helix based on throttle position
	if(launchActive == 0) {
		setCommandRPM();
	} else if (launchActive == 1){
		openCVT(255);
	}

	// Perform tasks at a specific interval
	if (millis() - iterationTimer >= iterationInterval) {
		batRead();
		helixRead();

		exportOnboardData();

		iterationTimer = millis();
	}
}

// Function to open the CVT
void openCVT(int revSpeed) {
	helixRead();
	
	if(helixPos > helixMin) {
		if(revDirection != 1) {
			stopPWM(0); // motorForwardA OFF
			digitalWrite(motorForwardB, LOW); // motorForwardB OFF
			delay(1);
			digitalWrite(motorReverseB, HIGH); // motorReverseB ON
			ledcWrite(1, revSpeed); // motorReverseA ON
			revDirection = 1;
		}	
	}	else	{
		stopCVT();
	}
}

// Function to close the CVT
void closeCVT(int fwdSpeed) {
	helixRead();
	
	if(helixPos < helixMax){

		if(revDirection != 0) {
			stopPWM(1); // motorReverseA OFF
			digitalWrite(motorReverseB, LOW);
			delay(1);
			digitalWrite(motorForwardB, HIGH);
			ledcWrite(0, fwdSpeed); // motorForwardA ON
			revDirection = 0;
		}
	}	else	{
		stopCVT();
	}
}

// Function to stop the CVT
void stopCVT() {
	stopPWM(0); // motorForwardA OFF
	stopPWM(1); // motorReverseA OFF
	delay(1);
	digitalWrite(motorForwardB, HIGH);
	digitalWrite(motorReverseB, HIGH);

	revDirection = 2;
}

// Function to stop the PWM output
void stopPWM(int channel) {
  ledcWrite(channel, 0); // Set duty cycle to 0 to stop PWM output
}

// Function to set the command RPM
void setCommandRPM(){
	potRead();

	commandRpm = map(throttlePos, 0, 100, 1200, 3200);

	if((rpm+50) < commandRpm) {
		openCVT(150);
	} else if((rpm-50) > commandRpm) {
		closeCVT(150);
	} else {
		stopCVT();
	}

}

void setCommandHelix() {
	if (helixPos < commandHelix) {
		closeCVT(150);
	} else if (helixPos > commandHelix) {
		openCVT(150);
	} else {
		stopCVT();
	}
}

// Function to read the helix position
void helixRead() {
	// Read the helix position from the AS5600 sensor
	rawHelix = (Encoder.readAngle() * AS5600_RAW_TO_DEGREES);

	helixPos = rawHelix - helixOffset;
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

	//rawThrottle = analogRead(A0);

	if(rawThrottle < throttleMin) {
		throttleMin = rawThrottle;
	} else if(rawThrottle > throttleMax) {
		throttleMax = rawThrottle;
	}

	throttlePos = map(rawThrottle, throttleMin, throttleMax, 0, 100);
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

// Function to read RPM from the engine sensor
void RPMRead() {
	pulseTime = micros();

	if (pulseTime < prevPulseTime) {
		// Handle rollover of pulse time
		magInterval = (4294967295 - prevPulseTime) + pulseTime;
	} else {
		magInterval = pulseTime - prevPulseTime;
	}

	if (magInterval > 3000) {
		rpmCalculate(); // Calculate RPM based on pulse interval
		prevPulseTime = pulseTime; // Store pulse time for next iteration
	}
}

// Function to calculate RPM based on pulse interval
void rpmCalculate() {
	if (magInterval > 0) {
		// Convert difference into frequency (seconds domain)
		magFreq = 60000000 / magInterval;
	}
	// Compensate for pulses per rotation
	rpm = magFreq / magnets;
}

// Function to handle launch button press
void launch() {
	int buttonState = digitalRead(launchButton);

	if (buttonState != launchPrevState) {
		launchDebounce = millis();
	}

	if (millis() - launchDebounce >= debounce) {
		if (buttonState == LOW && launchFlag == 0 && brakeStatus == 1 && rpm < 2000) {
			// Prime for launch if RPM is below 2000, brake is pressed, and button is pressed
			launchInterval = millis();
			launchFlag = 1;
		} else if ((millis() - launchInterval) >= 500 && buttonState == LOW && launchActive == 0 && launchFlag == 1) {
			// Activate launch if longer than 1/2 second
			launchActive = 1;
			launchFlag = 0;
		} else if (buttonState == HIGH) {
			// Reset launch state if button is released
			launchActive = 0;
			launchFlag = 0;
		}
	}

	launchPrevState = buttonState;
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

// Function to process onboard data received from UART
void processOnboardData(String data) {
	int dataIndex = data.indexOf(':');

	if (dataIndex != -1) {
		String item = data.substring(0, dataIndex);

		if (item == "RPM") {
			rpm = data.substring(dataIndex + 1).toInt();
		} else if (item == "Throttle") {
			rawThrottle = data.substring(dataIndex + 1).toInt();
		} else if (item == "Launch"){
			launchActive = data.substring(dataIndex + 1).toInt();
		} 
	}
}

// Function to export onboard data to UART
void exportOnboardData() {
	Onboard.print("Battery:"); // 0-1
	Onboard.print(batPercent);
	Onboard.print(",");

	Onboard.print("RPM:"); // 0-1
	Onboard.print(rpm);
	Onboard.print(",");

	Onboard.print("Launch:"); // 0-1
	Onboard.print(launchActive);
	Onboard.print(",");

	Onboard.print("Brake:"); // 0-1
	Onboard.print(brakeStatus);
	Onboard.print(",");

	Onboard.print("Throttle:"); // 0-1
	Onboard.print(throttlePos);
	Onboard.print(",");

	Onboard.print("Helix:"); // 0-1
	Onboard.print(helixPos);
	Onboard.println(",");
}
