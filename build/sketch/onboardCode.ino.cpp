#include <Arduino.h>
#line 1 "C:\\Users\\dying\\OneDrive - The University of Western Ontario\\Western Baja\\Github Code\\WBajaSAE\\onboardCode\\onboardCode.ino"

/*
    Name:       onboardCode.ino
    Created:	2/11/2024 12:08:47 AM
    Author:     WesternBajaSAE
*/

#include <HardwareSerial.h>

// Define two instances of HardwareSerial for two UART interfaces
HardwareSerial Phone(2);  // UART2
HardwareSerial Ecvt(0);   // UART0

// Pin Assignments *DO NOT CHANGE* ----------------------------------------------------------------------------

//High Power Outputs

#define revLight 32
#define rpmPower 33
#define fuelPower 25
#define varPower 26
#define varSignal 13
#define shockPower 27
#define phonePower 14

//Low Power Inputs

#define shockOne 2
#define shockTwo 15
#define shockThree 34
#define shockFour 39

#define engineSensor 23
#define wheelSensor 35
#define fuelSensor 22
#define batterySensor 36
#define revSensor 19

#define phoneRx 16
#define phoneTx 17
#define linkRx 3
#define linkTx 1

#define buttonOne 21
#define lapButton 18
#define muteButton 5
#define buttonFour 4

#define ON LOW
#define OFF HIGH

// Variable declarations ----------------------------------------------------------------

// Battery variables
int rawBattery;
int batPercent;

const int batLowLimit = 2150;
const int batHighLimit = 3180;

// Engine RPM variables:
unsigned long pulseTime;
unsigned long prevPulseTime;
unsigned long magFreq;
const int magnets = 3;
int rpm = 0;

unsigned long magInterval = 0;

// Wheel speed variables:
int speed;

// Fuel variables:
unsigned long fuelStart;
unsigned long fuelEnd;
int fuelTime;
int rawFuel = 0;
int fuelFlag = 0;
int fuel = 0;

const int fuelLowLimit = 800;
const int fuelHighLimit = 2000;

// Shock position variables:
int rawLeftFront = 0;
int rawRightFront = 0;
int rawLeftRear = 0;
int rawRightRear = 0;

int leftFront = 0;
int rightFront = 0;
int leftRear = 0;
int rightRear = 0;

// Driver input variables:
int panicStatus = 0;
int muteStatus = 0;
int lapReset = 0;
int button4 = 0;

int panicActive = 0;

int lapFlag;
int muteFlag;
int panicCount = 0;

unsigned long lapInterval;
unsigned long muteInterval;

unsigned long muteDebounce;
unsigned long lapDebounce;

int lapPrevState;
int mutePrevState;

// ECVT communication
String incomingEcvtData = "";
int brakeStatus = 0;
int launchStatus = 0;
int ecvtBat = 0;
int throttlePos = 0;
int helixPos = 0;
unsigned long ecvtExportTimer = 0;
const int ecvtExportInterval = 5;

// Phone communication
String incomingPhoneData = "";
int acknowledged = 0;
unsigned long phoneExportTimer = 0;
const int phoneExportInterval = 16;

// Global variables
int lapResetCount = 0;
const int batSamples = 1000;        // Number of samples to consider for moving average
const int fuelSamples = 1000;       // Number of samples to consider for moving average
int fuelLogger[fuelSamples];  // Array to store PWM samples
int batLogger[batSamples];   // Array to store PWM samples
int batIndex = 0;            // Index to keep track of the current sample
int fuelIndex = 0;         // Index to keep track of the current sample
const int debounce = 20;

// Function prototypes
void reverseCheck();
void batRead();
void updateBatAverage(int newValue);
void RPMRead();
void rpmCalculate();
void fuelRead();
void updateFuelAverage(int newValue);
void shockRead();
void muteButtonCheck();
void muteStatusUpdate();
void lapTimeReset();
void panicStatusUpdate();
void lapButtonCheck();
void readEcvtData();
void readPhoneData();
void processEcvtData(String data);
void processPhoneData(String data);
void exportPhoneData();
void exportEcvtData();

// The setup() function runs once each time the micro-controller starts
#line 164 "C:\\Users\\dying\\OneDrive - The University of Western Ontario\\Western Baja\\Github Code\\WBajaSAE\\onboardCode\\onboardCode.ino"
void setup();
#line 220 "C:\\Users\\dying\\OneDrive - The University of Western Ontario\\Western Baja\\Github Code\\WBajaSAE\\onboardCode\\onboardCode.ino"
void loop();
#line 398 "C:\\Users\\dying\\OneDrive - The University of Western Ontario\\Western Baja\\Github Code\\WBajaSAE\\onboardCode\\onboardCode.ino"
void batteryCheck();
#line 164 "C:\\Users\\dying\\OneDrive - The University of Western Ontario\\Western Baja\\Github Code\\WBajaSAE\\onboardCode\\onboardCode.ino"
void setup() {
  // Initialize serial communication with phone and eCVT
  Phone.begin(115200, SERIAL_8N1, phoneRx, phoneTx);
  Ecvt.begin(115200, SERIAL_8N1, linkRx, linkTx);
  
  // Initialize fuel samples array
  for (int i = 0; i < fuelSamples; i++) {
    fuelLogger[i] = fuelLowLimit;
  }
  
  // Initialize battery samples array
  for (int i = 0; i < batSamples; i++) {
    batLogger[i] = batLowLimit;
  }

  // Initialize regulator pins
  pinMode(revLight, OUTPUT);
  pinMode(rpmPower, OUTPUT);
  pinMode(fuelPower, OUTPUT);
  pinMode(varPower, OUTPUT);
  pinMode(varSignal, OUTPUT);
  pinMode(shockPower, OUTPUT);
  pinMode(phonePower, OUTPUT);

  // Initialize sensor pins
  pinMode(shockOne, INPUT);
  pinMode(shockTwo, INPUT);
  pinMode(shockThree, INPUT);
  pinMode(shockFour, INPUT);

  pinMode(engineSensor, INPUT);
  pinMode(wheelSensor, INPUT);
  pinMode(fuelSensor, INPUT);
  pinMode(batterySensor, INPUT);
  pinMode(revSensor, INPUT_PULLUP);

  pinMode(buttonOne, INPUT_PULLUP);
  pinMode(lapButton, INPUT_PULLUP);
  pinMode(muteButton, INPUT_PULLUP);
  pinMode(buttonFour, INPUT_PULLUP);

  // Control initial power output
  digitalWrite(revLight, OFF);
  digitalWrite(rpmPower, ON);
  digitalWrite(fuelPower, ON);
  digitalWrite(varPower, OFF);
  digitalWrite(varSignal, LOW);
  digitalWrite(shockPower, ON);
  digitalWrite(phonePower, ON);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(engineSensor), RPMRead, FALLING);
  attachInterrupt(digitalPinToInterrupt(fuelSensor), fuelRead, CHANGE);
}

// The loop() function runs continuously after setup()
void loop() {
  // Read data from eCVT and phone
  readEcvtData();
  readPhoneData();
  
  // Check lap button and mute button
  reverseCheck();
  lapButtonCheck();
  muteButtonCheck();
  shockRead();

  // Check if it's time to export data to phone
  if (millis() - phoneExportTimer >= phoneExportInterval) {
    // Run non-time critical functions

    // Export data to phone
    exportPhoneData();

    phoneExportTimer = millis();
  }

  // Check if it's time to export data to eCVT
  if (millis() - ecvtExportTimer >= ecvtExportInterval) {
    batRead();
    
    // Export data to eCVT
    exportEcvtData();

    ecvtExportTimer = millis();
  }
}

// Function to check reverse sensor and update reverse light
void reverseCheck() {
  if (digitalRead(revSensor) == LOW) {
    digitalWrite(revLight, ON);
  } else {
    digitalWrite(revLight, OFF);
  }
}

// Function to read battery voltage and calculate battery percentage
void batRead() {

  int rawBattery = analogRead(batterySensor);

  // Calibrate the reading
  if(rawBattery > batLowLimit){
    updateBatAverage(rawBattery);
  }
  
}

// Function to update moving average of PWM samples
void updateBatAverage(int newValue) {
  batLogger[batIndex] = newValue;
  batIndex = (batIndex + 1) % batSamples;
  long sum = 0;

  for (int i = 0; i < batSamples; i++) {
    sum += batLogger[i];
  }

  batPercent = map((sum / batSamples), batLowLimit, batHighLimit, 0, 99);
}

// Function to calculate RPM from pulse time
void RPMRead() {
  pulseTime = micros();

  if (pulseTime < prevPulseTime) {
    // Rollover detected
    magInterval = (4294967295 - prevPulseTime) + pulseTime;
  } else {
    magInterval = pulseTime - prevPulseTime;
  }

  if (magInterval > 3000) {
    rpmCalculate();
    // Store pulse time
    prevPulseTime = pulseTime;
  }
}

// Function to calculate RPM from magnetic interval
void rpmCalculate() {
  if (magInterval > 0) {
    // Convert difference into frequency (seconds domain)
    magFreq = 60000000 / magInterval;
  }

  // Compensate for pulses per rotation
  rpm = magFreq / magnets;
}

// Function to read fuel sensor and update fuel level
void fuelRead() {
  if (digitalRead(fuelSensor) == HIGH && fuelFlag == 0) {
    fuelStart = micros();
    fuelFlag = 1;
  } else if (digitalRead(fuelSensor) == LOW && fuelFlag == 1) {
    fuelEnd = micros();
    fuelFlag = 0;
  }

  fuelTime = fuelEnd - fuelStart;

  if (fuelFlag == 0 && fuelTime > fuelLowLimit) {
    updateFuelAverage(fuelTime);
  }
}

// Function to update moving average of PWM samples
void updateFuelAverage(int newValue) {
  fuelLogger[fuelIndex] = newValue;
  fuelIndex = (fuelIndex + 1) % fuelSamples;

  long sum = 0;

  for (int i = 0; i < fuelSamples; i++) {
    sum += fuelLogger[i];
  }

  rawFuel = sum / fuelSamples;

  fuel = map(rawFuel, fuelLowLimit, fuelHighLimit, 0, 99);
}

  // Function to read shock sensor values
void shockRead() {
  rawLeftFront = analogRead(shockOne);
  rawRightFront = analogRead(shockTwo);
  rawLeftRear = analogRead(shockThree);
  rawRightRear = analogRead(shockFour);

  leftFront = map(leftFront, 900, 4095, 0, 100);
  rightFront = map(rightFront, 900, 4095, 0, 100);
  leftRear = map(leftRear, 0, 4095, 0, 100);
  rightRear = map(rightRear, 0, 4095, 0, 100);
}

  // Function to check mute button state and update mute status
void muteButtonCheck() {
  int buttonState = digitalRead(muteButton);

  if (buttonState != mutePrevState) {
    muteDebounce = millis();
  }

  if (millis() - muteDebounce >= debounce) {
    if (buttonState == 0 && muteFlag == 0) {
      muteFlag = 1;
      muteInterval = millis();
    }

    if ((millis() - muteInterval) <= 1000 && buttonState == HIGH && muteFlag == 1) {
      muteStatusUpdate();
      muteFlag = 0;
    } else if ((millis() - muteInterval) > 1000 && buttonState == LOW && muteFlag == 1 && launchStatus == 0) {
      launchStatus = 1;
    } else if (buttonState == HIGH && muteFlag == 1) {
      muteFlag = 0;
      launchStatus = 0;
    }
  }

  mutePrevState = buttonState;
}

  // Function to update mute status
void muteStatusUpdate() {
  if (muteStatus == 0) {
    muteStatus = 1;
  } else {
    muteStatus = 0;
  }
}

void batteryCheck() {
  if (batPercent < 10) {
    panicStatusUpdate();
    panicActive = 1;
  }
}

  // Function to reset lap timer
void lapTimeReset() {
  if (lapReset == 0) {
    lapReset = 1;
  } else {
    lapReset = 0;
  }
}

  // Function to update panic status
void panicStatusUpdate() {
  if (panicStatus == 0) {
    panicStatus = 1;
  } else {
    panicStatus = 0;
  }
}

  // Function to check lap button state and perform corresponding actions
void lapButtonCheck() {
  int buttonState = digitalRead(lapButton);

  if (buttonState != lapPrevState) {
    lapDebounce = millis();
  }

  if (millis() - lapDebounce >= debounce) {
    if (buttonState == LOW && lapFlag == 0 && panicActive == 0) {
      lapFlag = 1;
      lapInterval = millis();
    } else if ((millis() - lapInterval) >= 2000 && buttonState == LOW && lapFlag == 1 && panicActive == 0) {
      // Activate panic if longer than 2 seconds
      panicStatusUpdate();
      panicActive = 1;
    } else if ((millis() - lapInterval) <= 1000 && buttonState == HIGH && lapFlag == 1) {
      // Reset lap timer if less than 1 second
      lapTimeReset();
      lapFlag = 0;
    } else if (buttonState == HIGH && lapFlag == 1) {
      lapFlag = 0;
      panicActive = 0;
    }
  }

  lapPrevState = buttonState;
}

  // Function to read data from eCVT
void readEcvtData() {
  // Read incoming data and append it to the buffer
  while (Ecvt.available() > 0) {
    char c = Ecvt.read();

    // Check if a complete message has been received
    if (c == ',') {
      // Process the complete message
      processEcvtData(incomingEcvtData);

      // Clear the buffer for the next message
      incomingEcvtData = "";
    } else if(c != '\n'){
      incomingEcvtData += c;
    }
  }
}

  // Function to read data from phone
void readPhoneData() {
  // Read incoming data and append it to the buffer
  while (Phone.available() > 0) {
    char c = Phone.read();

    // Check if a complete message has been received
    if (c == ',') {
      // Process the complete message
      processPhoneData(incomingPhoneData);

      // Clear the buffer for the next message
      incomingPhoneData = "";
    } else if(c != '\n'){
      incomingPhoneData += c;
    }
  }
}

  // Function to process eCVT data
void processEcvtData(String data) {
  int dataIndex = data.indexOf(':');
  if (dataIndex != -1) {
    String item = data.substring(0, dataIndex);

    if (item == "Brake"){
      brakeStatus = data.substring(dataIndex + 1).toInt();
    } else if(item == "Battery"){
      ecvtBat = data.substring(dataIndex + 1).toInt();
    } else if(item == "Helix"){
      helixPos = data.substring(dataIndex + 1).toInt();
    }
  }
}

  // Function to process phone data
void processPhoneData(String data) {
  int dataIndex = data.indexOf(':');
  if (dataIndex != -1) {
    String item = data.substring(0, dataIndex);

    if (item == "Confirm"){
      acknowledged = data.substring(dataIndex + 1).toInt();
    }
  }
}

  // Function to export data to phone
void exportPhoneData() {
  Phone.print("RPM:");  // 0-3800
  Phone.print(rpm);
  Phone.print(",");

  Phone.print("Speed:");   // 0-40
  Phone.print(speed);
  Phone.print(",");

  Phone.print("Battery:");  // 0-100
  Phone.print(batPercent);
  Phone.print(",");

  Phone.print("Fuel:");  // 0-100
  Phone.print(fuel);
  Phone.print(",");

  // Phone.print("LF:");       // 0-100
  // Phone.print(leftFront);
  // Phone.print(",");

  Phone.print("RF:");  // 0-100
  Phone.print(rightFront);
  Phone.print(",");

  // Phone.print("LB:");       // 0-100
  // Phone.print(leftRear);
  // Phone.print(",");

  // Phone.print("RB:");       // 0-100
  // Phone.print(rightRear);
  // Phone.print(",");

  Phone.print("panic:");  // 0-1
  Phone.print(panicStatus);
  Phone.print(",");

  Phone.print("mute:");  // 0-1
  Phone.print(muteStatus);
  Phone.print(",");

  Phone.print("lapTimer:");  // 0-1
  Phone.print(lapReset);
  Phone.print(",");

  Phone.print("eCVT:");  // 0-1
  Phone.print(ecvtBat);
  Phone.print(",");

  Phone.print("Launch:");  // 0-1
  Phone.print(launchStatus);
  Phone.println(",");
}

  // Function to export data to eCVT
void exportEcvtData() {
  Ecvt.print("Throttle:");
  Ecvt.print(rawRightFront);
  Ecvt.print(",");

  Ecvt.print("RPM:");
  Ecvt.print(rpm);
  Ecvt.print(",");

  Ecvt.print("Launch:");
  Ecvt.print(launchStatus);
  Ecvt.println(",");
}