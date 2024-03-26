
/*
    Name:       NewBoardCode.ino
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

//battery variables
int rawBattery;
int batPercent;

//engine RPM variables:
unsigned long pulseTime;
unsigned long prevPulseTime;
unsigned long magFreq;
const int magnets = 3;
int rpm = 0;

unsigned long magInterval = 0;

//wheel speed variables:
int speed;

//Fuel variables:
unsigned long fuelStart;
unsigned long fuelEnd;
unsigned long fuelTime;
int rawFuel = 0;
int fuelFlag = 0;
int fuel = 0;

const int fuelLowLimit = 620;
const int fuelHighLimit = 2500;

//Shock position variables:
int leftFront = 0;
int rightFront = 0;
int leftRear = 0;
int rightRear = 0;

//Driver input variables:
int panicStatus = 0;
int muteStatus = 0;
int lapReset = 0;
int button4 = 0;

int panicActive = 0;

int lapFlag;
int muteFlag;

unsigned long lapInterval;
unsigned long muteInterval;

unsigned long muteDebounce;
unsigned long lapDebounce;
const int debounce = 20;

int lapPrevState;
int mutePrevState;

//ECVT communication
String incomingEcvtData = "";
int brakeStatus = 0;
int launchStatus = 0;
int ecvtBat = 0;
int throttlePos = 0;
int helixPos = 0;
unsigned long ecvtExportTimer = 0;
const int ecvtExportInterval = 5;

//Phone communication
String incomingPhoneData = "";
int acknowledged = 0;
unsigned long phoneExportTimer = 0;
const int phoneExportInterval = 16;

//Program running stuff

int panicCount = 0;
int lapResetCount = 0;
const int numSamples = 200;  // Number of samples to consider for moving average
int pwmSamples[numSamples];  // Array to store PWM samples
int sampleIndex = 0;         // Index to keep track of the current sample


// Functions ---------------------------------------------------------------------------

void reverseCheck() {
  if (digitalRead(revSensor) == LOW) {
    digitalWrite(revLight, ON);
  } else {
    digitalWrite(revLight, OFF);
  }
}

void batRead() {

  rawBattery = analogRead(batterySensor);

  // calibrate the reading
  batPercent = map(rawBattery, 2150, 3150, 0, 99);
}

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
    //store pulse time
    prevPulseTime = pulseTime;
  }
}

void rpmCalculate() {

  if (magInterval > 0) {
    //convert difference into frequency (seconds domain)
    magFreq = 60000000 / (magInterval);
  }

  //compensate for pulses per rotation
  rpm = (magFreq / magnets);
}

// Function to update moving average
void updateMovingAverage(int newValue) {
  pwmSamples[sampleIndex] = newValue;
  sampleIndex = (sampleIndex + 1) % numSamples;
  int sum = 0;

  for (int i = 0; i < numSamples; i++) {
    sum += pwmSamples[i];
  }
  fuel = sum / numSamples;
}

void fuelRead() {

  if (digitalRead(fuelSensor) == HIGH && fuelFlag == 0) {
    fuelStart = micros();
    fuelFlag = 1;
  } else if (digitalRead(fuelSensor) == LOW && fuelFlag == 1) {
    fuelEnd = micros();
    fuelFlag = 0;
  }

  fuelTime = fuelEnd - fuelStart;

  rawFuel = map(fuelTime, fuelLowLimit, fuelHighLimit, 0, 100);
  if (rawFuel > 0) {
    updateMovingAverage(rawFuel);
  }
}

void shockRead() {
  leftFront = analogRead(shockOne);
  rightFront = analogRead(shockTwo);
  leftRear = analogRead(shockThree);
  rightRear = analogRead(shockFour);

  leftFront = map(leftFront, 900, 4095, 0, 100);
  rightFront = map(rightFront, 900, 4095, 0, 100);
  leftRear = map(leftRear, 0, 4095, 0, 100);
  rightRear = map(rightRear, 0, 4095, 0, 100);
}

void muteButtonCheck() {  //interrupt function to be held to turn off mute on phone
  int buttonState = digitalRead(muteButton);

  if (buttonState != mutePrevState) {
    muteDebounce = millis();
  }

  if (millis() - muteDebounce >= debounce) {

    if (buttonState == 0 && muteFlag == 0) {
      muteFlag = 1;
      muteInterval = millis();
    }

    if ((millis() - muteInterval) <= 1000 && buttonState == HIGH && muteFlag == 1) {  //activate panic if longer than 2 seconds
      muteStatusUpdate();
      muteFlag = 0;
    } else if ((millis() - muteInterval) > 1000 && buttonState == HIGH && muteFlag == 1) {
      muteFlag = 0;
    }
  }

  mutePrevState = buttonState;
}

void muteStatusUpdate() {

  if (muteStatus == 0) {
    muteStatus = 1;
  } else {
    muteStatus = 0;
  }

}

void lapTimeReset() {  //interrupt function to reset the lap timer on phone

  if (lapReset == 0) {
    lapReset = 1;
  } else {
    lapReset = 0;
  }

}

void panicStatusUpdate() {
  
  if (panicStatus == 0) {
    panicStatus = 1;
  } else {
    panicStatus = 0;
  }

}

void lapButtonCheck() {  //interrupt function to press to start panic stuff on phone and send messages
  int buttonState = digitalRead(lapButton);

  if (buttonState != lapPrevState) {
    lapDebounce = millis();
  }

  if (millis() - lapDebounce >= debounce) {

    if (buttonState == LOW && lapFlag == 0 && panicActive == 0) {
      lapFlag = 1;
      lapInterval = millis();
    }

    else if ((millis() - lapInterval) >= 2000 && buttonState == LOW && lapFlag == 1 && panicActive == 0) {  //activate panic if longer than 2 seconds
      panicStatusUpdate();
      panicActive = 1;
    }

    else if ((millis() - lapInterval) <= 1000 && buttonState == HIGH && lapFlag == 1) {  //activate lap reset if less than 1 second
      lapTimeReset();
      lapFlag = 0;
    }

    else if (buttonState == HIGH && lapFlag == 1) {
      lapFlag = 0;
      panicActive = 0;
    }
  }

  lapPrevState = buttonState;
}

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

void processEcvtData(String data) {
  int dataIndex = data.indexOf(':');
  if (dataIndex != -1) {
    String item = data.substring(0, dataIndex); // Extract temperature substring
    
    if (item == "Brake:"){
      brakeStatus = data.substring(dataIndex + 1).toInt();
    } else if(item == "Launch:"){
      launchStatus = data.substring(dataIndex + 1).toInt();
    } else if(item == "Battery"){
      ecvtBat = data.substring(dataIndex + 1).toInt();
    } else if(item == "RPM"){
      rpm = data.substring(dataIndex + 1).toInt();
    } else if(item == "Throttle"){
      throttlePos = data.substring(dataIndex + 1).toInt();
    } else if(item == "Helix"){
      helixPos = data.substring(dataIndex + 1).toInt();
    } 
  }
}

void processPhoneData(String data) {
  int dataIndex = data.indexOf(':');
  if (dataIndex != -1) {
    String item = data.substring(0, dataIndex);

    if (item == "Confirmed:"){
      acknowledged = data.substring(dataIndex + 1).toInt();
    }
  }
}

void exportPhoneData() {

  Phone.print("RPM:");  //0-3800
  Phone.print(rpm);
  Phone.print(",");

  // Phone.print("Speed:");   //0-40
  // Phone.print(speed);
  // Phone.print(",");

  Phone.print("Battery:");  //0-100
  Phone.print(batPercent);
  Phone.print(",");

  Phone.print("Fuel:");  //0-100
  Phone.print(fuel);
  Phone.print(",");

  // Phone.print("LF:");       //0-100
  // Phone.print(leftFront);
  // Phone.print(",");

  Phone.print("RF:");  //0-100
  Phone.print(rightFront);
  Phone.print(",");

  // Phone.print("LB:");       //0-100
  // Phone.print(leftRear);
  // Phone.print(",");

  // Phone.print("RB:");       //0-100
  // Phone.print(rightRear);
  // Phone.print(",");

  Phone.print("panic:");  //0-1
  Phone.print(panicStatus);
  Phone.print(",");

  Phone.print("mute:");  //0-1
  Phone.print(muteStatus);
  Phone.print(",");

  Phone.print("lapTimer:");  //0-1
  Phone.print(lapReset);
  Phone.print(",");

  // Phone.print("eCVT:");
  // Phone.print(ecvtBat);
  // Phone.print(",");
  
  // Phone.print("Launch:");
  // Phone.print(launchStatus);
  // Phone.println(",");

}

void exportEcvtData() {
  Ecvt.print("RPM:");
  Ecvt.print(rpm);
  Ecvt.println(",");
}

// The setup() function runs once each time the micro-controller starts
void setup() {
  Phone.begin(115200, SERIAL_8N1, phoneRx, phoneTx);
  Ecvt.begin(115200, SERIAL_8N1, linkRx, linkTx);

  for (int i = 0; i < numSamples; i++) {
    pwmSamples[i] = 0;
  }

  // Initialize regulator pins *DO NOT CHANGE*
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
  digitalWrite(phonePower, OFF);

  attachInterrupt(digitalPinToInterrupt(engineSensor), RPMRead, FALLING);
  attachInterrupt(digitalPinToInterrupt(fuelSensor), fuelRead, CHANGE);  //reverse program
}

// Add the main program code into the continuous loop() function
void loop() {

  readEcvtData();
  readPhoneData();
  
  lapButtonCheck();
  muteButtonCheck();

  // Check if time to export to phone
  if (millis() - phoneExportTimer >= phoneExportInterval) {

    //run non-time critical functions
    batRead();
    shockRead();

    //export to phone
    exportPhoneData();
    
    phoneExportTimer = millis();
  }

  // Check if time to export to ecvt
  if (millis() - ecvtExportTimer >= ecvtExportInterval) {

    //export to ecvt
    exportEcvtData();

    ecvtExportTimer = millis();
  }
}
