#include <HardwareSerial.h>

// Define two instances of HardwareSerial for two UART interfaces
HardwareSerial Onboard(0); // UART0

// Pin Assignments *DO NOT CHANGE* ----------------------------------------------------------------------------

//#define linkRx 3
//#define linkTx 1
#define engineSensor  12
#define batterySensor 36
#define brakeInput    27
#define launchButton  15  //can change this one

#define motorForwardA 32  //forward + mosfet
#define motorForwardB 26  //forward - mosfet
#define motorReverseA 25  //reverse + mosfet
#define motorReverseB 33  //reverse - mosfet

#define SDA 21
#define SCL 22

// Variable declarations ----------------------------------------------------------------

//Battery variables:
int rawBattery;
int batPercent;

//engine RPM variables:
unsigned long pulseTime;
unsigned long prevPulseTime;
unsigned long magFreq;
const int magnets = 3;
int rpm = 0;
unsigned long magInterval = 0;

//launch variables
int launchPrevState;
unsigned long launchDebounce;
int launchInterval;
int launchFlag;
int launchActive = 0; //this is (1) when the CVT needs to be open/system is in launch mode

//brake variables
int brakePrevState;
unsigned long brakeDebounce;
int brakeStatus = 0;  //this is (1) when the brake has been pressed for 1/2 second or more

//program variables
int debounce = 100;

//communication variables
String incomingOnboardData = "";

//program variables
unsigned long iterationTimer = 0;
int iterationInterval = 100;

// Functions ---------------------------------------------------------------------------

void batRead(){

  rawBattery = analogRead(batterySensor);

  // calibrate the reading
  batPercent = map(rawBattery,1830,2680,0,99);

}

void RPMRead(){
  pulseTime = micros();

  if(pulseTime < prevPulseTime) {
    // Rollover detected
    magInterval = (4294967295 - prevPulseTime) + pulseTime;
  } 
  else {
    magInterval = pulseTime - prevPulseTime;
  }

  if(magInterval > 3000){
    rpmCalculate();
    //store pulse time
    prevPulseTime = pulseTime;
  }
}

void rpmCalculate(){

  if(magInterval > 0){
    //convert difference into frequency (seconds domain)
    magFreq = 60000000/(magInterval);
  }
  
  //compensate for pulses per rotation
  rpm = (magFreq/magnets);

}

void launch (){            //trigger for launch mode status with debounce and 1/2second delay
  int buttonState = digitalRead(launchButton);

  if(buttonState != launchPrevState){
    launchDebounce = millis();
  }
  
  if(millis()-launchDebounce >= debounce){
    
    if(buttonState == LOW && launchFlag == 0 && brakeStatus == 1 && rpm < 2000){  //prime for launch if RPM < 2000, brake is pressed, and button is pressed
      launchInterval = millis();
      launchFlag = 1;
    } 
    
    else if((millis() - launchInterval) >= 500 && buttonState == LOW && launchActive == 0 && launchFlag == 1){   //activate launch if longer then 1/2 second
      launchActive = 1;
      launchFlag = 0;
    } 

    else if(buttonState == HIGH){
      launchActive = 0;
      launchFlag = 0;
    }
  }

  launchPrevState = buttonState;

}

void brake(){ //will prevent blips from changing brake status (1/2 second debounce)

  int brakeState = digitalRead(brakeInput);

  if(brakeState != brakePrevState){
    brakeDebounce = millis();
  }

  if(millis()-brakeDebounce >= 500){
    if(brakeState == HIGH){
      brakeStatus = 1;
    }
    else{
      brakeStatus = 0;
    }
  }

  brakePrevState = brakeState;

}

void readOnboardData() {
  
  // Read incoming data and append it to the buffer
  while (Onboard.available() > 0) {
    char c = Onboard.read();
    
    // Check if a complete message has been received
    if (c == ',') {
      // Process the complete message
      processOnboardData(incomingOnboardData);

      // Clear the buffer for the next message
      incomingOnboardData = "";
    } else if(c != '\n'){
      incomingOnboardData += c;
    }
  }

}

void processOnboardData(String data) {
  int dataIndex = data.indexOf(':');
  if (dataIndex != -1) {
    String item = data.substring(0, dataIndex);
    
    if (item == "RPM:"){
      rpm = data.substring(dataIndex + 1).toInt();
    }

  }
}

void exportOnboardData() {
  Onboard.print("Brake:");
  Onboard.print(brakeStatus);
  Onboard.println(",");

  Onboard.print("Launch:");
  Onboard.print(launchActive);
  Onboard.println(",");

  Onboard.print("RPM:");
  Onboard.print(rpm);
  Onboard.println(",");

  // Onboard.print("Throttle:");
  // Onboard.print("0");
  // Onboard.println(",");

  // Onboard.print("Helix:");
  // Onboard.print("0");
  // Onboard.println(",");
}

void setup() {
  //Onboard.begin(115200, SERIAL_8N1, linkRx, linkTx);
  Serial.begin(115200);

  pinMode(motorForwardA, OUTPUT);
  pinMode(motorForwardB, OUTPUT);
  pinMode(motorReverseA, OUTPUT);
  pinMode(motorReverseB, OUTPUT);

	pinMode(engineSensor, INPUT);
	pinMode(batterySensor, INPUT);
	
  pinMode(brakeInput, INPUT);
	pinMode(launchButton, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(engineSensor), RPMRead, FALLING);

}

void loop() {
  
  //updates status
  batRead();  //updates batPercent between 0-99
  launch();   //updates launchActive to 0 (Inactive) or 1 (Active)
  brake();    //updates brakeStatus to 0 (Inactive) or 1 (Active)

    // Check if the counter is a multiple of 100
  if (millis() - iterationTimer >= iterationInterval) {

    Serial.print("batPercent:"); //0-1
    Serial.print(batPercent);
    Serial.println(",");

    Serial.print("RPM:"); //0-1
    Serial.print(rpm);
    Serial.println(",");

    Serial.print("launchStatus:"); //0-1
    Serial.print(launchActive);
    Serial.println(",");

    Serial.print("brakeStatus:"); //0-1
    Serial.print(brakeStatus);
    Serial.println(",");
    
  }

}
