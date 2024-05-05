# 1 "C:\\Users\\dying\\OneDrive - The University of Western Ontario\\Western Baja\\Github Code\\WBajaSAE\\dctProtoCode\\dctProtoCode.ino"
// Includes
# 3 "C:\\Users\\dying\\OneDrive - The University of Western Ontario\\Western Baja\\Github Code\\WBajaSAE\\dctProtoCode\\dctProtoCode.ino" 2

// Define serial communication
HardwareSerial Onboard(0); // UART0
String incomingOnboardData = "";

// OUTPUT PIN ASSIGNMENTS





// ANALOG INPUT PIN ASSIGNMENTS


// DIGITAL INPUT PIN ASSIGNMENTS
# 30 "C:\\Users\\dying\\OneDrive - The University of Western Ontario\\Western Baja\\Github Code\\WBajaSAE\\dctProtoCode\\dctProtoCode.ino"
// Physical position setpoints
int motorDown = 1000;
int motorMid = 2000;
int motorUp = 3000;

// Physical state variables
int engineRpm = 3600; // Engine RPM
int drumPos = 0;

int reverseFlag = 0;

int inputGear = 0; // -1 = Reverse, 0 = Neutral, 1 = 1st gear, 2 = 2nd gear, 3 = 3rd gear, 4 = 4th gear, 5 = 5th gear

int downFlag = 0;
int upFlag = 0;

int motorCommand = motorMid;

// Automatic shifting variables
int mode = 0; // 0 = Manual, 1 = Automatic
int upshiftFlag = 0;
int downshiftFlag = 0;

// PID control parameters
const float Kp = 1.0; // Proportional gain
const float Ki = 0.0; // Integral gain
const float Kd = 0.0; // Derivative gain

const float Ti = 0.001; // Integral time constant
const float Td = 0.001; // Derivative time constant

int pidElapsedTime = 0; // Elapsed time since last PID update
unsigned long pidPrevTime = 0; // Previous time for PID update
float integral = 0.0; // Integral term for PID control
double previousError = 0.0; // Previous error for derivative term
int output = 0.0; // PID output

// Function prototypes
void setClutch(int gear);
void drumShift(int desiredDrum);
void shiftController(int commandPos);
void getDrumPos();
void shiftUp();
void shiftDown();
void autoShift();
void readOnboardData();
void processOnboardData(String data);

void setup() {
  // Set solenoid pins as outputs
  pinMode(2 /* Solenoid 1 connected to pin 2*/, 0x03);
  pinMode(3 /* Solenoid 2 connected to pin 3*/, 0x03);
  pinMode(4 /* Shift motor connected to pin 4*/, 0x03);
  pinMode(5 /* Shift motor connected to pin 5*/, 0x03);

  // Set gear position sensor pin as input
  pinMode(6 /* Gear position sensor connected to pin 6*/, 0x01);

  // Set switch pins as inputs
  pinMode(25 /* Reverse switch connected to pin 7*/, 0x01);
  pinMode(17 /* Neutral switch connected to pin 8*/, 0x01);
  pinMode(27 /* 1-2 shift switch connected to pin 9*/, 0x01);
  pinMode(19 /* 2-N shift switch connected to pin 10*/, 0x01);
  pinMode(26 /* 2-3 shift switch connected to pin 11*/, 0x01);
  pinMode(22 /* 3-N shift switch connected to pin 12*/, 0x01);
  pinMode(2 /* 3-4 shift switch connected to pin 13*/, 0x01);
  pinMode(24 /* 4-N shift switch connected to pin 14*/, 0x01);
  pinMode(12 /* 4-5 shift switch connected to pin 15*/, 0x01);
  pinMode(13 /* 5-N shift switch connected to pin 16*/, 0x01);

  // Get initial drum position
  getDrumPos();

  // Initialize serial communication
  Onboard.begin(9600);
}

void loop() {
  // Read data from UART & Bluetooth
  readOnboardData();

  if(mode == 0){
    setGear(inputGear);
  }

  if(mode == 1){
    autoShift();

    if(upshiftFlag == 1){
      setGear(drumPos + 1);
    }
    else if(downshiftFlag == 1){
      setGear(drumPos - 1);
    }
  }
}

void setGear(int gear) {
  getDrumPos();

  int commandGear;

  if(gear < drumPos && drumPos > 0) {
    commandGear = drumPos - 1;
  } else if (gear > drumPos && drumPos < 5) {
    commandGear = drumPos + 1;
  } else if (gear < drumPos && drumPos == 0 && engineRpm < 1600) {
    commandGear = drumPos - 1;
  } else {
    commandGear = drumPos;
  }

  if(commandGear > 0){
    reverseFlag = 0;
  } else {
    reverseFlag = 1;
  }

  switch(commandGear) {
    case -1:
      drumShift(-1);
      setClutch(-1);
      break;
    case 0:
      drumShift(0);
      setClutch(0);
      break;
    case 1:
      setClutch(1);
      drumShift(1);
      break;
    case 2:
      setClutch(2);
      drumShift(2);
      break;
    case 3:
      setClutch(3);
      drumShift(3);
      break;
    case 4:
      setClutch(4);
      drumShift(4);
      break;
    case 5:
      setClutch(5);
      drumShift(5);
      break;
    default:
      motorCommand = motorMid;
      Onboard.print("ERROR: Invalid gear setpoint (" + gear);
      Onboard.println(")");
      break;
  }

  shiftController(motorCommand);
}

// Sets clutches based on gear input & drum position
void setClutch(int gear) {
  getDrumPos();

  switch (gear) {
    case -1:
      if(digitalRead(25 /* Reverse switch connected to pin 7*/) == 0x1 && engineRpm < 1600) {
        digitalWrite(2 /* Solenoid 1 connected to pin 2*/, 0x0);
        digitalWrite(3 /* Solenoid 2 connected to pin 3*/, 0x0);
      }
      else if (digitalRead(25 /* Reverse switch connected to pin 7*/) == 0x1 && engineRpm > 1600){
        Onboard.println("ERROR: Current RPM does not match clutch setpoint (Reverse)");
      } else {
        Onboard.println("ERROR: Drum position does not match clutch setpoint (Reverse)");
      }
      break;
    case 0:
      if(digitalRead(17 /* Neutral switch connected to pin 8*/) == 0x1) {
        digitalWrite(2 /* Solenoid 1 connected to pin 2*/, 0x1);
        digitalWrite(3 /* Solenoid 2 connected to pin 3*/, 0x0);
      }
      else {
        Onboard.println("ERROR: Drum position does not match clutch setpoint (Neutral)");
      }
      break;
    case 1:
      if(digitalRead(27 /* 1-2 shift switch connected to pin 9*/) == 0x1) {
        digitalWrite(2 /* Solenoid 1 connected to pin 2*/, 0x0);
        digitalWrite(3 /* Solenoid 2 connected to pin 3*/, 0x1);
      }
      else {
        Onboard.println("ERROR: Drum position does not match clutch setpoint (1st gear)");
      }
      break;
    case 2:
      if(digitalRead(27 /* 1-2 shift switch connected to pin 9*/) == 0x1 || digitalRead(26 /* 2-3 shift switch connected to pin 11*/) == 0x1) {
        digitalWrite(2 /* Solenoid 1 connected to pin 2*/, 0x0);
        digitalWrite(3 /* Solenoid 2 connected to pin 3*/, 0x0);
      }
      else {
        Onboard.println("ERROR: Drum position does not match clutch setpoint (2nd gear)");
      }
      break;
    case 3:
      if(digitalRead(26 /* 2-3 shift switch connected to pin 11*/) == 0x1 || digitalRead(2 /* 3-4 shift switch connected to pin 13*/) == 0x1) {
        digitalWrite(2 /* Solenoid 1 connected to pin 2*/, 0x0);
        digitalWrite(3 /* Solenoid 2 connected to pin 3*/, 0x1);
      }
      else {
        Onboard.println("ERROR: Drum position does not match clutch setpoint (3rd gear)");
      }
      break;
    case 4:
      if(digitalRead(2 /* 3-4 shift switch connected to pin 13*/) || digitalRead(12 /* 4-5 shift switch connected to pin 15*/) == 0x1) {
        digitalWrite(2 /* Solenoid 1 connected to pin 2*/, 0x0);
        digitalWrite(3 /* Solenoid 2 connected to pin 3*/, 0x0);
      }
      else {
        Onboard.println("ERROR: Drum position does not match clutch setpoint (4th gear)");
      }
      break;
    case 5:
      if(digitalRead(12 /* 4-5 shift switch connected to pin 15*/) || digitalRead(25 /* Reverse switch connected to pin 7*/ /* 5th gear switch connected to pin 7*/) == 0x1) {
        digitalWrite(2 /* Solenoid 1 connected to pin 2*/, 0x0);
        digitalWrite(3 /* Solenoid 2 connected to pin 3*/, 0x1);
      }
      else {
        Onboard.println("ERROR: Drum position does not match clutch setpoint (5th gear)");
      }
      break;
    default:
      Onboard.print("ERROR: Invalid clutch setpoint (" + gear);
      Onboard.println(")");
      break;
  }
}

// Shifts drum to desired position
void drumShift(int desiredDrum) {

  // Get current motor position
  int motorPos = analogRead(6 /* Gear position sensor connected to pin 6*/);
  getDrumPos();

  // Calculate motor command based on desired shift position
  if(desiredDrum > drumPos) {

    if(motorPos < motorUp && downFlag == 0) {
      motorCommand = motorUp;
    }

    else if(motorPos > motorMid){
      downFlag = 1;
      motorCommand = motorMid;
    }

    else if(motorPos < motorMid && downFlag == 1){
      downFlag = 0;
    }
  }

  else if(desiredDrum < drumPos) {

    if(motorPos > motorDown && upFlag == 0) {
      motorCommand = motorDown;
    }

    else if(motorPos < motorMid){
      upFlag = 1;
      motorCommand = motorMid;
    }

    else if(motorPos > motorMid && upFlag == 1){
      upFlag = 0;
    }

  }

  else {
    motorCommand = motorMid;
  }
}

// PID controller to shift motor to desired position
void shiftController(int commandPos){

  // Calculate elapsed time since last PID update
  unsigned long currentTime = micros();
  if(currentTime < pidPrevTime) {
    pidElapsedTime = (currentTime + 4294967295) - pidPrevTime;
  } else {
    pidElapsedTime = currentTime - pidPrevTime;
  }

  // Get current shifter position
  int motorPos = analogRead(6 /* Gear position sensor connected to pin 6*/);

  // Calculate error
  double error = commandPos - motorPos;

  // Calculate integral term (approximate integral using trapezoidal rule)
  integral += (error + previousError) * pidElapsedTime / Ti;

  // Calculate derivative term
  double derivative = (error - previousError) / (pidElapsedTime / Td);

  // Compute PID output
  output = Kp * error + Ki * integral + Kd * derivative;

  // Update previous values for next iteration
  previousError = error;

  // Ensure output is within acceptable bounds (e.g., for PWM control)
  output = ((output)<(-255)?(-255):((output)>(255)?(255):(output)));

  // Update motor speed based on PID output
  if (output > 0) {
    analogWrite(4 /* Shift motor connected to pin 4*/, output);
    digitalWrite(5 /* Shift motor connected to pin 5*/, 0x0);
  } else if (output < 0) {
    digitalWrite(4 /* Shift motor connected to pin 4*/, 0x0);
    analogWrite(5 /* Shift motor connected to pin 5*/, output);
  } else {
    digitalWrite(4 /* Shift motor connected to pin 4*/, 0x0);
    digitalWrite(5 /* Shift motor connected to pin 5*/, 0x0);
  }
}

// Get drum position based on switch inputs
void getDrumPos(){
  if(digitalRead(25 /* Reverse switch connected to pin 7*/ == 0x1) && digitalRead(17 /* Neutral switch connected to pin 8*/ == 0x1) && digitalRead(27 /* 1-2 shift switch connected to pin 9*/ == 0x1) && digitalRead(26 /* 2-3 shift switch connected to pin 11*/ == 0x1) && digitalRead(2 /* 3-4 shift switch connected to pin 13*/ == 0x1) && digitalRead(12 /* 4-5 shift switch connected to pin 15*/ == 0x1) && digitalRead(25 /* Reverse switch connected to pin 7*/ /* 5th gear switch connected to pin 7*/ == 0x1) || digitalRead(25 /* Reverse switch connected to pin 7*/ == 0x0) && digitalRead(17 /* Neutral switch connected to pin 8*/ == 0x0) && digitalRead(27 /* 1-2 shift switch connected to pin 9*/ == 0x0) && digitalRead(26 /* 2-3 shift switch connected to pin 11*/ == 0x0) && digitalRead(2 /* 3-4 shift switch connected to pin 13*/ == 0x0) && digitalRead(12 /* 4-5 shift switch connected to pin 15*/ == 0x0) && digitalRead(25 /* Reverse switch connected to pin 7*/ /* 5th gear switch connected to pin 7*/ == 0x0)){
    drumPos = -2;
    Onboard.println("ERROR: Invalid drum position");
  }
  else if(digitalRead(25 /* Reverse switch connected to pin 7*/) == 0x1 && reverseFlag == 1) {
    drumPos = -1;
  }
  else if(digitalRead(17 /* Neutral switch connected to pin 8*/) == 0x1) {
    drumPos = 0;
  }
  else if(digitalRead(27 /* 1-2 shift switch connected to pin 9*/) == 0x1) {
    drumPos = 1;
  }
  else if(digitalRead(26 /* 2-3 shift switch connected to pin 11*/) == 0x1) {
    drumPos = 2;
  }
  else if(digitalRead(2 /* 3-4 shift switch connected to pin 13*/) == 0x1) {
    drumPos = 3;
  }
  else if(digitalRead(12 /* 4-5 shift switch connected to pin 15*/) == 0x1) {
    drumPos = 4;
  }
  else if(digitalRead(25 /* Reverse switch connected to pin 7*/ /* 5th gear switch connected to pin 7*/) == 0x1) {
    drumPos = 5;
  }
}

void autoShift(){
  if(engineRpm > 3500 && drumPos < 5){
    upshiftFlag = 1;
  }
  else if(engineRpm < 2000 && drumPos > 2){
    downshiftFlag = 1;
  }
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

// Functions to process data from UART & Bluetooth (FORMAT - "Item:Value")
void processOnboardData(String data) {
 int dataIndex = data.indexOf(':');

 if (dataIndex != -1) {
  String item = data.substring(0, dataIndex);

  if (item == "UP") {
      upshiftFlag = 1;
      downshiftFlag = 0;
    }
    else if (item == "DOWN") {
      downshiftFlag = 1;
      upshiftFlag = 0;
    }
    else if (item == "MODE") {
      mode = data.substring(dataIndex + 1).toInt();
    }
    else if (item == "RPM") {
      engineRpm = data.substring(dataIndex + 1).toInt();
    }
 }
}
