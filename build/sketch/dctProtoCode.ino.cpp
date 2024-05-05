#include <Arduino.h>
#line 1 "C:\\Users\\dying\\OneDrive - The University of Western Ontario\\Western Baja\\Github Code\\WBajaSAE\\dctProtoCode\\dctProtoCode.ino"
// Includes
#include <HardwareSerial.h>

// Define serial communication
HardwareSerial Onboard(0); // UART0
String incomingOnboardData = "";

// OUTPUT PIN ASSIGNMENTS
#define linSolenoid 2 // Solenoid 1 connected to pin 2
#define shiftSolenoid 3 // Solenoid 2 connected to pin 3
#define shiftMotorFwd 4 // Shift motor connected to pin 4
#define shiftMotorRev 5 // Shift motor connected to pin 5

// ANALOG INPUT PIN ASSIGNMENTS
#define shiftPot 6 // Gear position sensor connected to pin 6

// DIGITAL INPUT PIN ASSIGNMENTS
#define rev_Switch 25 // Reverse switch connected to pin 7
#define neutral_Switch 17 // Neutral switch connected to pin 8
#define one_two_Switch 27 // 1-2 shift switch connected to pin 9
#define two_neutral_Switch 19 // 2-N shift switch connected to pin 10
#define two_three_Switch 26 // 2-3 shift switch connected to pin 11
#define three_neutral_Switch 22 // 3-N shift switch connected to pin 12
#define three_four_Switch 2 // 3-4 shift switch connected to pin 13
#define four_neutral_Switch 24 // 4-N shift switch connected to pin 14
#define four_five_Switch 12 // 4-5 shift switch connected to pin 15
#define five_neutral_Switch 13 // 5-N shift switch connected to pin 16
#define five_Switch rev_Switch // 5th gear switch connected to pin 7

// Physical position setpoints
int motorDown = 1000;
int motorMid = 2000;
int motorUp = 3000;

// Physical state variables
int engineRpm = 3600; // Engine RPM
int drumPos = 0;

int reverseFlag = 0;

int inputGear = 0;  // -1 = Reverse, 0 = Neutral, 1 = 1st gear, 2 = 2nd gear, 3 = 3rd gear, 4 = 4th gear, 5 = 5th gear

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

#line 78 "C:\\Users\\dying\\OneDrive - The University of Western Ontario\\Western Baja\\Github Code\\WBajaSAE\\dctProtoCode\\dctProtoCode.ino"
void setup();
#line 107 "C:\\Users\\dying\\OneDrive - The University of Western Ontario\\Western Baja\\Github Code\\WBajaSAE\\dctProtoCode\\dctProtoCode.ino"
void loop();
#line 127 "C:\\Users\\dying\\OneDrive - The University of Western Ontario\\Western Baja\\Github Code\\WBajaSAE\\dctProtoCode\\dctProtoCode.ino"
void setGear(int gear);
#line 78 "C:\\Users\\dying\\OneDrive - The University of Western Ontario\\Western Baja\\Github Code\\WBajaSAE\\dctProtoCode\\dctProtoCode.ino"
void setup() {
  // Set solenoid pins as outputs
  pinMode(linSolenoid, OUTPUT);
  pinMode(shiftSolenoid, OUTPUT);
  pinMode(shiftMotorFwd, OUTPUT);
  pinMode(shiftMotorRev, OUTPUT);

  // Set gear position sensor pin as input
  pinMode(shiftPot, INPUT);

  // Set switch pins as inputs
  pinMode(rev_Switch, INPUT);
  pinMode(neutral_Switch, INPUT);
  pinMode(one_two_Switch, INPUT);
  pinMode(two_neutral_Switch, INPUT);
  pinMode(two_three_Switch, INPUT);
  pinMode(three_neutral_Switch, INPUT);
  pinMode(three_four_Switch, INPUT);
  pinMode(four_neutral_Switch, INPUT);
  pinMode(four_five_Switch, INPUT);
  pinMode(five_neutral_Switch, INPUT);

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

  if(gear <  drumPos && drumPos > 0) {
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
      if(digitalRead(rev_Switch) == HIGH && engineRpm < 1600) {
        digitalWrite(linSolenoid, LOW);
        digitalWrite(shiftSolenoid, LOW);
      }
      else if (digitalRead(rev_Switch) == HIGH && engineRpm > 1600){
        Onboard.println("ERROR: Current RPM does not match clutch setpoint (Reverse)");
      } else {
        Onboard.println("ERROR: Drum position does not match clutch setpoint (Reverse)");
      }
      break;
    case 0:
      if(digitalRead(neutral_Switch) == HIGH) {
        digitalWrite(linSolenoid, HIGH);
        digitalWrite(shiftSolenoid, LOW);
      }
      else {
        Onboard.println("ERROR: Drum position does not match clutch setpoint (Neutral)");
      }
      break;
    case 1:
      if(digitalRead(one_two_Switch) == HIGH) {
        digitalWrite(linSolenoid, LOW);
        digitalWrite(shiftSolenoid, HIGH);
      }
      else {
        Onboard.println("ERROR: Drum position does not match clutch setpoint (1st gear)");
      }
      break;
    case 2:
      if(digitalRead(one_two_Switch) == HIGH || digitalRead(two_three_Switch) == HIGH) {
        digitalWrite(linSolenoid, LOW);
        digitalWrite(shiftSolenoid, LOW);
      }
      else {
        Onboard.println("ERROR: Drum position does not match clutch setpoint (2nd gear)");
      }
      break;
    case 3:
      if(digitalRead(two_three_Switch) == HIGH || digitalRead(three_four_Switch) == HIGH) {
        digitalWrite(linSolenoid, LOW);
        digitalWrite(shiftSolenoid, HIGH);
      }
      else {
        Onboard.println("ERROR: Drum position does not match clutch setpoint (3rd gear)");
      }
      break;
    case 4:
      if(digitalRead(three_four_Switch) || digitalRead(four_five_Switch) == HIGH) {
        digitalWrite(linSolenoid, LOW);
        digitalWrite(shiftSolenoid, LOW);
      }
      else {
        Onboard.println("ERROR: Drum position does not match clutch setpoint (4th gear)");
      }
      break;
    case 5:
      if(digitalRead(four_five_Switch) || digitalRead(five_Switch) == HIGH) {
        digitalWrite(linSolenoid, LOW);
        digitalWrite(shiftSolenoid, HIGH);
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
  int motorPos = analogRead(shiftPot);
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
  int motorPos = analogRead(shiftPot);

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
  output = constrain(output, -255, 255);

  // Update motor speed based on PID output
  if (output > 0) {
    analogWrite(shiftMotorFwd, output);
    digitalWrite(shiftMotorRev, LOW);
  } else if (output < 0) {
    digitalWrite(shiftMotorFwd, LOW);
    analogWrite(shiftMotorRev, output);
  } else {
    digitalWrite(shiftMotorFwd, LOW);
    digitalWrite(shiftMotorRev, LOW);
  }
}

// Get drum position based on switch inputs
void getDrumPos(){
  if(digitalRead(rev_Switch == HIGH) && digitalRead(neutral_Switch == HIGH) && digitalRead(one_two_Switch == HIGH) && digitalRead(two_three_Switch == HIGH) && digitalRead(three_four_Switch == HIGH) && digitalRead(four_five_Switch == HIGH) && digitalRead(five_Switch == HIGH) || digitalRead(rev_Switch == LOW) && digitalRead(neutral_Switch == LOW) && digitalRead(one_two_Switch == LOW) && digitalRead(two_three_Switch == LOW) && digitalRead(three_four_Switch == LOW) && digitalRead(four_five_Switch == LOW) && digitalRead(five_Switch == LOW)){
    drumPos = -2;
    Onboard.println("ERROR: Invalid drum position");
  }
  else if(digitalRead(rev_Switch) == HIGH && reverseFlag == 1) {
    drumPos = -1;
  }
  else if(digitalRead(neutral_Switch) == HIGH) {
    drumPos = 0;
  }
  else if(digitalRead(one_two_Switch) == HIGH) {
    drumPos = 1;
  }
  else if(digitalRead(two_three_Switch) == HIGH) {
    drumPos = 2;
  }
  else if(digitalRead(three_four_Switch) == HIGH) {
    drumPos = 3;
  }
  else if(digitalRead(four_five_Switch) == HIGH) {
    drumPos = 4;
  }
  else if(digitalRead(five_Switch) == HIGH) {
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
