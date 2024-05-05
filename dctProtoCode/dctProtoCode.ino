// Define solenoid pin connections
const int linSolenoid = 2; // Solenoid 1 connected to pin 2
const int shiftSolenoid = 3; // Solenoid 2 connected to pin 3

const int shiftMotorFwd = 4; // Shift motor connected to pin 4
const int shiftMotorRev = 5; // Shift motor connected to pin 5

const int shiftPot = 6; // Gear position sensor connected to pin 6

const int rev_Switch = 25; // Reverse switch connected to pin 7
const int neutral_Switch = 17; // Neutral switch connected to pin 8
const int one_two_Switch = 27; // 1-2 shift switch connected to pin 9
const int two_neutral_Switch = 19; // 2-N shift switch connected to pin 10
const int two_three_Switch = 26; // 2-3 shift switch connected to pin 11
const int three_neutral_Switch = 22; // 3-N shift switch connected to pin 12
const int three_four_Switch = 2; // 3-4 shift switch connected to pin 13
const int four_neutral_Switch = 24; // 4-N shift switch connected to pin 14
const int four_five_Switch = 12; // 4-5 shift switch connected to pin 15
const int five_neutral_Switch = 13; // 5-N shift switch connected to pin 16
const int five_Switch = rev_Switch; // 5th gear switch connected to pin 7

int upRequest = 0; // Variable to store upshift request
int downRequest = 0; // Variable to store downshift request

int drumPos = 0;
int reverseFlag = 0;

int engineRpm = 3600;

int motorDown = 1000;
int motorMid = 2000;
int motorUp = 3000;

// PID control parameters
const float Kp = 1.0; // Proportional gain
const float Ki = 0.0; // Integral gain
const float Kd = 0.0; // Derivative gain

const float Ti = 0.001; // Integral time constant
const float Td = 0.001; // Derivative time constant

float motorCommand = 0.0; // Motor command (output of PID controller)

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

  // Initialize transmission to neutral
  setClutch(0);

  // Intitalize drum position
  drumShift(0);

  // Initialize serial communication
  Serial.begin(9600);
}

void loop() {
  // Your code to control gear shifting goes here
  if(upRequest == 1 && downRequest == 1) {
    upRequest = 0;
    downRequest = 0;
    Serial.println("ERROR: Both up and down shift requests received");
  }

  // Shift up:
  if (upRequest == 1 && engineRpm > 2000) {
    shiftUp();
    upRequest = 0;
  }
  // Shift down:
  else if (downRequest == 1 && engineRpm < 2800) {
    shiftDown();
    downRequest = 0;
  }

  shiftController(motorMid);
}

void setClutch(int gear) {
  switch (gear) {
    case -1:
      digitalWrite(linSolenoid, LOW);
      digitalWrite(shiftSolenoid, LOW);
      break;
    case 0:
      digitalWrite(linSolenoid, HIGH);
      digitalWrite(shiftSolenoid, LOW);
      break;
    case 1:
      digitalWrite(linSolenoid, LOW);
      digitalWrite(shiftSolenoid, HIGH);
      break;
    case 2:
      digitalWrite(linSolenoid, LOW);
      digitalWrite(shiftSolenoid, LOW);
      break;
    case 3:
      digitalWrite(linSolenoid, LOW);
      digitalWrite(shiftSolenoid, HIGH);
      break;
    case 4:
      digitalWrite(linSolenoid, LOW);
      digitalWrite(shiftSolenoid, LOW);
      break;
    case 5:
      digitalWrite(linSolenoid, LOW);
      digitalWrite(shiftSolenoid, HIGH);
      break;
    default:
      Serial.println("ERROR: Invalid clutch setpoint");
      break;
  }
}

void drumShift(int desiredDrum) {

  // Get current motor position
  int motorPos;

  int downFlag = 0;
  int upFlag = 0;

  // Calculate motor command based on desired shift position
  while(desiredDrum > drumPos) {
    motorPos = analogRead(shiftPot);

    if(motorPos < motorUp && downFlag == 0) {
      shiftController(motorUp);
    }

    else if(motorPos > motorMid){
      downFlag = 1;
      shiftController(motorMid);
    }

    else if(motorPos < motorMid && downFlag == 1){
      downFlag = 0;
    }
    
    getDrumPos();
  }

  while(desiredDrum < drumPos) {
    motorPos = analogRead(shiftPot);

    if(motorPos > motorDown && upFlag == 0) {
      shiftController(motorDown);
    }

    else if(motorPos < motorMid){
      upFlag = 1;
      shiftController(motorMid);
    }

    else if(motorPos > motorMid && upFlag == 1){
      upFlag = 0;
    }

    getDrumPos();
  }

  shiftController(motorMid);
}

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

void getDrumPos(){
  if(digitalRead(rev_Switch == HIGH) && digitalRead(neutral_Switch == HIGH) && digitalRead(one_two_Switch == HIGH) && digitalRead(two_three_Switch == HIGH) && digitalRead(three_four_Switch == HIGH) && digitalRead(four_five_Switch == HIGH) && digitalRead(five_Switch == HIGH) || digitalRead(rev_Switch == LOW) && digitalRead(neutral_Switch == LOW) && digitalRead(one_two_Switch == LOW) && digitalRead(two_three_Switch == LOW) && digitalRead(three_four_Switch == LOW) && digitalRead(four_five_Switch == LOW) && digitalRead(five_Switch == LOW)){
    drumPos = -2;
    Serial.println("ERROR: Invalid drum position");
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

void shiftUp() {
  getDrumPos();
  if (drumPos < 5 && drumPos > 0) {
    setClutch(drumPos + 1);
    drumShift(drumPos + 1);
  }

  else if (drumPos == 0 || drumPos == -1) {
    drumShift(drumPos + 1);
    setClutch(drumPos);
    reverseFlag = 0;
  }
}

void shiftDown() {
  getDrumPos();
  if (drumPos > 0) {
    drumShift(drumPos - 1);
    setClutch(drumPos);
  }
  else if (drumPos == 0 && engineRpm < 1600) {
    drumShift(drumPos - 1);
    setClutch(drumPos);
    reverseFlag = 1;
  }
}
