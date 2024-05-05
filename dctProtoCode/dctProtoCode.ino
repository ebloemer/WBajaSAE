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

int currentGear = 0;
int drumPos = 0;

// PID control parameters
const int Kp = 1.0; // Proportional gain
const int Ki = 0.0; // Integral gain
const int Kd = 0.0; // Derivative gain

double Ti = 0.0; // Integral time constant
double Td = 0.0; // Derivative time constant

float motorCommand = 0.0; // Motor command (output of PID controller)

double pidElapsedTime = 0.0; // Elapsed time since last PID update
double integral = 0.0; // Integral term for PID control
double previousError = 0.0; // Previous error for derivative term
int output = 0.0; // PID output


// Define gear states
enum Gear {
  REV_GEAR,
  NEUTRAL,
  FIRST_GEAR,
  SECOND_GEAR,
  THIRD_GEAR,
  FOURTH_GEAR,
  FIFTH_GEAR,
};

Gear currentGear = NEUTRAL; // Starting gear

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

  // Initialize serial communication
  Serial.begin(9600);
}

void loop() {
  // Your code to control gear shifting goes here

  // Shift up:
  if (upRequest == 1) {
    shiftUp();
    upRequest = 0;
  }
  // Shift down:
  else if (downRequest == 1) {
    shiftDown();
    downRequest = 0;
  }
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
      // Handle unexpected gear
      break;
  }
  currentGear = gear;
}

int drumShift(int shiftPos) {
  
  while(drumPos != shiftPos) {
    if(drumPos < shiftPos) {
      motorCommand = 3000;
    }
    else if(drumPos > shiftPos) {
      motorCommand = 1000;
    }
    getDrumPos();
  }
}

void shiftController(int commandPos){

  // Get current shifter position
  int actualPos = analogRead(shiftPot);

  // Calculate error
  double error = commandPos - actualPos;

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
  if(digitalRead(rev_Switch) == HIGH) {
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

  else if (drumPos < 1) {
    drumShift(drumPos + 1);
    setClutch(drumPos);
  }
}

void shiftDown() {
  getDrumPos();
  if (drumPos > 1) {
    drumShift(drumPos - 1);
    setClutch(drumPos);
  }
}
