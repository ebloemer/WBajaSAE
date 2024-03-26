# 1 "C:\\Users\\dying\\OneDrive - The University of Western Ontario\\Western Baja\\Github Code\\WBajaSAE\\workingCompCode\\workingCompCode.ino"

/*

    Name:       NewBoardCode.ino

    Created:	2/11/2024 12:08:47 AM

    Author:     WesternBajaSAE

*/
# 8 "C:\\Users\\dying\\OneDrive - The University of Western Ontario\\Western Baja\\Github Code\\WBajaSAE\\workingCompCode\\workingCompCode.ino"
// Pin Assignments *DO NOT CHANGE* ----------------------------------------------------------------------------

//High Power Outputs
# 20 "C:\\Users\\dying\\OneDrive - The University of Western Ontario\\Western Baja\\Github Code\\WBajaSAE\\workingCompCode\\workingCompCode.ino"
//Low Power Inputs
# 46 "C:\\Users\\dying\\OneDrive - The University of Western Ontario\\Western Baja\\Github Code\\WBajaSAE\\workingCompCode\\workingCompCode.ino"
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

int panicFlag;
int muteFlag;

unsigned long panicInterval;
unsigned long muteInterval;

unsigned long muteDebounce;
unsigned long panicDebounce;
const int debounce = 20;

int panicPrevState;
int mutePrevState;

//Program running stuff
unsigned long iterationTimer = 0;
const int iterationInterval = 16;
int panicCount = 0;
int lapResetCount = 0;
const int numSamples = 200; // Number of samples to consider for moving average
int pwmSamples[numSamples]; // Array to store PWM samples
int sampleIndex = 0; // Index to keep track of the current sample


// Functions ---------------------------------------------------------------------------

void reverseCheck() {
 if (digitalRead(19) == 0x0) {
  digitalWrite(32, 0x0);
  }
  else {
    digitalWrite(32, 0x1);
  }
}

void batRead(){

  rawBattery = analogRead(36);

  // calibrate the reading
  batPercent = map(rawBattery,2150,3150,0,99);

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

void fuelRead(){

  if(digitalRead(22) == 0x1 && fuelFlag == 0){
    fuelStart = micros();
    fuelFlag =1;
  }
  else if(digitalRead(22) == 0x0 && fuelFlag == 1){
    fuelEnd = micros();
    fuelFlag =0;
  }

  fuelTime = fuelEnd - fuelStart;

  rawFuel = map(fuelTime, fuelLowLimit, fuelHighLimit, 0, 100);
  if(rawFuel>0){
    updateMovingAverage(rawFuel);
  }
}

void shockRead(){
  leftFront = analogRead(2);
  rightFront = analogRead(15);
  leftRear = analogRead(34);
  rightRear = analogRead(39);

  leftFront = map(leftFront, 900, 4095, 0, 100);
  rightFront = map(rightFront, 900, 4095, 0, 100);
  leftRear = map(leftRear, 0, 4095, 0, 100);
  rightRear = map(rightRear, 0, 4095, 0, 100);
}

void mute (){ //interrupt function to be held to turn off mute on phone
  int buttonState = digitalRead(5);

  if(buttonState != mutePrevState){
    muteDebounce = millis();
  }

  if(millis()-muteDebounce >= debounce){

    if(buttonState == 0 && muteFlag == 0){
      muteFlag = 1;
      muteInterval = millis();
    }

    if((millis() - muteInterval) <= 1000 && buttonState == 0x1 && muteFlag == 1){ //activate panic if longer than 2 seconds

      if(muteStatus == 0){
        muteStatus = 1;
      }
      else{
        muteStatus = 0;
      }

      muteFlag = 0;
    }
    else if((millis() - muteInterval) > 1000 && buttonState == 0x1 && muteFlag == 1){
      muteFlag = 0;
    }


  }

  mutePrevState = buttonState;
}

void randomButtonChange (){ //interrupt function for unused button

}

void lapTimeReset (){ //interrupt function to reset the lap timer on phone

}

void panic (){ //interrupt function to press to start panic stuff on phone and send messages
  int buttonState = digitalRead(18);

  if(buttonState != panicPrevState){
    panicDebounce = millis();
  }

  if(millis()-panicDebounce >= debounce){

    if(buttonState == 0x0 && panicFlag == 0 && panicActive == 0){
      panicFlag = 1;
      panicInterval = millis();
    }

    else if((millis() - panicInterval) >= 2000 && buttonState == 0x0 && panicFlag == 1 && panicActive == 0){ //activate panic if longer than 2 seconds
      if(panicStatus == 0){
        panicStatus = 1;
      }
      else{
        panicStatus = 0;
      }

      panicActive = 1;
    }

    else if((millis() - panicInterval) <= 1000 && buttonState == 0x1 && panicFlag == 1){ //activate lap reset if less than 1 second
      if(lapReset == 0){
        lapReset = 1;
      }
      else{
        lapReset = 0;
      }
      panicFlag = 0;
    }

    else if(buttonState == 0x1 && panicFlag == 1){
      panicFlag = 0;
      panicActive = 0;
    }
  }

  panicPrevState = buttonState;

}


// The setup() function runs once each time the micro-controller starts
void setup()
{
 Serial.begin(115200, SERIAL_8N1, 16, 17);

  for (int i = 0; i < numSamples; i++) {
    pwmSamples[i] = 0;
  }

 // Initialize regulator pins *DO NOT CHANGE*
  pinMode(32, 0x03);
  pinMode(33, 0x03);
  pinMode(25, 0x03);
  pinMode(26, 0x03);
  pinMode(13, 0x03);
  pinMode(27, 0x03);
  pinMode(14, 0x03);

 // Initialize sensor pins 
 pinMode(2, 0x01);
 pinMode(15, 0x01);
 pinMode(34, 0x01);
 pinMode(39, 0x01);

 pinMode(23, 0x01);
 pinMode(35, 0x01);
 pinMode(22, 0x01);
 pinMode(36, 0x01);
 pinMode(19, 0x05);

 pinMode(21, 0x05);
 pinMode(18, 0x05);
 pinMode(5, 0x05);
 pinMode(4, 0x05);

 // Control initial power output
 digitalWrite(32, 0x1);
 digitalWrite(33, 0x0);
 digitalWrite(25, 0x0);
 digitalWrite(26, 0x1);
 digitalWrite(13, 0x0);
 digitalWrite(27, 0x0);
 digitalWrite(14, 0x1);

  attachInterrupt((((23)<40)?(23):-1), RPMRead, 0x02);
  attachInterrupt((((22)<40)?(22):-1), fuelRead, 0x03); //reverse program
}

// Add the main program code into the continuous loop() function
void loop(){

  panic();
  mute();

  // Check if the counter is a multiple of 100
  if (millis() - iterationTimer >= iterationInterval) {

    //run non-time critical functions
    batRead();
    shockRead();

    //print to serial

    Serial.print("RPM:"); //0-3800
    Serial.print(rpm);
    Serial.print(",");

    Serial.print("Speed:"); //0-40
    Serial.print(speed);
    Serial.print(",");

    Serial.print("Battery:"); //0-100
    Serial.print(batPercent);
    Serial.print(",");

    Serial.print("Fuel:"); //0-100
    Serial.print(fuel);
    Serial.print(",");

    Serial.print("LF:"); //0-100
    Serial.print(leftFront);
    Serial.print(",");

    Serial.print("RF:"); //0-100
    Serial.print(rightFront);
    Serial.print(",");

    Serial.print("LB:"); //0-100
    Serial.print(leftRear);
    Serial.print(",");

    Serial.print("RB:"); //0-100
    Serial.print(rightRear);
    Serial.print(",");

    Serial.print("panic:"); //0-1
    Serial.print(panicStatus);
    Serial.print(",");

    Serial.print("mute:"); //0-1
    Serial.print(muteStatus);
    Serial.print(",");

    Serial.print("lapTimer:"); //0-1
    Serial.print(lapReset);
    Serial.println(",");

    iterationTimer = millis();
  }
}
