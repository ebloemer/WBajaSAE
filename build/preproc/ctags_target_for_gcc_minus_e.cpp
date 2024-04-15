# 1 "C:\\Users\\dying\\OneDrive - The University of Western Ontario\\Western Baja\\Github Code\\WBajaSAE\\ecvtWithPID\\ecvtWithPID.ino"
// This file contains the code for the ecvtCode project.

# 4 "C:\\Users\\dying\\OneDrive - The University of Western Ontario\\Western Baja\\Github Code\\WBajaSAE\\ecvtWithPID\\ecvtWithPID.ino" 2
# 5 "C:\\Users\\dying\\OneDrive - The University of Western Ontario\\Western Baja\\Github Code\\WBajaSAE\\ecvtWithPID\\ecvtWithPID.ino" 2
# 6 "C:\\Users\\dying\\OneDrive - The University of Western Ontario\\Western Baja\\Github Code\\WBajaSAE\\ecvtWithPID\\ecvtWithPID.ino" 2
# 7 "C:\\Users\\dying\\OneDrive - The University of Western Ontario\\Western Baja\\Github Code\\WBajaSAE\\ecvtWithPID\\ecvtWithPID.ino" 2

//error codes




// DEBUG mode



// Serial communication
HardwareSerial Onboard(0); // UART0
BluetoothSerial SerialBT; // Bluetooth debug

// Define an instance of the AS5600 sensor
AS5600 Encoder; // AS5600 sensor

// Pin Assignments *DO NOT CHANGE* ----------------------------------------------------------------------------

// High Power Outputs





// Low Power Inputs




// Communication Pins







// Variable declarations ----------------------------------------------------------------

// Battery variables:
int batLowLimit = 2150;
int batHighLimit = 3180;

int rawBattery;
int batPercent;

const int batSamples = 200;
int batLogger[batSamples];
int batIndex = 0;

// Engine RPM variables:
int commandRpm;
int actualRpm = 0;

int rpmVariance = 50;

// Launch variables
int launchActive = 0; // This is (1) when the CVT needs to be open/system is in launch mode

// Brake variables
unsigned long brakeTimer;
const int brakeDebounce = 100;

int brakePrevState = 0;
int brakeStatus = 0; // This is (1) when the brake has been pressed for 1/2 second or more

// Throttle variables
int throttleMin = 2910;
int throttleMax = 1820;






int rawThrottle = 0;


int throttlePos = 0;

// Helix variables
float rawHelix = 0;
float helixPos = 0;
int helixMax = 47;
int helixMin = 0;
float helixOffset = 166;

int helixMinVariance = 5;
int helixMaxVariance = 5;

int closeSpeed = 150;
int openSpeed = 150;
int returnSpeed = 100;

int limitCheck = 1;

// Motor variables
int motorState = 2; // 0 = Forward, 1 = Reverse, 2 = Stopped, 3 = Past Min, 4 = Past Max

// Define PID parameters
double Kp = 0.1; // Proportional gain
double Ki = 0.1; // Integral gain
double Kd = 0.01; // Derivative gain
double Ti = 100; // Integral time constant
double Td = 0; // Derivative time constant

double previousError = 0;
double integral = 0;
double output = 0; // Output control signal

unsigned long pidPrevTime = 0;
const int pidInterval = 1; // Controls how often the PID loop runs in milliseconds
int pidElapsedTime;

// Serial communication
String incomingOnboardData = "";
String incomingBluetoothData = "";

// Runtime variables
unsigned long updateTimer = 0;
const int updateInterval = 50; // Update interval in milliseconds


// Function prototypes --------------------------------------------------------------------
void openCVT(int revSpeed);
void closeCVT(int fwdSpeed);
void stopCVT();
void getCommandRPM();
void setCommandRPM();
void pastMin(int speed);
void pastMax(int speed);
int checkLimits();
void setCommandHelix(int commandHelix);
void helixRead();
void helixCalibrate(int duration);
void batRead();
void updateBatAverage(int newValue);
void brake();
void readOnboardData();
void processOnboardData(String data);
//void exportOnboardData();
void exportOnboardDiag();
void readBluetoothData();
void processBluetoothData(String data);
void exportBluetoothDiag();

// Functions ---------------------------------------------------------------------------

void setup() {
 // Initialize the Onboard serial communication at a baud rate of 115200
 Onboard.begin(115200, SERIAL_8N1, 3, 1);

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
 pinMode(32 /* forward + mosfet*/, 0x03);
 pinMode(25 /* reverse + mosfet*/, 0x03);

 // Setup LEDC for PWM
 ledcSetup(0, 1000, 8); //forward
 ledcSetup(1, 1000, 8); //reverse


  ledcSetup(2, 1000, 8); //debug
  ledcAttachPin(2, 2);


 // Set the pin modes for the sensor inputs
 pinMode(12, 0x01);
 pinMode(36, 0x01);

 // Set the pin modes for the brake input and launch button
 pinMode(27, 0x01);

 // Ensure the motor is stopped
 stopCVT();
}

// Loop function
void loop() {


  delay(100), // Delay for 100 milliseconds


 // Read serial data
 readOnboardData(); // Read onboard data
 if(SerialBT.connected()) {
  readBluetoothData(); // Read bluetooth data
 }

 if(actualRpm < 1500){ // Correct for low idling/starting RPM
  actualRpm = 1500;
 }

 brake(); // Check brake conditions

 // This begins 'useful' code--------------------------------------------------------------------------

 // Normal operation
  pidElapsedTime = millis() - pidPrevTime;

 if(launchActive == 0 && pidElapsedTime >= pidInterval) {
  setCommandRPM();


   exportOnboardDiag(); // Export onboard data


  if(SerialBT.connected()) {
   exportBluetoothDiag();
  }

  pidPrevTime = millis();
 }

 // Launch operation
 else if (launchActive == 1){
  setCommandHelix(helixMin);


   Onboard.println("Launch Active..,");


  if(SerialBT.connected()) {
   SerialBT.println("Launch Active..,");
  }
 }

 // This ends 'useful' code--------------------------------------------------------------------------

 // Update certain parameters at a fixed interval
 if (millis() - updateTimer >= updateInterval) {
  batRead(); // Read battery voltage





  updateTimer = millis();
 }
}

// Function to open the CVT
void openCVT(int revSpeed) {
 // Check if the motor is already in the reverse state
 if(motorState != 1) {
  stopCVT();
  digitalWrite(25 /* reverse + mosfet*/, 0x1); // Turn on the reverse + mosfet
  ledcAttachPin(33 /* reverse - mosfet*/, 1); // Attach the reverse - mosfet to LEDC channel 1
  ledcWrite(1, revSpeed); // Set the speed of the reverse - mosfet
  motorState = 1;
 }

 // Check if the speed of the reverse - mosfet needs to be updated
 if(ledcRead(1) != revSpeed) {
  ledcWrite(1, revSpeed);
 }


  // Print the opening message to the onboard serial communication
  Onboard.print("Opening..,");
  Onboard.println(revSpeed);


 // Print the opening message to the Bluetooth serial communication if connected
 if(SerialBT.connected()) {
  SerialBT.println("Opening..,");
 }
}

// Function to close the CVT
void closeCVT(int fwdSpeed) {
 // Check if the motor is already in the forward state
 if(motorState != 0) {
  stopCVT();
  digitalWrite(32 /* forward + mosfet*/, 0x1); // Turn on the forward + mosfet
  ledcAttachPin(26 /* forward - mosfet*/, 0); // Attach the forward - mosfet to LEDC channel 0
  ledcWrite(0, fwdSpeed); // Set the speed of the forward - mosfet
  motorState = 0;
 }

 // Check if the speed of the forward - mosfet needs to be updated
 if(ledcRead(0) != fwdSpeed) {
  ledcWrite(0, fwdSpeed);
 }


  // Print the closing message to the onboard serial communication
  Onboard.print("Closing..,");
  Onboard.println(fwdSpeed);


 // Print the closing message to the Bluetooth serial communication if connected
 if(SerialBT.connected()) {
  SerialBT.println("Closing..,");
 }
}

// Function to stop the CVT
void stopCVT() {
 // Read the helix position
 helixRead();

 // Turn off the forward and reverse + mosfets
 digitalWrite(32 /* forward + mosfet*/, 0x0);
 digitalWrite(25 /* reverse + mosfet*/, 0x0);

 // Detach the forward and reverse - mosfets from LEDC channels
 ledcDetachPin(26 /* forward - mosfet*/);
 ledcDetachPin(33 /* reverse - mosfet*/);

 // Set the speed of the LEDC channels to idle
 ledcWrite(0, 0);
 ledcWrite(1, 0);

 // Delay for 1 millisecond to ensure the motor is disconnected
 delay(1);

 // Update the motor state to indicate that it is stopped
 motorState = 2;


  // Print the stopping message to the onboard serial communication
  Onboard.print("Stopping..,");


 // Print the stopping message to the Bluetooth serial communication if connected
 if(SerialBT.connected()) {
  SerialBT.print("Stopping..,");
 }
}

// Function to get the command RPM
void getCommandRPM() {





  throttlePos = map(rawThrottle, 0, 100, 0, 100);


 throttlePos = ((throttlePos)<(0)?(0):((throttlePos)>(100)?(100):(throttlePos)));

 if(throttlePos <= 5){
  throttlePos = 0;
 }

 commandRpm = map(throttlePos, 0, 100, 1500, 3000);
}

// Function to set the command RPM
void setCommandRPM(){

  // Set the command RPM based on the throttle position
  getCommandRPM();

  // Calculate error
  double error = commandRpm - actualRpm;

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


  // Set the debug LED to the output value
    if(ledcRead(2) != (abs(output)/4)){
      ledcWrite(2, (abs(output)/4));
    }



  // Print PID parameters
  Onboard.print("Error: ");
  Onboard.print(error);

  Onboard.print(" Integral: ");
  Onboard.print(integral);

  Onboard.print(" Derivative: ");
  Onboard.print(derivative);
  Onboard.println(", ");


 // Read the helix position
  limitCheck = checkLimits();

  // Execute PID control signal
  if (output > 0 && limitCheck==0 && helixPos > (helixMin+helixMinVariance)) {
    // Open CVT
    openCVT(output); // Adjust PWM duty cycle for motor control
  } else if (output < 0 && limitCheck==0 && helixPos < (helixMax-helixMaxVariance)) {
    // Close CVT
    closeCVT(abs(output)); // Adjust PWM duty cycle for motor control
  } else if (limitCheck == 0){
    // Stop CVT
    stopCVT();


   Onboard.print("Setpoint reached..,");
   Onboard.println(output);


  if(SerialBT.connected()) {
   SerialBT.println("Setpoint reached..,");
  }
  }
}

// Function to handle when the helix position is past the minimum limit
void pastMin(int speed) {
 // Check if the motor state is not already indicating past the minimum limit
 if (motorState != 3) {
  stopCVT();
  digitalWrite(32 /* forward + mosfet*/, 0x1); // Turn on the forward + mosfet
  ledcAttachPin(26 /* forward - mosfet*/, 0); // Attach the forward - mosfet to LEDC channel 0
  ledcWrite(0, speed); // Set the speed of the forward - mosfet
  motorState = 3;
 }

 // Check if the speed of the forward - mosfet needs to be updated
 if (ledcRead(0) != speed) {
  ledcWrite(0, speed);
 }


  // Print the message indicating that the helix is past the minimum limit to the onboard serial communication
  Onboard.println("Helix PAST min..,");


 // Print the message indicating that the helix is past the minimum limit to the Bluetooth serial communication if connected
 if (SerialBT.connected()) {
  SerialBT.println("Helix PAST min..,");
 }
}

// Function to handle when the helix position is past the maximum limit
void pastMax(int speed) {
 // Check if the motor state is not already indicating past the maximum limit
 if (motorState != 4) {
  stopCVT();
  digitalWrite(25 /* reverse + mosfet*/, 0x1); // Turn on the reverse + mosfet
  ledcAttachPin(33 /* reverse - mosfet*/, 1); // Attach the reverse - mosfet to LEDC channel 1
  ledcWrite(1, speed); // Set the speed of the reverse - mosfet
  motorState = 4;
 }

 // Check if the speed of the reverse - mosfet needs to be updated
 if (ledcRead(1) != speed) {
  ledcWrite(1, speed);
 }


  // Print the message indicating that the helix is past the maximum limit to the onboard serial communication
  Onboard.println("Helix PAST max..,");


 // Print the message indicating that the helix is past the maximum limit to the Bluetooth serial communication if connected
 if (SerialBT.connected()) {
  SerialBT.println("Helix PAST max..,");
 }
}

// Function to check if the helix position is within the limits
int checkLimits() {
 helixRead(); // Read the helix position from the sensor

 // Check if the helix position is past the maximum limit
 if (helixPos > (helixMax + helixMaxVariance)) {
  pastMax(returnSpeed); // Handle when the helix position is past the maximum limit
  return 1;
 }
 // Check if the helix position is past the minimum limit
 else if (helixPos < (helixMin - helixMinVariance)) {
  pastMin(returnSpeed); // Handle when the helix position is past the minimum limit
  return 1;
 }
 // The helix position is within the limits
 else {
  return 0; // Return 0 to indicate that the helix position is within the limits
 }
}

// Function to set the command helix position
void setCommandHelix(int commandHelix) {

 limitCheck = checkLimits(); // Check if the helix position is within the limits

 // Check if the helix position is above the setpoint
 if (helixPos > (commandHelix + helixMinVariance) && limitCheck == 0) {
  openCVT(openSpeed); // Open the CVT
 }

 // Check if the helix position is below the setpoint
 else if (helixPos < (commandHelix - helixMaxVariance) && limitCheck == 0) {
  closeCVT(closeSpeed); // Close the CVT
 }

 // Check if the helix position is within the acceptable range
 else if (limitCheck == 0) {
  stopCVT(); // Stop the CVT

  // Print the message indicating that the helix is at the setpoint to the Bluetooth serial communication if connected
  if (SerialBT.connected()) {
   SerialBT.println("Helix at setpoint..,");
  }


   // Print the message indicating that the helix is at the setpoint to the onboard serial communication
   Onboard.println("Helix at setpoint..,");

 }
}

// Function to read the helix position
void helixRead() {
 // Read the helix position from the AS5600 sensor
 rawHelix = map(Encoder.readAngle(),4095,0,0,4095);





}

// Function to read the battery voltage and calculate the battery percentage
void batRead() {

 int rawBattery = analogRead(36);

 // Average the reading
 if(rawBattery > batLowLimit) {
  updateBatAverage(rawBattery);
 }
}

// Function to update the moving average of battery samples
void updateBatAverage(int newValue) {
 batLogger[batIndex] = newValue;
 batIndex = (batIndex + 1) % batSamples;
 int sum = 0;

 // Calculate the sum of all battery samples
 for (int i = 0; i < batSamples; i++) {
  sum += batLogger[i];
 }

 // Calculate the battery percentage based on the average value
 batPercent = map((sum / batSamples), batLowLimit, batHighLimit, 0, 99);

 // Ensure that the battery percentage is within the valid range
 if (batPercent < 0) {
  batPercent = 0;
 } else if (batPercent > 100) {
  batPercent = 100;
 }
}

// Function to handle brake input
void brake() {
 int brakeState = digitalRead(27);

 // Check if brake state has changed
 if (brakeState != brakePrevState) {
  brakeTimer = millis();
 }

 // Check if debounce time has passed
 if (millis() - brakeTimer >= brakeDebounce) {
  // Update brake status based on current state
  if (brakeState == 0x1) {
   brakeStatus = 1; // Brake is pressed
  } else {
   brakeStatus = 0; // Brake is not pressed
  }
 }

 brakePrevState = brakeState;
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

// Functions to process data from UART & Bluetooth (FORMAT - "Item:Value")
void processOnboardData(String data) {
 int dataIndex = data.indexOf(':');

 if (dataIndex != -1) {
  String item = data.substring(0, dataIndex);

  if (item == "RPM") {
   actualRpm = data.substring(dataIndex + 1).toInt();
  } else if (item == "Throttle") {
   rawThrottle = data.substring(dataIndex + 1).toInt();
  } else if (item == "Launch"){
   launchActive = data.substring(dataIndex + 1).toInt();
  } else if (item == "Helix"){
   helixPos = data.substring(dataIndex + 1).toInt();
  } else if (item == "Offset"){
   helixOffset = data.substring(dataIndex + 1).toInt();
  } else if (item == "Min"){
   helixMin = data.substring(dataIndex + 1).toInt();
  } else if (item == "Max"){
   helixMax = data.substring(dataIndex + 1).toInt();
  } else if (item == "Return"){
   returnSpeed = data.substring(dataIndex + 1).toInt();
  } else if (item == "Kp"){
      Kp = data.substring(dataIndex + 1).toFloat();
    } else if (item == "Ki"){
      Ki = data.substring(dataIndex + 1).toFloat();
    } else if (item == "Kd"){
      Kd = data.substring(dataIndex + 1).toFloat();
    }
 }
}

void processBluetoothData(String data) {
 int dataIndex = data.indexOf(':');

 if (dataIndex != -1) {
  String item = data.substring(0, dataIndex);

  if (item == "RPM") {
   actualRpm = data.substring(dataIndex + 1).toInt();
  } else if (item == "Throttle") {
   rawThrottle = data.substring(dataIndex + 1).toInt();
  } else if (item == "Launch"){
   launchActive = data.substring(dataIndex + 1).toInt();
  } else if (item == "Helix"){
   helixPos = data.substring(dataIndex + 1).toInt();
  } else if (item == "Offset"){
   helixOffset = data.substring(dataIndex + 1).toInt();
  } else if (item == "Min"){
   helixMin = data.substring(dataIndex + 1).toInt();
  } else if (item == "Max"){
   helixMax = data.substring(dataIndex + 1).toInt();
  } else if (item == "Return"){
   returnSpeed = data.substring(dataIndex + 1).toInt();
  } else if (item == "Kp"){
      Kp = data.substring(dataIndex + 1).toFloat();
    } else if (item == "Ki"){
      Ki = data.substring(dataIndex + 1).toFloat();
    } else if (item == "Kd"){
      Kd = data.substring(dataIndex + 1).toFloat();
    }

 }
}

// Function to export data to UART
void exportOnboardData() {

 Onboard.print("Battery:");
 Onboard.print(rawBattery);
 Onboard.print(",");

 Onboard.print("Helix:");
 Onboard.print(helixPos);
 Onboard.print(",");

 Onboard.print("Brake:");
 Onboard.print(brakeStatus);
 Onboard.println(",");
}

// Functions to export debugging data to UART & Bluetooth
void exportOnboardDiag() {
 Onboard.print("Fwd:"); // 0-1
 Onboard.print(digitalRead(32 /* forward + mosfet*/));;
 Onboard.print(",");

 Onboard.print("Rev:"); // 0-1
 Onboard.print(digitalRead(25 /* reverse + mosfet*/));
 Onboard.print(",");

  Onboard.print("Bat:");
 Onboard.print(rawBattery);
 Onboard.print(",");

 Onboard.print("RPM:");
 Onboard.print(actualRpm);
 Onboard.print(",");

 Onboard.print("Cmd RPM:");
 Onboard.print(commandRpm);
 Onboard.print(",");

 Onboard.print("Throttle:");
 Onboard.print(rawThrottle);
 Onboard.print(",");

 Onboard.print("Helix:");
 Onboard.print(helixPos);
 Onboard.print(",");

 Onboard.print("Return Spd:");
 Onboard.print(returnSpeed);
 Onboard.print(",");

 Onboard.print("Kp:");
 Onboard.print(Kp);
 Onboard.print(",");

 Onboard.print("Ki:");
 Onboard.print(Ki);
 Onboard.print(",");

 Onboard.print("Kd:");
 Onboard.print(Kd);
 Onboard.println(",");
}

void exportBluetoothDiag(){
 SerialBT.print("Fwd:"); // 0-1
 SerialBT.print(digitalRead(32 /* forward + mosfet*/));
 SerialBT.print(",");

 SerialBT.print("Rev:"); // 0-1
 SerialBT.print(digitalRead(25 /* reverse + mosfet*/));
 SerialBT.print(",");

 SerialBT.print("Bat:");
 SerialBT.print(rawBattery);
 SerialBT.print(",");

 SerialBT.print("RPM:");
 SerialBT.print(actualRpm);
 SerialBT.print(",");

 SerialBT.print("Cmd RPM:");
 SerialBT.print(commandRpm);
 SerialBT.print(",");

 SerialBT.print("Throttle:");
 SerialBT.print(throttlePos);
 SerialBT.print(",");

 SerialBT.print("Helix:");
 SerialBT.print(helixPos);
 SerialBT.print(",");

 SerialBT.print("Return Spd:");
 SerialBT.print(returnSpeed);
 SerialBT.print(",");

 SerialBT.print("Kp:");
 SerialBT.print(Kp);
 SerialBT.print(",");

 SerialBT.print("Ki:");
 SerialBT.print(Ki);
 SerialBT.print(",");

 SerialBT.print("Kd:");
 SerialBT.print(Kd);
 SerialBT.println(",");

}
