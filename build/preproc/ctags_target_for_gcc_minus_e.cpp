# 1 "C:\\Users\\dying\\OneDrive - The University of Western Ontario\\Western Baja\\Github Code\\WBajaSAE\\ecvtCode\\ecvtCode.ino"
// This file contains the code for the ecvtCode project.

# 4 "C:\\Users\\dying\\OneDrive - The University of Western Ontario\\Western Baja\\Github Code\\WBajaSAE\\ecvtCode\\ecvtCode.ino" 2

// Define two instances of HardwareSerial for two UART interfaces
HardwareSerial Onboard(0); // UART0

// Pin Assignments *DO NOT CHANGE* ----------------------------------------------------------------------------

// High Power Outputs





// Low Power Inputs
# 27 "C:\\Users\\dying\\OneDrive - The University of Western Ontario\\Western Baja\\Github Code\\WBajaSAE\\ecvtCode\\ecvtCode.ino"
// Variable declarations ----------------------------------------------------------------

// Battery variables:
int rawBattery;
int batPercent;

// Engine RPM variables:
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
int brakeStatus = 0; // This is (1) when the brake has been pressed for 1/2 second or more

// Throttle variables
int throttlePos = 0;

// Helix variables
int helixPos = 0;

// Onboard communication
String incomingOnboardData = "";

// Global variables
const int debounce = 100;
unsigned long iterationTimer = 0;
const int iterationInterval = 5;


// Function prototypes --------------------------------------------------------------------

void batRead();
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
  Onboard.begin(115200, SERIAL_8N1, 3, 1);

  // Set the pin modes for the motor control pins
  pinMode(32 /* forward + mosfet*/, 0x03);
  pinMode(26 /* forward - mosfet*/, 0x03);
  pinMode(25 /* reverse + mosfet*/, 0x03);
  pinMode(33 /* reverse - mosfet*/, 0x03);

  // Set the pin modes for the sensor inputs
  pinMode(12, 0x01);
  pinMode(36, 0x01);

  // Set the pin modes for the brake input and launch button
  pinMode(27, 0x01);
  pinMode(15 /* can change this one*/, 0x05);

  // Attach an interrupt to the engine sensor pin to detect RPM changes
  attachInterrupt((((12)<40)?(12):-1), RPMRead, 0x02);
}

// Loop function
void loop() {
  readOnboardData(); // Read onboard data

  launch(); // Check launch conditions
  brake(); // Check brake conditions

  // Perform tasks at a specific interval
  if (millis() - iterationTimer >= iterationInterval) {
    batRead();

    exportOnboardData();

    iterationTimer = millis();
  }
}

// Read battery voltage and calculate battery percentage
void batRead() {
  rawBattery = analogRead(36);
  // Calibrate the reading to calculate battery percentage
  batPercent = map(rawBattery, 1830, 2680, 0, 99);
}

// Read RPM from engine sensor
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

// Calculate RPM based on pulse interval
void rpmCalculate() {
  if (magInterval > 0) {
    // Convert difference into frequency (seconds domain)
    magFreq = 60000000 / magInterval;
  }
  // Compensate for pulses per rotation
  rpm = magFreq / magnets;
}

// Handle launch button press
void launch() {
  int buttonState = digitalRead(15 /* can change this one*/);

  if (buttonState != launchPrevState) {
    launchDebounce = millis();
  }

  if (millis() - launchDebounce >= debounce) {
    if (buttonState == 0x0 && launchFlag == 0 && brakeStatus == 1 && rpm < 2000) {
      // Prime for launch if RPM is below 2000, brake is pressed, and button is pressed
      launchInterval = millis();
      launchFlag = 1;
    } else if ((millis() - launchInterval) >= 500 && buttonState == 0x0 && launchActive == 0 && launchFlag == 1) {
      // Activate launch if longer than 1/2 second
      launchActive = 1;
      launchFlag = 0;
    } else if (buttonState == 0x1) {
      // Reset launch state if button is released
      launchActive = 0;
      launchFlag = 0;
    }
  }

  launchPrevState = buttonState;
}

// Handle brake input
void brake() {
  int brakeState = digitalRead(27);

  // Check if brake state has changed
  if (brakeState != brakePrevState) {
    brakeDebounce = millis();
  }

  // Check if debounce time has passed
  if (millis() - brakeDebounce >= 500) {
    // Update brake status based on current state
    if (brakeState == 0x1) {
      brakeStatus = 1; // Brake is pressed
    } else {
      brakeStatus = 0; // Brake is not pressed
    }
  }

  brakePrevState = brakeState;
}

// Read onboard data from UART
void readOnboardData() {
  while (Onboard.available() > 0) {
    char c = Onboard.read();

    if (c == ',') {
      processOnboardData(incomingOnboardData);
      incomingOnboardData = "";
    }
    else if (c != '\n') {
      incomingOnboardData += c;
    }
  }
}

// Process onboard data received from UART
void processOnboardData(String data) {
  int dataIndex = data.indexOf(':');

  if (dataIndex != -1) {
    String item = data.substring(0, dataIndex);

    if (item == "RPM") {
      rpm = data.substring(dataIndex + 1).toInt();
    }
  }
}

// Export onboard data to UART
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
  Onboard.print(",");
}
