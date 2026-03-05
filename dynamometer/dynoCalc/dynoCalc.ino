#include <Arduino.h>
#include "HX711.h"

HX711 scale;

// HX711 Pins
const int DOUT = 16;  // Data pin
const int CLK = 17;   // Clock pin

// Solenoid Pins
const int flowValve = 9;      // Flow solenoid PWM
const int pressureValve = 10;  // Pressure solenoid PWM

// RPM Sensor Pins
const int enginePulseSensor = 11;
const int pumpPulseSensor = 12;

// Solenoid PWM values
int flowValveValue = 0;      // Current flow valve setting (0-255)
int flowValveFrequency = 250; // PWM frequency for flow valve
int pressureValveValue = 0; // Current pressure valve setting (0-255)
int pressureValveFrequency = 250; // PWM frequency for pressure valve

int pwmResolution = 12; // PWM resolution (12 bits for 0-4095 range)

// Tunable parameters
int mode = 0; // 0 = manual, 1 = torque control, 2 = fixed ratio control, 3 = variable ratio control
int manualPressure = 0; // Manual pressure valve setting (0-100%)
int manualFlow = 0; // Manual flow valve setting (0-100%)

int maxPressureValve = 0.98 * (pow(2, pwmResolution) - 1); // Maximum pressure valve setting (98% duty cycle)
int maxFlowValve = 0.98 * (pow(2, pwmResolution) - 1); // Maximum flow valve setting (98% duty cycle)

int primarySprocket = 28; // Teeth on primary sprocket
int secondarySprocket = 40; // Teeth on secondary sprocket

float pressureP = 0.5; // Proportional gain for pressure control
float pressureI = 0.1; // Integral gain for pressure control
float pressureD = 0.05; // Derivative gain for pressure control
float pressureIntegral = 0; // Integral term for pressure control
float pressurePreviousError = 0; // Previous error for pressure control
unsigned long pressurePreviousTime = 0; // Previous time for pressure control PID

float flowP = 0.5;     // Proportional gain for flow control
float flowI = 0.1;     // Integral gain for flow control
float flowD = 0.05;    // Derivative gain for flow control
float flowIntegral = 0; // Integral term for flow control
float flowPreviousError = 0; // Previous error for flow control
unsigned long flowPreviousTime = 0; // Previous time for flow control PID

// Control loop variables
float targetTorque = 50.0; // Target torque in Nm
float targetRpm = 3000.0;   // Target RPM

// Engine RPM variables:
unsigned long enginePulse;
unsigned long prevEnginePulse;
int engineMagnets = 1;

// Pump RPM variables:
const int maxPumpRpm = 3000; // Maximum pump RPM for safety
unsigned long pumpPulse;
unsigned long prevPumpPulse;
int pumpMagnets = 3;

// Final output values:
float engineRpm = 0;
float pumpRpm = 0;
float secondaryRpm = 0;
float torque = 0;
float power = 0;

// Function to calculate RPM from pulse time
void engineMagRead() {
  prevEnginePulse = enginePulse;
  enginePulse = micros();
}

// Function to calculate RPM from magnetic interval
void getEngineRpm() {
  // Convert difference into frequency (seconds domain)
  if (micros() - enginePulse > 1000000){ // If more than 1 second has passed, assume engine is stopped
    engineRpm = 0;
  } else if(enginePulse > prevEnginePulse){
    double magFreq = 60000000 / (enginePulse - prevEnginePulse);
    engineRpm = magFreq / engineMagnets;
  }
}

// Function to calculate RPM from pulse time
void pumpMagRead() {
  prevPumpPulse = pumpPulse;
  pumpPulse = micros();
}

// Function to calculate RPM from magnetic interval
void getPumpRpm() {
  // Convert difference into frequency (seconds domain)
  if (micros() - pumpPulse > 1000000){ // If more than 1 second has passed, assume pump is stopped
    pumpRpm = 0;
  } else if(pumpPulse > prevPumpPulse){
    double magFreq = 60000000 / (pumpPulse - prevPumpPulse);
    pumpRpm = magFreq / pumpMagnets;
  }
}

void getTorque(){
  if (scale.is_ready()) {
      torque = scale.get_units(10);
    }
}

void valveControl() {
  // Set flow valve (0-255 PWM)
  if (mode == 0) { // Manual mode
    flowValveValue = map(manualFlow, 0, 100, 0, maxFlowValve);
    pressureValveValue = map(manualPressure, 0, 100, 0, maxPressureValve);
  } else if (mode == 1) { // Torque control mode
    flowValveValue = 0; // Start with flow valve closed
    
    getTorque(); // Update torque reading
    float error = targetTorque - torque;

    PIDControl(error, pressureIntegral, pressurePreviousError, pressureP, pressureI, pressureD, pressureValveValue, maxPressureValve, pressurePreviousTime);
    //ledcwrite(pressureValveValue); // For debugging: visualize pressure valve setting with LED brightness
  
  } else if (mode >= 2) { // RPM control mode
    pressureValveValue = 0; // Start with pressure valve closed
    
    getPumpRpm(); // Update pump RPM reading
    int targetPumpRpm = (primarySprocket / secondarySprocket) * targetRpm; // Calculate target pump RPM based on sprocket ratio
    targetPumpRpm = constrain(targetPumpRpm, 0, maxPumpRpm); // Ensure target pump RPM does not exceed target engine RPM
    float error = targetPumpRpm - pumpRpm;

    PIDControl(error, flowIntegral, flowPreviousError, flowP, flowI, flowD, flowValveValue, maxFlowValve, flowPreviousTime);
    //ledcwrite(flowValveValue); // For debugging: visualize flow valve setting with LED brightness
  }

  // Write PWM values to valves
  ledcWrite(flowValve, flowValveValue);
  ledcWrite(pressureValve, pressureValveValue);
}

void PIDControl(float error, float &integral, float &previousError, float P, float I, float D, int &valveValue, int maxValve, unsigned long &previousTime) {
  unsigned long now = micros();
  float dt = (now - previousTime) / 1000000.0;  // seconds
  if (dt <= 0) dt = 0.000001;                   // safety
  previousTime = now;

  float proportional = P * error;

  integral += error * I * dt;
  integral = constrain(integral, 0, maxValve);   // anti-windup

  float derivative = D * ((error - previousError) / dt);

  float output = proportional + integral + derivative;

  valveValue = constrain(output, 0, maxValve);

  previousError = error;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Initialize HX711
  scale.begin(DOUT, CLK);
  scale.set_scale(2280.f);  // Set the scale factor (calibration value)
  scale.tare();             // Zero the scale
  
  // Initialize H-Bridge pins
  pinMode(flowValve, OUTPUT);
  pinMode(pressureValve, OUTPUT);
  pinMode(enginePulseSensor, INPUT);
  pinMode(pumpPulseSensor, INPUT);

  // Set PWM frequency and resolution
  ledcAttach(flowValve, flowValveFrequency, pwmResolution);
  ledcAttach(pressureValve, pressureValveFrequency, pwmResolution);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(enginePulseSensor), engineMagRead, RISING);
  attachInterrupt(digitalPinToInterrupt(pumpPulseSensor), pumpMagRead, RISING);
  
  
  Serial.println("Dynamometer initialized");
}

void loop() {
  getEngineRpm();

  
  // Control solenoid valves (0-255 PWM)
  valveControl();
}