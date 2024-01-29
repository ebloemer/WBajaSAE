#define battery 4
#define fuelSensor  15 // Analog pin connected to the fuel sensor
#define RXD2 16       //for serial output on usb
#define TXD2 17  
#define rpmSensor 18
#define FUEL_READINGS 300
#define RPM_READINGS 100
#define BAT_READINGS 200
#define reverse 23
#define horn 32
#define boardLight 2


int batteryPercent = 0;
int rawBattery = 0;
int batReadings[BAT_READINGS];
int batIndex = 0;
int batTotal = 0;
int batAverage = 0;

int magnets = 3;
unsigned long pulseTime=0;
int engineRPM = 0;
unsigned long diff =0;
float period;
unsigned long enginePrevTime;

int RPMTotal = 0;
int RPMIndex = 0;
int RPMReadings[RPM_READINGS];
int rawRPM = 0;


int rawFuel =0;
int fuel = 0;
int fuelReadings[FUEL_READINGS];
int fuelIndex = 0;
int fuelTotal = 0;
int fuelAverage = 0;


TaskHandle_t fuelReadTask;
TaskHandle_t fuelFilterTask;
TaskHandle_t beeperTask;
TaskHandle_t batteryTask;
TaskHandle_t RPMFilterTask;

void setup() {
  Serial.begin(115200, SERIAL_8N1, RXD2, TXD2);   //setting serial print for usb from rx, tx pins     ensure tx is plugged into rx pin and tx into rx pin
  pinMode(fuelSensor, INPUT);           //selecting fuel sensor from defines

  xTaskCreatePinnedToCore(fuelRead, "fuelRead", 1000, NULL, 1, &fuelReadTask, 1); //task function, name of task, stack size, parameter of task, priority of task, task handle , core selection    //readin fuel value from sensor
  xTaskCreatePinnedToCore(fuelFilter, "fuelFilter", 1000, NULL, 1, &fuelFilterTask, 1);   //taking average fuel reading 
  xTaskCreatePinnedToCore(beeper, "beeper", 1000, NULL, 1, &beeperTask, 1);   //beeper and light
  xTaskCreatePinnedToCore(batRead, "batRead", 1000, NULL, 1, &batteryTask, 0);
  xTaskCreatePinnedToCore(RPMFilter, "RPMFilter", 1000, NULL, 1, &RPMFilterTask, 0);


  pinMode(rpmSensor, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rpmSensor), RPMRead, FALLING);


  pinMode(reverse, INPUT_PULLUP);    //reverse
  pinMode(horn, OUTPUT);          //horn
  pinMode(boardLight, OUTPUT);           //light board
}

void loop() {     // loop runs in core 1, use this or printing to serial
  // Format and display the fuel level
  Serial.print("Fuel:");
  Serial.print(fuel);
  Serial.print(",");

  Serial.print("Raw:");
  Serial.print(rawFuel);
  Serial.print(",");

  Serial.print("RPM:");
  Serial.print(rawRPM);
  Serial.print(",");

  Serial.print("RPM:");
  Serial.print(engineRPM);
  Serial.print(",");


  Serial.print("Battery:");
  Serial.print(batteryPercent);
  Serial.println(",");

  //Serial.print("BatteryRaw :");
  //Serial.print(rawBattery);
  //Serial.println(",");

  delay(100);


}

void fuelRead(void *parameter){
  for(;;){

    rawFuel = analogRead(fuelSensor);     //read analog from fuel sensor
  }
}

void fuelFilter(void *parameter){    
  for(;;){
      fuelTotal -= fuelReadings[fuelIndex];
      fuelReadings[fuelIndex] = rawFuel;
      fuelTotal += rawFuel;
      fuelIndex = (fuelIndex +1)% FUEL_READINGS;
      fuelAverage = fuelTotal / FUEL_READINGS;
      fuel = map(fuelAverage, 0,600,0,100);
    
  }
}

void beeper(void *parameter){
  for(;;){
    if(digitalRead(reverse) == LOW){
      digitalWrite(32, HIGH);
      digitalWrite(2, HIGH);
    } else{
      digitalWrite(32, LOW);
      digitalWrite(2, LOW);
  }
  }
}

void RPMRead(){
    pulseTime = micros();
    diff = pulseTime - enginePrevTime;
    period = 1000000/diff; //us to seconds
    rawRPM = int(period*60/magnets);
    enginePrevTime = pulseTime;
    
}
void RPMFilter(void *parameter){    
  for(;;){
      RPMTotal -= RPMReadings[RPMIndex];
      RPMReadings[RPMIndex] = rawRPM;
      RPMTotal += rawRPM;
      RPMIndex = (RPMIndex +1)% RPM_READINGS;
      engineRPM = RPMTotal / RPM_READINGS;
    
  }
}

void batRead(void *parameter){
  for(;;){
    rawBattery = analogRead(battery);     //read analog from fuel sensor
    batTotal -= batReadings[batIndex];
    batReadings[batIndex] = rawBattery;
    batTotal += rawBattery;
    batIndex = (batIndex +1)% BAT_READINGS;
    batAverage = batTotal / BAT_READINGS;
    batteryPercent = map(batAverage, 2770,3660,0,100);        
  }
}




