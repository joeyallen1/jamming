#include <Wire.h>
#include "FDC1004.h"
#include "Adafruit_MPRLS.h"

unsigned long pressureTime;
unsigned long capacitanceTime;

float pressureInit;
float capC0Init = -2.0;
float capC1Init = -2.0;
float capC2Init = -2.0;

float capC0UJ = -2.0;
float capC1UJ = -2.0;
float capC2UJ = -2.0;

int valveF = 3;
int valveJ  = 9;

int motorPosPress = 10;
int motorNegPress = 11;

FDC1004 fdc;


////////////Fixed measurement parameters, do not change/////////////////////
const float capMax = 14.9;
const float capdacConversion = 3.125;


const uint8_t measSingl = 0; // 0 is single read, 1 is continuous read


const uint8_t measA = 0;
const uint8_t measB = 1;
const uint8_t measC = 2;

const uint8_t pin1 = (uint8_t)FDC1004_Chan0;
const uint8_t pin2 = (uint8_t)FDC1004_Chan1;
const uint8_t pin3 = (uint8_t)FDC1004_Chan2;

uint8_t capdacA;
uint8_t capdacB;
uint8_t capdacC;


///////// Pressure sensor changes /////////
// You dont *need* a reset and EOC pin for most uses, so we set to -1 and don't connect
#define RESET_PIN  -1  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN    -1  // set to any GPIO pin to read end-of-conversion by pin
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);

///////////Parameters you can change//////////////////////////////////////////
int readRate = 10; //does not currently work above 100 Hz

void setup() {
  Serial.begin(115200);
  delay(1);
  Wire.begin();
  delay(1);

  // set up all pins as outputs
  pinMode(valveF,OUTPUT);
  pinMode(valveJ,OUTPUT);
  pinMode(motorPosPress,OUTPUT);
  pinMode(motorNegPress,OUTPUT);
  delay(1);

  sensorSetup();

  delay(1000);

  // read initial unjammed capacitance
  while ((capC1UJ < 1.0) || (capC2UJ < 1.0) || (capC0UJ < 1.0)){
  Serial.println("Initializing capacitance on all channels");
  capC0UJ = absoluteCapacitance(measA, pin1, pin1, measSingl, &capdacA);
  capC1UJ = absoluteCapacitance(measB, pin2, pin2, measSingl, &capdacB);
  capC2UJ = absoluteCapacitance(measC, pin3, pin3, measSingl, &capdacC);
  }

  Serial.println("Unjammed capacitances (C0, C1, C2)");
  Serial.println(capC0UJ, 4);
  Serial.println(capC1UJ, 4);
  Serial.println(capC2UJ, 4);


  //grasp object
  digitalWrite(motorPosPress,HIGH);
  delay(3000);
  digitalWrite(motorPosPress,LOW);

  // calibrate jammed capacitance
  digitalWrite(valveJ, HIGH);
  digitalWrite(motorNegPress, HIGH);

  delay(5000);

  // read initial pressure
  pressureTime = millis();
  pressureInit = mpr.readPressure();
  Serial.print("Pressure, ");
  Serial.print(pressureTime);
  Serial.print(", ");
  Serial.println(pressureInit);

  // read initial jammed capacitance
  while ((capC1Init < 1.0) || (capC2Init < 1.0) || (capC0Init < 1.0)){
  Serial.println("Finding jammed state for all capacitors");
  capC0Init = absoluteCapacitance(measA, pin1, pin1, measSingl, &capdacA);
  capC1Init = absoluteCapacitance(measB, pin2, pin2, measSingl, &capdacB);
  capC2Init = absoluteCapacitance(measC, pin3, pin3, measSingl, &capdacC);
  }

  Serial.println(capC0Init, 4);
  Serial.println(capC1Init, 4);
  Serial.println(capC2Init, 4);

  Serial.println("Jammed state has been calibrated.");
}


void loop() {

  digitalWrite(valveJ, LOW);
  digitalWrite(valveF, LOW);
  digitalWrite(motorPosPress, LOW);
  digitalWrite(motorNegPress, LOW);

  int countdown = 50;

  while (countdown){
    logger();
    countdown -= 1;
  }

  deactivateActuator(2000, valveJ);
  deactivateActuator(2000, valveF);



  //uncomment relevant case to run. Don't uncomment more than one!


  //continuously on. //motors are always running. Do not include valves in flow
  //continuouslyOn();


  //On just once. Sequentially activates valve/motor for 1 s, then turns off. The on time needs to be tuned based on data
  //periodic(1000, 2^31);


  //On periodically. Sequentially activates valve/motor for 1 s, then off for 10 mins. These times need to be tuned based on data
  //periodic(1000, 1000*10*60);


  //Threshold activation. Activates only when the capacitance on one finger drops below the initial value to the threhold fraction
  //threshold(0.9);
}


void continuouslyOn(){
  //motors are always on, so valves are not included in the setup. Need to be completely physically bypassed in jamming actuator

  while (1){
    logger();
  }
}


void periodic(unsigned long onTime, unsigned long offTime){

  unsigned long startTime = millis();

  //turn on fingers, then jamming actuators
  activateActuator(onTime, valveF);
  activateActuator(onTime, valveJ);
  while ((millis() - startTime) < (onTime + offTime)){
    logger();
  }
}


void threshold(float thresholdCap){
  unsigned long onTime = 100;
  unsigned long waitTime = 5; //wait for 5 ms before switching to the next set
  //not sure when to re-jam vs. reactivate fingers. Could do a rejam check, followed by reactivate fingers if unsuccessful?


  float cap0 = absoluteCapacitance(measA, pin1, pin1, measSingl, &capdacA);
  float cap1 = absoluteCapacitance(measB, pin2, pin2, measSingl, &capdacB);
  float cap2 = absoluteCapacitance(measC, pin3, pin3, measSingl, &capdacC);


  int capC0Low = (cap0 < thresholdCap*capC0Init);
  int capC1Low = (cap1 < thresholdCap*capC1Init);
  int capC2Low = (cap2 < thresholdCap*capC2Init);


  if (capC1Low || capC2Low || capC0Low){
    //if capacitance on any channel drops too low, re-jam
    activateActuator(onTime, valveF);
    activateActuator(onTime, valveJ);
    logger();
  }
  else{
    logger();
  }
}


void activateActuator(unsigned long onTime, int valvePin){
  unsigned long startTime = millis();

  // turn fingers or jammer motor on, open valve for negative pressure only
  if (valvePin == valveJ){
    digitalWrite(motorNegPress, HIGH);
    digitalWrite(valvePin,HIGH);
  }
  else{
    float pressureRead = mpr.readPressure();
      if ((pressureInit - pressureRead) >  200){
      // if the pressure in the fingers is less than some amount, continue inflation
      digitalWrite(motorPosPress, HIGH);
      }
  }
 
  while ((millis() - startTime) < onTime){
    logger();
  }

  // turn fingers or jammer motor off
  if (valvePin == valveJ){
    digitalWrite(motorNegPress, LOW);
    digitalWrite(valvePin, LOW);
  }
  else{
    digitalWrite(motorPosPress, LOW);
  }
}

void deactivateActuator(unsigned long onTime, int valvePin){
  // turn fingers or jammer motor off, open valve for negative pressure only

  unsigned long startTime = millis();

  if (valvePin == valveJ){
    digitalWrite(motorNegPress, LOW);
    digitalWrite(valvePin,HIGH);
  }
  else{
    digitalWrite(motorPosPress, LOW);
    digitalWrite(valvePin, HIGH);
    }
 
  while ((millis() - startTime) < onTime){
    logger();
  }

  // turn fingers or jammer motor off
  if (valvePin == valveJ){
    digitalWrite(motorNegPress, LOW);
    digitalWrite(valveJ, LOW);
  }
  else{
    digitalWrite(motorPosPress, LOW);
    digitalWrite(valveF, LOW);
  }
}

void logger(){
  pressureTime = millis();
  float pressure_hPa = mpr.readPressure();
  Serial.print("Pressure, ");
  Serial.print(pressureTime);
  Serial.print(", ");
  Serial.println(pressure_hPa);

  float cap0 = absoluteCapacitance(measA, pin1, pin1, measSingl, &capdacA);
  Serial.print("Cap0, ");
  Serial.print(capacitanceTime);
  Serial.print(", ");
  Serial.println(cap0, 4);

  float cap1 = absoluteCapacitance(measB, pin2, pin2, measSingl, &capdacB);
  Serial.print("Cap1, ");
  Serial.print(capacitanceTime);
  Serial.print(", ");
  Serial.println(cap1, 4);

  float cap2 = absoluteCapacitance(measC, pin3, pin3, measSingl, &capdacC);
  Serial.print("Cap2, ");
  Serial.print(capacitanceTime);
  Serial.print(", ");
  Serial.println(cap2, 4);
}



float absoluteCapacitance(uint8_t measSlot, uint8_t chanA, uint8_t chanB, uint8_t measType, uint8_t * capdacPtr) {
  float capacitanceRelative;
  float capacitanceAbsolute = -1;
  if (measType == 0){
    capacitanceRelative =  configTrigRead(measSlot, chanA, chanB, measType, *capdacPtr);
    }
  else{
    //do not need to re-trigger for continuous read
    capacitanceRelative = relativeCapacitance(measSlot);
    }
  capacitanceTime = millis();
  if ((capacitanceRelative <= capMax) && (capacitanceRelative >= -capMax)){
    capacitanceAbsolute = capacitanceRelative + (3.125 * (float)*capdacPtr); //converts capdac to pF
  }
  else{
    Serial.println("Capacitance out of bounds, re-running setCAPDAC");
    setCAPDAC(measSlot, chanA, chanB, measType, capdacPtr);
    }
  return capacitanceAbsolute;
}


float relativeCapacitance(uint8_t measSlot) {  
  delay(1000/readRate);
  uint16_t value[2];
  float capacitanceRelative = capMax + 1;
  if (!fdc.readMeasurement(measSlot, value)) {
    int16_t capBits = value[0];
    float capRatio = (15.0) / (32767.0); //This converts bits (2^15 - 1) into capacitance (+/- 15 pF range)
    capacitanceRelative = (float)(capBits) * capRatio; // value in picofarads (pF)
  }
  return capacitanceRelative;
}


uint8_t setCAPDAC(uint8_t measSlot, uint8_t chanA, uint8_t chanB, uint8_t measType, uint8_t * capdacPtr){
  *capdacPtr = 0;
  if (chanA == chanB){
    //only execute if in single-ended mode. CAPDAC is not set in differential mode


    uint8_t capdacTimeout = 0;
    uint8_t capdacTimeoutLimit = 30;


    float capacitanceRelative = configTrigRead(measSlot, chanA, chanB, measType, *capdacPtr);
    uint8_t capdacFlag = 1;


    while ((capdacTimeout < capdacTimeoutLimit) && (capdacFlag)){
      capdacFlag = adjustCAPDAC(capacitanceRelative, capdacPtr);
      capacitanceRelative = configTrigRead(measSlot, chanA, chanB, measType, *capdacPtr);
      capdacTimeout++;
      delay(5);
    }

    if (capdacFlag){
      //resetting capdac timed out
      *capdacPtr = 0;
    }
  }
  return *capdacPtr;
}


uint8_t adjustCAPDAC(float capacitanceRelative, uint8_t * capdacPtr) {  
  if ((capacitanceRelative < capMax) && (capacitanceRelative > -1*capMax)){
    // if it's in range, adjust capdac so capacitance is as close to 0 as possible
    *capdacPtr += capacitanceRelative/capdacConversion;
    *capdacPtr = max(*capdacPtr, ((uint8_t)0));
    *capdacPtr = min(*capdacPtr, ((uint8_t)FDC1004_CAPDAC_MAX));
    return 0;
  }
  else if (capacitanceRelative <= -1*capMax) {
    *capdacPtr -= 5; //decrease by 15.625 pF, which is just outside the bounds of +/- 15 pF measurement range
    *capdacPtr = max(*capdacPtr, ((uint8_t)0));
    return 1;
  }
  else {
    *capdacPtr += 5; //increase by 15.625 pF, which is just outside the bounds of +/- 15 pF measurement range
    *capdacPtr = min(*capdacPtr, ((uint8_t)FDC1004_CAPDAC_MAX));
    return 1;
  }
}


float configTrigRead(uint8_t measSlot, uint8_t chanA, uint8_t chanB, uint8_t measType, uint8_t capdac){
  fdc.configureMeasurement(measSlot, chanA, chanB, capdac);
  fdc.triggerMeasurement(measSlot, FDC1004_100HZ, measType);
  return relativeCapacitance(measSlot);
}

void sensorSetup(){
  Serial.println("Setting up MPLRS pressure sensor");
  while (! mpr.begin()) {
    Serial.println("Failed to communicate with MPRLS sensor, check wiring?");
    delay(1000);
  }
  Serial.println("Found MPRLS sensor");


  //Begin capacitance measurement
  //Make sure device is present on I2C bus
  Wire.beginTransmission(fdc._addr);
  int error = Wire.endTransmission();
  int I2C_timeout = 10;
  while (I2C_timeout > 0 && error > 0){
    Serial.print("I2C device NOT found. ");
    Serial.println("Waiting for 5 seconds before attempting setup...");
    delay(5000);
    I2C_timeout -= 1;
    error = Wire.endTransmission();
  }
  if (error == 0){
    Serial.println("FDC1004 device found");
  }
  else{
    Serial.print("I2C device NOT found  and timeout reached. Data is invalid.");
  }

  fdc.resetDevice();
  delay(500);
  setCAPDAC(measA, pin1, pin1, measSingl, &capdacA);
  setCAPDAC(measB, pin2, pin2, measSingl, &capdacB);
}







