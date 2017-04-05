// robot must be held in upright position during startup in order to determin its target angle (currently commented out)
// min PWM that the motors will react to is about 80



#include <I2C.h>
#include <PID.h>

// sensor readings and calculations
const byte MPU_ADDRESS = 104; // I2C Address of MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ, GyXOffset, GyYOffset, GyZOffset;  // measurement values
float valAcX,valAcY,valAcZ,valTmp,valGyX,valGyY,valGyZ; // calculated values e.g. g (for accel) and m/s^2 (for gyro)
# define bufferSize 20
int16_t bufferAcX[bufferSize], bufferAcY[bufferSize], bufferAcZ[bufferSize], bufferGyX[bufferSize], bufferGyY[bufferSize], bufferGyZ[bufferSize], bufferTime[bufferSize];
int bufferPos = 0;
int lastBufferPos = 0;
long bufferAcXSum = 0, bufferAcYSum = 0, bufferAcZSum = 0, bufferGyXSum = 0, bufferGyYSum = 0, bufferGyZSum = 0;
float accelRes = 2.0f / 32768.0f;
float gyroRes = 250.0f / 32768.0f;
uint16_t lastReadingTime, thisReadingTime, minReadInterval = 50; // for timings
float mixParam = 0.9;
unsigned long last = 0;
unsigned long now = 0;
//const float RAD_TO_DEG = (float)360 / (2 * 3.14159265359);  // this is actually already defined in arduino.h
// note: if need to pass these to functions then must have those functions in additional .h file
struct angles {float x; float y; float z;};
angles currentMixedAngle;
angles currentAccelAngle;
angles currentGyroAngle;
angles changeGyroAngle;
angles currentAngle;

// interupts and process status flags
volatile bool interuptReceived = false;
byte intPin = 2;
byte intStatus;
bool sensorRead = false;
byte i2cTimeout = 0;
bool sensorsRead = false;

// other
uint16_t counter = 0;
bool DEBUG = false;
bool SERIAL_PRINT = false;



// MPU Register addresses
const byte WHO_AM_I = 117;
const byte PWR_MGMT_1 = 107;   // Power management
const byte CONFIG = 26;
const byte FS_SEL = 27; //  Gyro config
const byte AFS_SEL = 28; //  Accelerometer config
const byte INT_PIN_CFG = 55; // Interupt pin config
const byte INT_ENABLE = 56; // Interupt enable
const byte INT_STATUS = 58; // Interupt status

const byte ACCEL_XOUT_H = 59;   // [15:8]
const byte ACCEL_XOUT_L = 60;   //[7:0]
const byte ACCEL_YOUT_H = 61;   // [15:8]
const byte ACCEL_YOUT_L = 62;   //[7:0]
const byte ACCEL_ZOUT_H = 63;   // [15:8]
const byte ACCEL_ZOUT_L = 64;   //[7:0]
const byte TEMP_OUT_H = 65;
const byte TEMP_OUT_L = 66;
const byte GYRO_XOUT_H = 67;  // [15:8]
const byte GYRO_XOUT_L = 68;   //[7:0]
const byte GYRO_YOUT_H = 69;   // [15:8]
const byte GYRO_YOUT_L = 70;   //[7:0]
const byte GYRO_ZOUT_H = 71;   // [15:8]
const byte GYRO_ZOUT_L = 72;   //[7:0]

// Motor functionality
const int LeftForwardPin = 5;
const int LeftReversePin = 6;
const int RightForwardPin = 11;
const int RightReversePin = 10;
const int OutputVal = 255;

// PID controller
PID pidY; // with current orientation, Z corresponds to forwards/backwards

// balance
float target = 80;  //not zero, due to way sensor is orientated // may change later
float actual;
float output;
float lastOutput;
int modOutput;
long lastCalc = 0;
long nowCalc;
int modOutput2;
int deadZone = 10;





///////////////////////////////////////////////////////////////////////////////////
//              SETUP

void setup() {
  Serial.begin(115200);
  Serial.println(F("Starting setup..."));

// Motor functionality
  pinMode(LeftForwardPin, OUTPUT);
  pinMode(LeftReversePin, OUTPUT);
  pinMode(RightForwardPin, OUTPUT);
  pinMode(RightReversePin, OUTPUT);

  // PID controller
  pidY.setFactors(20.0f,0.0f,0.0f);
  pidY.setLimits(-255,255);

  
  // I2C setup
  I2c.begin();
  I2c.timeOut(50);
  I2c.setSpeed(1);  // fast (400Hz)

  // check MPU connected
  byte MPU_ADDRESS_CHECK = readRegister(MPU_ADDRESS,WHO_AM_I);
  if(MPU_ADDRESS_CHECK==MPU_ADDRESS){
    Serial.println(F("MPU-6050 available"));
  }
  else {
    Serial.println(F("ERROR: MPU-6050 NOT FOUND"));
    Serial.println(F("Try reseting..."));
    while(1);
  }

  // set up interupt 0 (which is on pin 2)
  attachInterrupt(0,mpuInterupt,RISING);

  // all config required for MPU
  setupMPU();
  
//   fill buffers
  for (int i = 0; i < bufferSize + 10; i++){
    readMainSensors(MPU_ADDRESS);
    updateBuffers();
  }

//  calibrateGyro(5000);
  GyXOffset = -292;
  GyYOffset = -342;
  GyZOffset = -449;

  
//   initialise angles
  smoothData();
  calcAnglesAccel();
  initialiseCurrentGyroAngles();
  mixAngles();

  // set target angle
//  float tmp = 0;
//  int tmpCnt = 0;
//  for(int k = 0;k<500;k++){
//      readMainSensors(MPU_ADDRESS);
//      if(sensorRead){
//        updateBuffers();
//        smoothData();
//        calcAnglesAccel();
//        calcGyroChange();
//        updateGyroAngles();
//        mixAngles();
//
//        tmp += currentMixedAngle.y;
//        
//        tmpCnt++;
//       }   
//  }
//  target = (tmp * RAD_TO_DEG)/tmpCnt;
//  Serial.println(tmp);
//  Serial.println(tmpCnt);
  target = 83.5f;
  Serial.print(F("Target angle to maintain"));Serial.println(target);
    
  Serial.println(F("Setup complete"));
}

///////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////
//                  MAIN

void loop() {
  
  // START ANGLE CALCULATION
  if(interuptReceived){
//    if(getInteruptStatus(MPU_ADDRESS) & 1 == 1) {
      readMainSensors(MPU_ADDRESS);
//    }
    if(sensorRead){
      updateBuffers();
      smoothData();
      calcAnglesAccel();
      calcGyroChange();
      updateGyroAngles();
      mixAngles();
//    sendSerialDataFloat(currentMixedAngle.x * RAD_TO_DEG,currentMixedAngle.y * RAD_TO_DEG,currentMixedAngle.z * RAD_TO_DEG);
    }   
  interuptReceived = false;
  }
  // FINISH ANGLE CALCULATION


  nowCalc = millis();
  if(nowCalc-lastCalc > 100){  
    lastCalc = nowCalc;
    actual = currentMixedAngle.y * RAD_TO_DEG;
//    Serial.print(actual);Serial.print("\t");
    output = pidY.calculate(target,actual);
//    Serial.print(output);Serial.print("\t");
    
    // if motor changing direction then stop first
    if((output>0 && lastOutput<0) || (output<0 && lastOutput>0)){
      Stop();
    }
  
    modOutput = (int)output;
    Serial.println(modOutput);

    if(abs(modOutput)>deadZone){
  
      if(modOutput>0){
  //      GoForwards(modOutput);
  //        GoForwards(max(100,min(modOutput,255)));
  //        GoForwards(min(modOutput,255));
          GoForwards(max(modOutput,100));
  
  
      }
      else if(modOutput<0){
  //      GoBackwards(-modOutput);
  //      GoBackwards(min(-modOutput,255));
  //      GoBackwards(max(100,min(-modOutput,255)));
          GoBackwards(max(-modOutput,100));
      }
    
      lastOutput = output;
      
    }
  }
 }

///////////////////////////////////////////////////////////////////////////////////
//    INTERUPT ROUTINE

void mpuInterupt(){
  interuptReceived = true;
}



///////////////////////////////////////////////////////////////////////////////////
// OUTPUT

void sendSerialDataFloat(float one, float two, float three){
    Serial.print(one); Serial.print('\t');
    Serial.print(two); Serial.print('\t');
    Serial.print(three); Serial.print('\t');
    Serial.print('\n');
}

void sendSerialDataInt(int one, int two, int three){
    Serial.print(one); Serial.print('\t');
    Serial.print(two); Serial.print('\t');
    Serial.print(three); Serial.print('\t');
    Serial.print('\n');
}



///////////////////////////////////////////////////////////////////////////////////
///         SETUP MPU-6050

void setupMPU(){
  writeBitsNew(MPU_ADDRESS,PWR_MGMT_1,7,1,1);  // resets the device
  delay(50);  // delay desirable after reset
  writeRegister(MPU_ADDRESS,PWR_MGMT_1,0);  // wake up the MPU-6050
//  writeBits(MPU_ADDRESS,INT_PIN_CFG,0,1,0); // Set interupt Active High - this is already default
  writeBitsNew(MPU_ADDRESS,INT_ENABLE,0,1,1); // Enable Data Ready interupt
  writeBitsNew(MPU_ADDRESS,CONFIG,0,3,1); // set low pass filter
  writeBitsNew(MPU_ADDRESS,PWR_MGMT_1,0,3,1); // sets clock source to X axis gyro (as recommended in user guide)
}

///////////////////////////////////////////////////////////////////////////////////
//          INTERACTING WITH MPU-6050

// ADD GENERAL FUNCTION TO READ MULTIPLE REGISTERS

void writeRegister(byte address, byte sensorRegister, byte data) {
 i2cTimeout = I2c.write(address,sensorRegister,data); // start transmission to device
} 

byte readRegister(byte address, byte sensorRegister) {
 i2cTimeout = I2c.read(address, sensorRegister, 1);
 return I2c.receive();
} 


// there's already a write bits function which has been used instead of my own???
void writeBitsNew(byte address, byte registerToWrite,byte startingReplacementBit, byte noReplacementBits, byte replacementValue){
  byte originalregisterValue = readRegister(address,registerToWrite);
  byte newRegisterValue = modifyBits(originalregisterValue,startingReplacementBit,noReplacementBits,replacementValue);
  writeRegister(address,registerToWrite,newRegisterValue);
}


void writeBitsNew2(byte address, byte registerToWrite, byte originalReading, byte startingReplacementBit, byte noReplacementBits, byte replacementValue){
  byte newRegisterValue = modifyBits(originalReading,startingReplacementBit,noReplacementBits,replacementValue);
  writeRegister(address,registerToWrite,newRegisterValue);
}

int readBitsNew(byte address, byte registerToRead,byte startingBit, byte noOfBits){
  byte registerVal = readRegister(address,registerToRead);
  
  // create mask that will just have 1s in bits of interest
  // apply mask to read value
  //return
  return registerVal; //placeholder
}

void readMainSensors(byte address) {
  //
  lastReadingTime = thisReadingTime;
  thisReadingTime = millis();
  I2c.read(address, ACCEL_XOUT_H, 14);
  // read the most significant bit register into the variable then shift to the left
  // and binary add the least significant
  if(I2c.available()==14){
  AcX=I2c.receive()<<8|I2c.receive();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)  
  AcY=I2c.receive()<<8|I2c.receive();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=I2c.receive()<<8|I2c.receive();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=I2c.receive()<<8|I2c.receive();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=I2c.receive()<<8|I2c.receive();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=I2c.receive()<<8|I2c.receive();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=I2c.receive()<<8|I2c.receive();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  sensorRead = true;
  }
  else {
    sensorRead = false;
//    Serial.println(I2c.available());
    flushI2cBuffer();
  }
} 

byte getInteruptStatus(byte address){
  return readRegister(address,INT_STATUS);
}

void flushI2cBuffer(){
  while(I2c.available()){
    I2c.receive();
  }
}


///////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////
//                  CALIBRATE??





///////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////
//            RECORDING HISTORY AND SMOOTHING


void checkReadings(){
  // check reading against previous
  // ideally want to check against next reading as well
  // not sure required - low pass filter seems to remove wierd readings
  
}



void updateBuffers(){
  // keeps track of totals so they don't need to be calculated each time
  
  // remove old value from total
  bufferAcXSum -= bufferAcX[bufferPos];
  bufferAcYSum -= bufferAcY[bufferPos];
  bufferAcZSum -= bufferAcZ[bufferPos];
  bufferGyXSum -= bufferGyX[bufferPos];
  bufferGyYSum -= bufferGyY[bufferPos];
  bufferGyZSum -= bufferGyZ[bufferPos];
  // add new value to buffer
  bufferTime[bufferPos] = thisReadingTime;
  bufferAcX[bufferPos] = AcX;
  bufferAcY[bufferPos] = AcY;
  bufferAcZ[bufferPos] = AcZ;
  bufferGyX[bufferPos] = GyX - GyXOffset;
  bufferGyY[bufferPos] = GyY - GyYOffset;
  bufferGyZ[bufferPos] = GyZ - GyZOffset;
  // add new value to sum
  bufferAcXSum += bufferAcX[bufferPos];
  bufferAcYSum += bufferAcY[bufferPos];
  bufferAcZSum += bufferAcZ[bufferPos];
  bufferGyXSum += bufferGyX[bufferPos];
  bufferGyYSum += bufferGyY[bufferPos];
  bufferGyZSum += bufferGyZ[bufferPos];  
  // increment buffer position and wrap if reach end
  lastBufferPos = bufferPos;
  bufferPos += 1;
  if(bufferPos == bufferSize) bufferPos = 0; //reset position when get to end
  
}

void smoothData(){
  // note assumption that buffers have been filled
  AcX = bufferAcXSum / bufferSize;
  AcY = bufferAcYSum / bufferSize;
  AcZ = bufferAcZSum / bufferSize;
}

///////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////
//                PERFORMING CALCULATIONS WITH SENSOR MEASUREMENTS

void calibrateGyro(int repetitions){
  Serial.println(F("Calibrating gyro"));
  long GyXSum = 0;
  long GyYSum = 0;
  long GyZSum = 0;
  for(int i=0; i<repetitions; i++){
    readMainSensors(MPU_ADDRESS);
    // ADD STEP TO VALIDATE SENSOR READINGS
    GyXSum += GyX;
    GyYSum += GyY;
    GyZSum += GyZ;
  }
  GyXOffset = GyXSum/repetitions;
  GyYOffset = GyYSum/repetitions;
  GyZOffset = GyZSum/repetitions;
  Serial.println(F("Gyro offset values:"));
  sendSerialDataInt(GyXOffset,GyYOffset,GyZOffset);
}



void initialiseCurrentGyroAngles(){
  // this will need to run at the beginning to populate currentAngle
  // just take average from buffer or run more times?
  // buffer must be filled before running this
  currentGyroAngle.x = currentAccelAngle.x;
  currentGyroAngle.y = currentAccelAngle.y;
  currentGyroAngle.z = currentAccelAngle.z;
}

// not used currently
void convertReadingsToValues(){
  valAcX = AcX * accelRes;
  valAcY = AcY * accelRes;
  valAcZ = AcZ * accelRes;
  valGyX = GyX * gyroRes;
  valGyY = GyY * gyroRes;
  valGyZ = GyZ * gyroRes;
}

void calcAnglesAccel(){

  currentAccelAngle.x = atan2(AcY,AcZ);
  currentAccelAngle.y = atan2(AcX,AcZ);
  currentAccelAngle.z = atan2(AcX,AcY);

//    alternative calc
//    tan-1(axis reading / sqrt(other axis ^2 + other axis^2))
//    currentAccelAngle.x = atan(AcX/sqrt(pow(AcY,2) + pow(AcZ,2)));
//    currentAccelAngle.y = atan(AcY/sqrt(pow(AcX,2) + pow(AcZ,2)));
//    currentAccelAngle.z = atan(AcZ/sqrt(pow(AcY,2) + pow(AcX,2)));
}

void calcGyroChange(){
  // lastBufferPos is the most recent measurement taken
  // prevBufferPos is the reading before that
  // I could change the updateBuffers function to update position at the beginning and then wouldn't need to create prevBufferPos
  uint16_t prevBufferPos;
  if (lastBufferPos==0) {prevBufferPos = bufferSize-1;}  //wrap
  else {prevBufferPos = lastBufferPos - 1;}
  uint16_t interval = (bufferTime[lastBufferPos] - bufferTime[prevBufferPos]) * 1000;
  changeGyroAngle.x = ((bufferGyX[lastBufferPos] - bufferGyX[prevBufferPos]) * gyroRes) / interval;  // rad/s
  changeGyroAngle.y = ((bufferGyY[lastBufferPos] - bufferGyY[prevBufferPos]) * gyroRes) / interval;  // rad/s
  changeGyroAngle.z = ((bufferGyZ[lastBufferPos] - bufferGyZ[prevBufferPos]) * gyroRes) / interval;  // rad/s
}

void updateGyroAngles(){
  currentGyroAngle.x = currentMixedAngle.x + changeGyroAngle.x;
  currentGyroAngle.y = currentMixedAngle.y + changeGyroAngle.y;
  currentGyroAngle.z = currentMixedAngle.z + changeGyroAngle.z;
}

float mixAnglesCalc(float angleGyro, float angleAccel, float weight){
  float mixedAngle = (angleGyro * weight) + (angleAccel * (1-weight));
  return mixedAngle;
   // do I need to take into account quadrants at all?
}

void mixAngles(){
  currentMixedAngle.x = mixAnglesCalc(currentGyroAngle.x, currentAccelAngle.x, mixParam);
  currentMixedAngle.y = mixAnglesCalc(currentGyroAngle.y, currentAccelAngle.y, mixParam);
  currentMixedAngle.z = mixAnglesCalc(currentGyroAngle.z, currentAccelAngle.z, mixParam);
}

///////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////
//                OTHER SUPPORING FUNCTIONS

byte modifyBits(byte originalByte, byte startingReplacementBit, byte noReplacementBits, byte replacementValue){
  byte clearMask;
  byte replaceMask;
  byte newByte;
  clearMask = (1<< noReplacementBits)-1;  // set required number of 1s
  clearMask = clearMask << startingReplacementBit; // move to required position
  clearMask = ~clearMask; // invert so that bits to overwrite are 0 in the mask
  newByte = originalByte & clearMask; // apply mask and clear bits of interest
  replaceMask = replacementValue << startingReplacementBit; // move replacement bits to correct position in mask
  newByte = newByte | replaceMask; // apply mask
  return newByte;
}



///////////////////////////////////////////////////////////////////////////////////
//              MOTOR FUNCTIONALITY

void GoForwards(int pwm) {
      analogWrite(LeftForwardPin, pwm);
      analogWrite(LeftReversePin, 0);
      analogWrite(RightForwardPin, pwm);
      analogWrite(RightReversePin, 0);
}

void GoBackwards(int pwm) {
      analogWrite(LeftForwardPin, 0);
      analogWrite(LeftReversePin, pwm);
      analogWrite(RightForwardPin, 0);
      analogWrite(RightReversePin, pwm);
}


void Stop() {
      analogWrite(LeftForwardPin, 0);
      analogWrite(LeftReversePin, 0);
      analogWrite(RightForwardPin, 0);
      analogWrite(RightReversePin, 0);
}

//void TurnLeft(int pwm) {
//      analogWrite(LeftForwardPin, 0);
//      analogWrite(LeftReversePin, pwm);
//      analogWrite(RightForwardPin, pwm);
//      analogWrite(RightReversePin, 0);
//}
//
//void TurnRight(int pwm) {
//      analogWrite(LeftForwardPin, pwm);
//      analogWrite(LeftReversePin, 0);
//      analogWrite(RightForwardPin, 0);
//      analogWrite(RightReversePin, pwm);
//}




