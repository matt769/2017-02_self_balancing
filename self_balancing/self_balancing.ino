// communication with MPU-6050 handled by Jeff Rowberg's code which can be found here:
// https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050

// PID controller by Brett Beauregard, code can be found here
// https://github.com/br3ttb/Arduino-PID-Library/


#include <PID_v1.h>
#include <SoftwareSerial.h>

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

///////////////////////////////////////////////////////////////////////////////////
//              MPU

// class default I2C address is 0x68 - AD0 low = 0x68
MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

bool newReading = false; //to indicate a new reading has been retrieved

///////////////////////////////////////////////////////////////////////////////////
//              INTERUPT ROUTINE

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}


///////////////////////////////////////////////////////////////////////////////////
//              MOTOR FUNCTIONALITY

const int LeftForwardPin = 8;
const int LeftReversePin = 9;
const int RightForwardPin = 10;
const int RightReversePin = 11;
const int LeftPwmPin = 5;
const int RightPwmPin = 6;
const int MOTOR_MIN_PWM = 25;


///////////////////////////////////////////////////////////////////////////////////
//            Balance and PID controller
double target = 9.0;
double actual;
double output;
double lastOutput;
int modOutput;
double recoverableRange = 30.0;
byte UPRIGHT = 1;
byte LYING_DOWN = 0;
byte STATE = LYING_DOWN;

double kP = 38, kI = 0, kD = 0.2;
PID myPID(&actual, &output, &target, kP, kI, kD, REVERSE);

///////////////////////////////////////////////////////////////////////////////////
//          User Control
bool newSetting = false;
char setting;

///////////////////////////////////////////////////////////////////////////////////
//          For checking loop timings
int counterMain = 0;
long counterWhile = 0;
long counterPID = 0;
unsigned long timeMainStart = 0;
unsigned long timeWhile = 0;
unsigned long timeWhileTmp = 0;

///////////////////////////////////////////////////////////////////////////////////
//          Other

// set up serial connection
SoftwareSerial sSerial1(12, 13); // to RX, TX of BT module


///////////////////////////////////////////////////////////////////////////////////
//              SETUP

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  Serial.begin(115200);   // from computer
  sSerial1.begin(115200); //from bluetooth module

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(-343);
  mpu.setYGyroOffset(-232);
  mpu.setZGyroOffset(28);
  mpu.setZAccelOffset(847);

  //   low pass filter
  byte dlpf = 0;
  mpu.setDLPFMode(dlpf);
  Serial.print(F("DLPF: ")); Serial.println(dlpf);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // allow MPU to settle
  unsigned long tmp;
  tmp = millis();
  while (millis() - tmp < 3000) {
  }
  mpu.resetFIFO();


  // Motor functionality
  pinMode(LeftForwardPin, OUTPUT);
  pinMode(LeftReversePin, OUTPUT);
  pinMode(RightForwardPin, OUTPUT);
  pinMode(RightReversePin, OUTPUT);
  pinMode(LeftPwmPin, OUTPUT);
  pinMode(RightPwmPin, OUTPUT);

  Serial.print(F("Target angle to maintain: ")); Serial.println(target);

  myPID.SetOutputLimits(-255, 255);
  myPID.SetSampleTime(4);
  myPID.SetMode(AUTOMATIC);


  Serial.println(F("Setup complete"));
}


///////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////
//                  MAIN

void loop() {
  if (counterMain == 0) {
    timeMainStart = millis();
  }
  counterMain += 1;

  if (counterMain == 1000) {
    // Serial.print(F("Main loop count: "));Serial.println(counterMain);
    // Serial.print(F("Main loop time: "));Serial.println(millis()-timeMainStart);
    // Serial.print(F("While loop count: "));Serial.println(counterWhile);
    // Serial.print(F("While loop time: "));Serial.println(timeWhile);
    // Serial.print(F("PID count: "));Serial.println(counterPID);
    counterMain = 0;
    counterWhile = 0;
    counterPID = 0;
    timeMainStart = 0;
    timeWhile = 0;
  }

  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {

    /////////////////////////////////////////
    /////////////////////////////////////////
    /////////////////////////////////////////

    counterWhile += 1;
    timeWhileTmp = millis();

    checkForCommands();
    if (newSetting) {
      updateSettings();
    }

    if (newReading) {

      // the following section is currently commented out because it is causing strange behaviour sometimes
      // I believe I am not handling the turning on/off of the PID controller correctly

      // check that robot is in a recoverable position (i.e. close to upright)
      //      if (actual>(target-recoverableRange) && actual<(target+recoverableRange)){
      //        if(STATE==LYING_DOWN){
      //          myPID.SetMode(AUTOMATIC); // turn PID back on
      //        }
      //        STATE = UPRIGHT;
      //      }
      //    else {
      //        STATE = LYING_DOWN;
      //        myPID.SetMode(MANUAL);  // turn off, don't want to accumulate errors
      //                                // not certain this is working properly, seems to have offset when put down and then back upright
      //        Stop();
      //      }
      STATE = UPRIGHT;


      if (STATE == UPRIGHT) {

        if (myPID.Compute()) { // if the PID has not recalculated then no need to update motors

          counterPID += 1;

          // if motor changing direction then stop first
          if ((output > 0 && lastOutput < 0) || (output < 0 && lastOutput > 0)) {
            Stop();
          }

          modOutput = (int)output;

          if (modOutput > 0) {
            modOutput = map(modOutput, 0, 255, MOTOR_MIN_PWM, 255); //re-scale with the minimum the motor will turn at
            GoForwards(modOutput);

          }
          else if (modOutput < 0) {
            modOutput = -modOutput;
            modOutput = map(modOutput, 0, 255, MOTOR_MIN_PWM, 255); //re-scale with the minimum the motor will turn at
            GoBackwards(modOutput);
          }

          lastOutput = output;
        }
        timeWhile += millis() - timeWhileTmp;
      }
      newReading = false;
    }

    /////////////////////////////////////////
    /////////////////////////////////////////
    /////////////////////////////////////////
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    //mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetPitch(ypr, &q, &gravity); // only calculating the pitch, don't need the others
    // pitch angle in degrees
    actual = ypr[1] * 180 / M_PI;
    newReading = true;


  }
}


///////////////////////////////////////////////////////////////////////////////////
// FOR CHANGING SETTINGS

// assumes commands are not continuously sent
void checkForCommands() {
  if (Serial.available()) {
    setting = Serial.read();
    newSetting = true;
  }
  if (sSerial1.available()) {
    setting = sSerial1.read();
    newSetting = true;
  }
}


void updateSettings() {
  if (newSetting) {
    switch (setting) {
      case 'p':
        kP += 1.0;
        Serial.println(kP);
        break;
      case ';':
        kP -= 1.0;
        Serial.println(kP);
        break;
      case 'i':
        kI += 1.0;
        Serial.println(kI);
        break;
      case 'k':
        kI -= 1.0;
        Serial.println(kI);
        break;
      case 'd':
        kD += 0.01;
        Serial.println(kD);
        break;
      case 'c':
        kD -= 0.01;
        Serial.println(kD);
        break;
      case 't':
        target += 0.5;
        Serial.println(target);
        break;
      case 'g':
        target -= 0.5;
        Serial.println(target);
        break;
      case 'w':
        target += 1;
        Serial.println(target);
        break;
      case 's':
        target -= 1;
        Serial.println(target);
        break;
    }
    //    pidY.setFactors(kP, kI, kD);
    myPID.SetTunings(kP, kI, kD);
    newSetting = false;
  }
}


///////////////////////////////////////////////////////////////////////////////////
// OUTPUT

void sendSerialDataFloat(float one, float two, float three) {
  Serial.print(one); Serial.print('\t');
  Serial.print(two); Serial.print('\t');
  Serial.print(three); Serial.print('\t');
  Serial.print('\n');
}

void sendSerialDataInt(int one, int two, int three) {
  Serial.print(one); Serial.print('\t');
  Serial.print(two); Serial.print('\t');
  Serial.print(three); Serial.print('\t');
  Serial.print('\n');
}




///////////////////////////////////////////////////////////////////////////////////
//              MOTOR FUNCTIONALITY

void GoForwards(int pwm) {
  analogWrite(LeftPwmPin, pwm);
  analogWrite(RightPwmPin, pwm);
  digitalWrite(LeftForwardPin, HIGH);
  digitalWrite(LeftReversePin, LOW);
  digitalWrite(RightForwardPin, HIGH);
  digitalWrite(RightReversePin, LOW);
}

void GoBackwards(int pwm) {
  analogWrite(LeftPwmPin, pwm);
  analogWrite(RightPwmPin, pwm);
  digitalWrite(LeftForwardPin, LOW);
  digitalWrite(LeftReversePin, HIGH);
  digitalWrite(RightForwardPin, LOW);
  digitalWrite(RightReversePin, HIGH);
}


void Stop() {
  analogWrite(LeftPwmPin, 0);
  analogWrite(RightPwmPin, 0);
  digitalWrite(LeftForwardPin, LOW);
  digitalWrite(LeftReversePin, LOW);
  digitalWrite(RightForwardPin, LOW);
  digitalWrite(RightReversePin, LOW);
}




