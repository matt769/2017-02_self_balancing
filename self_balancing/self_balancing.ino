//try using Jeff Rowburg code to get angle
// based on MPU6050_DMP6.ino sketch
// I used a new MPU6050 because I needed to change the orientation - the pitch wasn't calculated well as it was
// I have recalculated gyro offsets

// try to copy this code:
// http://www.instructables.com/id/2-Wheel-Self-Balancing-Robot-by-using-Arduino-and-/


#include <PID_v1.h>

// Based on MPU_6050_DMP6 sketch by Jeff Rowberg

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



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// MPU Register addresses
// all covered in I2Cdec now

// Motor functionality
const int LeftForwardPin = 9;
const int LeftReversePin = 8;
const int RightForwardPin = 10;
const int RightReversePin = 11;
const int LeftPwmPin = 5;
const int RightPwmPin = 6;
const int MOTOR_MIN_PWM = 110;


// balance + PID controller
double target = 6.5;
double actual;
double output;
double lastOutput;
int modOutput;
long lastCalc = 0;
long nowCalc;

//PID pidY; // with current orientation, Y corresponds to forwards/backwards
//float kP, kI, kD;
bool newSetting = false;
char setting;


//Specify the links and initial tuning parameters
double kP=5, kI=0, kD=0;
PID myPID(&actual, &output, &target, kP, kI, kD, REVERSE);


// check loop timings
unsigned long firstLoop = 0;
unsigned long lastLoop = 0;
int countLoop = 0;


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
  Serial.begin(115200);

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(-343);
  mpu.setYGyroOffset(-232);
  mpu.setZGyroOffset(28);
  mpu.setZAccelOffset(1788); // this is what was in JR code - not sure what mine is or how to find out

// using the calibrate MPU sketch
//  mpu.setXGyroOffset(-83);
//  mpu.setYGyroOffset(-55);
//  mpu.setZGyroOffset(10);
//  mpu.setZAccelOffset(847);


  // low pass filter
  byte dlpf = 0;
  mpu.setDLPFMode(dlpf);
  Serial.print(F("DLPF; ")); Serial.println(dlpf);

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


  // Motor functionality
  pinMode(LeftForwardPin, OUTPUT);
  pinMode(LeftReversePin, OUTPUT);
  pinMode(RightForwardPin, OUTPUT);
  pinMode(RightReversePin, OUTPUT);
  pinMode(LeftPwmPin, OUTPUT);
  pinMode(RightPwmPin, OUTPUT);

//  // PID controller - OLD
//  kP = 5.0f;
//  kI = 0.0f;
//  kD = 0.0f;
//  pidY.setFactors(kP, kI, kD);
//  pidY.setLimits(-255, 255);
  
  Serial.print(F("Target angle to maintain: ")); Serial.println(target);

  myPID.SetOutputLimits(-255, 255);
  myPID.SetSampleTime(20);
  myPID.SetMode(AUTOMATIC);
  
  Serial.println(F("Setup complete"));
}


///////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////
//                  MAIN


void loop() {

  //  if(countLoop > 100) {
  //    countLoop = 0;
  //    lastLoop = millis();
  //    Serial.print("Milliseconds for 100 main loops: ");
  //    Serial.println(lastLoop - firstLoop);
  //    firstLoop = lastLoop;
  //  }
  //  countLoop += 1;

  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {

    /////////////////////////////////////////
    // MY CODE

    if (newSetting) {
      updateSettings();
    }

    nowCalc = millis();
    if (nowCalc - lastCalc > 20) {

      lastCalc = nowCalc;
      actual = ypr[1] * 180 / M_PI;
      //    Serial.print(actual);Serial.print("\t");
//                  Serial.println(actual);
      myPID.Compute();
//      output = pidY.calculate(target, actual);
      //    Serial.print(output);Serial.print("\t");
//                Serial.println(output);

      // if motor changing direction then stop first
      if ((output > 0 && lastOutput < 0) || (output < 0 && lastOutput > 0)) {
        Stop();
      }

      modOutput = (int)output;
      //              Serial.print(actual);Serial.print('\t');
      //              Serial.print(actual-target);Serial.print('\t');
      //              Serial.print(output);Serial.print('\t');
      //              Serial.print(modOutput);Serial.print('\t');

      if (modOutput > 0) {
        modOutput = min(modOutput, 255); // cap at 255
        modOutput = map(modOutput, 0, 255, MOTOR_MIN_PWM, 255); //re-scale with the minimum the motor will turn at
        //            Serial.println(modOutput);
        GoForwards(modOutput);

      }
      else if (modOutput < 0) {
        modOutput = -modOutput;
        modOutput = min(modOutput, 255); // cap at 255
        modOutput = map(modOutput, 0, 255, MOTOR_MIN_PWM, 255); //re-scale with the minimum the motor will turn at
        //            Serial.println(modOutput);
        GoBackwards(modOutput);
      }
//      else {
//        Stop();
//      }
//      Serial.println(modOutput);

      lastOutput = output;

    }

    // END MY CODE
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
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  }
}


///////////////////////////////////////////////////////////////////////////////////
// FOR CHANGING SETTINGS

void serialEvent() {
  setting = Serial.read();
  newSetting = true;
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
        kD += 0.5;
        Serial.println(kD);
        break;
      case 'c':
        kD -= 0.5;
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

//void TurnLeft(int pwm) {
//      analogWrite(LeftForwardPin, 0);
//      analogWrite(LeftReversePin, 255);
//      analogWrite(RightForwardPin, 255);
//      analogWrite(RightReversePin, 0);
//}
//
//void TurnRight(int pwm) {
//      analogWrite(LeftForwardPin, 255);
//      analogWrite(LeftReversePin, 0);
//      analogWrite(RightForwardPin, 0);
//      analogWrite(RightReversePin, 255);
//}




