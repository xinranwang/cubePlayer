#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include <SPI.h>
#include <SdFat.h>
#include <SdFatUtil.h>
#include <SFEMP3Shield.h>

MPU6050 mpu;
SFEMP3Shield MP3player;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// My variables
float rotX = 0;
float rotY = 0;
float rotZ = 0;

int states[4][3];

int stateCount = 0;

int accStates[3] = {
  0, 0, 0}; // acceleration state
int pAccStates[3] = {
  0, 0, 0}; // previous acceleration states

int pTempStates[3] = {
  0, 0, 0};

float pEuler[3];

long lastTime = 0;

int totalTracks = 14;
int currentTrack = 14;

boolean pauseFlag = false;
boolean playFlag = false;
boolean moveFlag = false;

// Mp3 player
byte result;

char title[30];
char artist[30];
char album[30];

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
  // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  Serial.print(F("Free RAM = ")); // available in Version 1.0 F() bases the string to into Flash, to use less SRAM.
  Serial.print(FreeRam(), DEC);  // FreeRam() is provided by SdFatUtil.h
  Serial.println(F(" Should be a base line of 1007, on ATmega328 when using INTx"));

  //boot up the MP3 Player Shield
  result = MP3player.begin();
  //check result, see readme for error codes.
  if(result != 0) {
    Serial.print(F("Error code: "));
    Serial.print(result);
    Serial.println(F(" when trying to start MP3 player"));
    if ( result == 6 ) {
      Serial.println(F("Warning: patch file not found, skipping.")); // can be removed for space, if needed.
      Serial.println(F("Use the \"d\" command to verify SdCard can be read")); // can be removed for space, if needed.
    }
  }

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  //  while (!Serial.available());                 // wait for data
  //  while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(1, dmpDataReady, RISING); //// change 0 to 1 to avoid conflict
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } 
  else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  // = {-1, 0, 0, 0, -1, 0, 1, 0, 0, 0, 1, 0}
  states[0][0] = -1;
  states[0][1] = 0;
  states[0][2] = 0;
  states[1][0] = 0;
  states[1][1] = -1;
  states[1][2] = 0;
  states[2][0] = 1;
  states[2][1] = 0;
  states[2][2] = 0;
  states[3][0] = 0;
  states[3][1] = 1;
  states[3][2] = 0;
}

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
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
  } 
  else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);
    mpu.dmpGetAccel(&aa, fifoBuffer);

    // accelerometer: aa.x; aa.y; aa.z;
    recordAccStates();

    //    Serial.print(accStates[0]);
    //    Serial.print("\t");
    //    Serial.print(accStates[1]);
    //    Serial.print("\t");
    //    Serial.println(accStates[2]);


    rotZ = euler[0] * 180/M_PI;
    rotY = euler[1] * 180/M_PI;
    rotX = euler[2] * 180/M_PI;

    //Serial.println(aa.z);
    
    // STOP
    if(accStates[2] == -1) { 
      //Serial.println("+");
      MP3player.stopTrack();
      pauseFlag = false;
      playFlag = false;
    }
    
    // PLAY
    else if (accStates[2] == 0) {
      //Serial.println(0);

      if(playFlag == false) {
        if(pauseFlag == true) {
          MP3player.resumeDataStream();
          pauseFlag = false;

        } 
        else
          //if(pauseFlag == false) 
        {
          MP3player.playTrack(currentTrack);

        }
        playFlag = true;
      }
//      else if(moveFlag == true) {
//        Serial.println("moveFlag");
//        if(isNext() == 1) {
//          Serial.println("next");
//          currentTrack++;
//          MP3player.stopTrack();
//          playFlag = false;
//        }
//        else if(isNext() == -1) {
//          Serial.println("back");
//          currentTrack--;
//          MP3player.stopTrack();
//          playFlag = false;
//        }
//        currentTrack = currentTrack % totalTracks;
//        if (currentTrack == 0) currentTrack = totalTracks;
//        //        MP3player.playTrack(currentTrack);
//      }
//      Serial.println(currentTrack);
    }
    
    // PAUSE
    else if (accStates[2] == 1) {
      //Serial.println("-");
      if(MP3player.isPlaying() == 1) {
        MP3player.pauseDataStream();
        pauseFlag = true;
        playFlag = false;
      }
    }
  }
}

void recordAccStates() {
  int tempStates[3] = {
    0, 0, 0                        };

  for (int i = 0; i < 3; i++) {
    //tempStates[i] = accStates[i];
    tempStates[i] = pTempStates[i];

  }
  //int nowStates[3];

  if(aa.x > 6000) {
    tempStates[0] = 1;
  }
  else if(aa.x > -2000 && aa.x < 2000) {
    tempStates[0] = 0;
  }
  else if(aa.x < -6000) {
    tempStates[0] = -1;
  }

  if(aa.y > 6000) {
    tempStates[1] = 1;
  }
  else if(aa.y > -2000 && aa.y < 2000) {
    tempStates[1] = 0;
  }
  else if(aa.y < -6000) {
    tempStates[1] = -1;
  }

  if(aa.z > 6000) {
    tempStates[2] = 1;
  }
  else if(aa.z > -2000 && aa.z < 2000) {
    tempStates[2] = 0;
  }
  else if(aa.z < -6000) {
    tempStates[2] = -1;
  }

  //if(tempStates[0] == accStates[0] && tempStates[1] == accStates[1] && tempStates[2] == accStates[2])
  if(tempStates[0] == pTempStates[0] && tempStates[1] == pTempStates[1] && tempStates[2] == pTempStates[2])
    stateCount++;
  else 
    stateCount = 0;

  for (int i = 0; i < 3; i++) {
    pTempStates[i] = tempStates[i];
  }

  //if(tempStates[0] != accStates[0] || tempStates[1] != accStates[1] || tempStates[2] != accStates[2])
  if(stateCount > 20 && (tempStates[0] != accStates[0] || tempStates[1] != accStates[1] || tempStates[2] != accStates[2]))
  {
    int positiveCount = 0;
    int negativeCount = 0;
    for(int i = 0; i < 3; i++) {
      if(tempStates[i] == 1) positiveCount++;
      if(tempStates[i] == -1) negativeCount++;
    }
    //return count;
    if((positiveCount == 1 && negativeCount == 0) || (positiveCount == 0 && negativeCount == 1)) {
      for (int i = 0; i < 3; i++) {
        pAccStates[i] = accStates[i];
        accStates[i] = tempStates[i];
      }
      pEuler[0] = euler[0] * 180/M_PI;
      pEuler[1] = euler[1] * 180/M_PI;
      pEuler[2] = euler[2] * 180/M_PI;
      moveFlag = true;
      Serial.println("move!");
    }
    else moveFlag = false;
    Serial.println("moveFlag = false;");
    //lastTime = millis();
  }

  //  Serial.print("pAcc\t");
  //  for(int i = 0; i < 3; i++) {
  //
  //    Serial.print(pAccStates[i]);
  //    Serial.print("\t");
  //  }
  //  Serial.println();
  //
  //  Serial.print("acc\t");
  //  for(int i = 0; i < 3; i++) {
  //    Serial.print(accStates[i]);
  //    Serial.print("\t");
  //  }
  //  Serial.println();

  //  for(int i = 0; i < 4; i++) {
  //    for(int j = 0; j < 3; i++) {
  //      Serial.print(states[i][j]);
  //      Serial.print("\t");
  //    }
  //    Serial.println();
  //  }
}

int isNext() {
  int currentStateIndex;
  for(int i = 0; i < 4; i++) {
    int count = 0;
    for(int j = 0; j < 3; j++) {
      if(accStates[j] == states[i][j]) count++;
    }
    if(count == 3)
      currentStateIndex = i;
  }

  if(pAccStates[0] == states[(currentStateIndex - 1) % 4][0] 
    && pAccStates[1] == states[(currentStateIndex - 1) % 4][1] 
    && pAccStates[2] == states[(currentStateIndex - 1) % 4][2])
    return 1;

  else if(pAccStates[0] == states[(currentStateIndex + 1) % 4][0] 
    && pAccStates[1] == states[(currentStateIndex + 1) % 4][1] 
    && pAccStates[2] == states[(currentStateIndex + 1) % 4][2])
    return -1;


  return 0;
}












