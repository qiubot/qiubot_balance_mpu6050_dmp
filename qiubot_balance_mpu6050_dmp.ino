// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20_new.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */


#define OUTPUT_READABLE_YAWPITCHROLL


#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

#define INFINITE (unsigned long)1000000

#define motorControlDebug false
#define mpuReadingDebug true
#define frequencyDebug false

#define Time_for_intialize (unsigned long)20000000
#define P 0.055f
#define D 0.73f
#define V_upper_limit 10.0f

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
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

uint32_t Now = 0;
uint32_t lastUpdate = 0;
float deltat = 0.0f;
float sum = 0.0f;
uint32_t count = 0;
uint32_t sumCount = 0; 
uint32_t delt_t = 0;




int dir_pins[4] = {34, 36, 38, 40};
int step_pins[4] = {35, 37, 39, 41};
unsigned long time_stamps[4] = {0, 0, 0, 0};
unsigned long delays[4] = {INFINITE, INFINITE, INFINITE, INFINITE};
unsigned long setup_time;
bool initialized = false;
boolean dirs[4] = {true, true, true, true};

//for pid control
float kp_pitch = P;
float kd_pitch = D;
float kp_roll = P;
float kd_roll = D;

float error_pitch = 0.0;
float pre_error_pitch = 0.0;
float error_roll = 0.0;
float pre_error_roll = 0.0;

float dt = 0.01;

float speed_max = 0.0;
float speed_min = 0.0;

float pid_output;

float velocity_pitch = 0.0;
float velocity_roll = 0.0;
float acc_pitch = 0.0;
float acc_roll = 0.0;

float setpoint_pitch = 0.0;
float setpoint_roll = 0.0;

float pid_pitch(float pv)
{
    error_pitch = setpoint_pitch - pv;
    pid_output = kp_pitch * error_pitch + kd_pitch * (error_pitch - pre_error_pitch) / dt;
    pre_error_pitch = error_pitch;
    return pid_output;
}

float pid_roll(float pv)
{
    error_roll = setpoint_roll - pv;
    pid_output = kp_roll * error_roll + kd_roll * (error_roll - pre_error_roll) / dt;
    pre_error_roll = error_roll;
    return pid_output;
}



void set_delay_pitch(float acc)
{
  velocity_pitch += acc * dt;
  if (fabs(velocity_pitch) > V_upper_limit)
  {
    velocity_pitch = velocity_pitch > 0 ? V_upper_limit : -V_upper_limit;
  }
  if(fabs(velocity_pitch) > 0.002)
  {
    if (velocity_pitch>0)
    {
      dirs[2]=true;
      dirs[3]=false;
    }
    else
    {
      dirs[2]=false;
      dirs[3]=true;
    }
    delays[2]= static_cast<unsigned long>(round(fabs(2000 / velocity_pitch)));
    delays[3] = delays[2];
  }
  else
  {
    delays[2] = INFINITE;
    delays[3] = INFINITE;
  }
}


void set_delay_roll(float acc)
{
  velocity_roll += acc * dt;
  if (fabs(velocity_roll) > V_upper_limit)
  {
    velocity_roll = velocity_roll > 0 ? V_upper_limit : -V_upper_limit;
  }
  if(fabs(velocity_roll) > 0.002)
  {
    if (velocity_roll>0)
    {
      dirs[0]=true;
      dirs[1]=false;
    }
    else
    {
      dirs[0]=false;
      dirs[1]=true;
    }
    delays[0]= static_cast<unsigned long>(round(fabs(2000 / velocity_roll)));
    delays[1] = delays[0];
  }
  else
  {
    delays[0] = INFINITE;
    delays[1] = INFINITE;
  }
}


void step(int motor_idx)
{
  //TODO: take care of the time stamp overflow

  //check time stamp
  if(time_stamps[motor_idx] + delays[motor_idx] <= micros())
  {
    //change rotating direction
    if(dirs[motor_idx])
    {
      digitalWrite(dir_pins[motor_idx], HIGH);
    }
    else
    {
      digitalWrite(dir_pins[motor_idx], LOW);
    }
    digitalWrite(step_pins[motor_idx], HIGH);
    time_stamps[motor_idx] = time_stamps[motor_idx] + delays[motor_idx];
    digitalWrite(step_pins[motor_idx], LOW);
  }
    
}



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

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

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
//    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
//    while (Serial.available() && Serial.read()); // empty buffer
//    while (!Serial.available());                 // wait for data
//    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

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

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    for (int i=0; i<3; i++)
    {
      digitalWrite(LED_PIN, true);
      delay(500);
      digitalWrite(LED_PIN, false);
      delay(500);
    }
    setup_time = micros();
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
      
//        if(motorControlDebug)
//        {
//          Serial.print("delay 0: ");
//          Serial.print(delays[0]);
//          Serial.print(" delay 2: ");
//          Serial.print(delays[2]);
//          Serial.print(" delay 1: ");
//          Serial.print(delays[1]);
//          Serial.print(" delay 3: ");
//          Serial.println(delays[3]);
//        }
        
        for(int i = 0; i < 4; i++)
        {
          step(i);
        }
          
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

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
           

             delt_t = millis() - count;
              if(delt_t > 10)
              {
                if(mpuReadingDebug)
                {
                  Serial.print("ypr\t");
                  Serial.print(ypr[0] * 180/M_PI);
                  Serial.print("\t");
                  Serial.print(ypr[1] * 180/M_PI);
                  Serial.print("\t");
                  Serial.println(ypr[2] * 180/M_PI);
                  if(frequencyDebug)
                  {
                    Serial.print("rate = ");
                    Serial.print((float)sumCount/sum, 2);
                    Serial.println(" Hz");
                  }
                }
                  
                count = millis();
                sumCount = 0;
                sum = 0;
        
              }  
              
              Now = micros();
              deltat = ((Now - lastUpdate) / 1000000.0f);
              lastUpdate = Now;
              dt = deltat;

              sum += deltat;
              sumCount++;

              if (initialized)
              {
                set_delay_pitch(pid_pitch(ypr[1] * 180/M_PI));
                set_delay_roll(pid_roll(ypr[2] * 180/M_PI));
              }
              else
              {
                if (Now - setup_time > Time_for_intialize)
                {
                  initialized = true;
                  Serial.print("Initialization finished!");
                  Serial.print("P: ");
                  Serial.print(P);
                  Serial.print(" D: ");
                  Serial.println(D);
                  setpoint_pitch = ypr[1] * 180/M_PI;
                  setpoint_roll = ypr[2] * 180/M_PI;
                  
                  for (int i=0; i<10; i++)
                  {
                    digitalWrite(LED_PIN, true);
                    delay(50);
                    digitalWrite(LED_PIN, false);
                    delay(50);
                  }

                  delay(5000);
                }
              }
              
              if(motorControlDebug)
              {
                Serial.print("delay 0: ");
                Serial.print(delays[0]);
                Serial.print(" delay 2: ");
                Serial.print(delays[2]);
                Serial.print(" delay 1: ");
                Serial.print(delays[1]);
                Serial.print(" delay 3: ");
                Serial.println(delays[3]);
                
                Serial.print("velocity roll: ");
                Serial.print(velocity_roll);
                Serial.print(" velocity pitch: ");
                Serial.println(velocity_pitch);
              }
              
              for(int i = 0; i < 4; i++)
              {
                step(i);
              }

        #endif
    
        // blink LED to indicate activity
        //digitalWrite(LED_PIN, false);
    }
}
