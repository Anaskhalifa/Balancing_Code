#include <SoftwareSerial.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
//#define LED_PIN 13
bool blinkState = false;
#define ena 3
#define enb 9 
#define in1 11
#define in2 10
#define in3 5
#define in4 4

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

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;

}
//////////////////////////////////////////////
float e,e_old,u,get_angle,e_dot,sl,sr,s_old,d;
float P,I,D,E,pwm;
float ref=0;





void setup() {
  Serial.begin(115200);

DDRD|= B00111000;
DDRB|= B00001110;
 /* pinMode(ena,OUTPUT);
  pinMode(enb,OUTPUT);
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  pinMode(in3,OUTPUT);
  pinMode(in4,OUTPUT);*/
  
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

   //  Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
//    Serial.println(F("Testing device connections..."));
//    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
// Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
     mpu.setXGyroOffset(125);
    mpu.setYGyroOffset(21);
    mpu.setZGyroOffset(53);
    mpu.setXAccelOffset(-845);
    mpu.setYAccelOffset(-1055);
    mpu.setZAccelOffset(1359); 

    if (devStatus == 0) {
      //  Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        
//        Serial.print(F("DMP Initialization failed (code "));
//        Serial.print(devStatus);
//        Serial.println(F(")"));
    }
   
 
 

}

void loop() {                     //BALANCE CODE ONLY 
   
  /*  Dear matrexn3 :D 
   *  1- first check cw & ccw functions by writing it only in void loop then make other codes below as comments 
   *  2- second check imuu() functions & make sure that this variable (ypr[1] * 180/M_PI) belongs to the angle that changes with forward & backward direction 
   *  3- third check that when the error > 0 that the function cw() is the right function to decrease this error & make it reaching 0 otherwise you can change it to ccw(pwm) & make the function below else cw(pwm)
   *  4- finally check the hole code as below & u can see the serial plotter which u can see what happens to your curve of your PID control ^_^
*/


  imuu();
  Serial.println(ref);
  Serial.println(ypr[1] * 180/M_PI);
  PID_control(7.4,0.42,0.4);  //adjust your own tuning based on your model :) pid(kp,ki,d) first tune kp then kd & "tune ki only if u need it"  
  
  if(e>0)
   cw(pwm);
    
  else 
   ccw(-pwm);


}


//////////////////////////////////////////////////////////////////////
void PID_control(float kp,float ki,float kd)
{
  e=ref-ypr[1] * 180/M_PI; 
 P=kp*e;
 E=e+e_old;
 e_dot=e-e_old;
 I=ki*E;
 D=kd*e_dot;
 pwm=P+I+D;
 e_old=e;
 if(pwm>255)
 {
  pwm=255;
 }
 if(pwm<-255)
 {
  pwm=-255;
 }
}
/////////////////////////////////////////////////////////////////////////
void imuu()
{
  if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
    }

     mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
       // Serial.println(F(" FIFO overflow! "));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

//        #ifdef OUTPUT_READABLE_QUATERNION
//            // display quaternion values in easy matrix form: w x y z
//            mpu.dmpGetQuaternion(&q, fifoBuffer);
//            Serial.print("quat\t");
//            Serial.print(q.w);
//            Serial.print("\t");
//            Serial.print(q.x);
//            Serial.print("\t");
//            Serial.print(q.y);
//            Serial.print("\t");
//            Serial.println(q.z);
//        #endif

//        #ifdef OUTPUT_READABLE_EULER
//            // display Euler angles in degrees
//            mpu.dmpGetQuaternion(&q, fifoBuffer);
//            mpu.dmpGetEuler(euler, &q);
//            Serial.print("euler\t");
//            Serial.print(euler[0] * 180/M_PI);
//            Serial.print("\t");
//            Serial.print(euler[1] * 180/M_PI);
//            Serial.print("\t");
//            Serial.println(euler[2] * 180/M_PI);
//        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//            Serial.print("ypr\t");
//            Serial.print(ypr[0] * 180/M_PI);
//            Serial.print("\t");
//            Serial.print(ypr[1] * 180/M_PI);
           //Serial.print("\t");
            //Serial.println(ypr[2] * 180/M_PI);
           
        #endif

//        #ifdef OUTPUT_READABLE_REALACCEL
//            // display real acceleration, adjusted to remove gravity
//            mpu.dmpGetQuaternion(&q, fifoBuffer);
//            mpu.dmpGetAccel(&aa, fifoBuffer);
//            mpu.dmpGetGravity(&gravity, &q);
//            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//            Serial.print("areal\t");
//            Serial.print(aaReal.x);
//            Serial.print("\t");
//            Serial.print(aaReal.y);
//            Serial.print("\t");
//            Serial.println(aaReal.z);
//        #endif

//        #ifdef OUTPUT_READABLE_WORLDACCEL
//            // display initial world-frame acceleration, adjusted to remove gravity
//            // and rotated based on known orientation from quaternion
//            mpu.dmpGetQuaternion(&q, fifoBuffer);
//            mpu.dmpGetAccel(&aa, fifoBuffer);
//            mpu.dmpGetGravity(&gravity, &q);
//            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
//            Serial.print("aworld\t");
//            Serial.print(aaWorld.x);
//            Serial.print("\t");
//            Serial.print(aaWorld.y);
//            Serial.print("\t");
//            Serial.println(aaWorld.z);
//        #endif
    
//        #ifdef OUTPUT_TEAPOT
//            // display quaternion values in InvenSense Teapot demo format:
//            teapotPacket[2] = fifoBuffer[0];
//            teapotPacket[3] = fifoBuffer[1];
//            teapotPacket[4] = fifoBuffer[4];
//            teapotPacket[5] = fifoBuffer[5];
//            teapotPacket[6] = fifoBuffer[8];
//            teapotPacket[7] = fifoBuffer[9];
//            teapotPacket[8] = fifoBuffer[12];
//            teapotPacket[9] = fifoBuffer[13];
//            Serial.write(teapotPacket, 14);
//            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
//        #endif

        // blink LED to indicate activity
//        blinkState = !blinkState;
//        digitalWrite(LED_PIN, blinkState);
    }
}




/////////////////////////////////////////////
  void cw(double x)
{
  PORTB&= ~B00000100;
  PORTD&= ~B00010000;
  PORTD|= B00100000;
  PORTB|= B00001000;
  
  
  analogWrite(ena,x);
  analogWrite(enb,x);
  /*digitalWrite(in1,1);
  digitalWrite(in2,0);
  digitalWrite(in3,1);
  digitalWrite(in4,0);*/
}
///////////////////////////////////////
void ccw(double x)
{
  PORTD&= ~B00100000;
  PORTB&= ~B00001000;
  PORTD|= B00010000;
  PORTB|= B00000100;
  
  analogWrite(enb,x);
  analogWrite(ena,x);
  /*digitalWrite(in1,0);
  digitalWrite(in2,1);
  digitalWrite(in3,0);
  digitalWrite(in4,1);*/
}

