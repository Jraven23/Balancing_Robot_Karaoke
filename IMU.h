#ifndef IMU_H
#define IMU_H

#include "MPU6050_6Axis_MotionApps20.h"

//#define OUTPUT_READABLE_EULER
//#define OUTPUT_READABLE_YAWPITCHROLL

//IMU
//GND - GND
//VCC - VCC
//SDA - Pin A4
//SCL - Pin A5
//INT - Pin 2
#define INTERRUPT_PIN 2

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

// MPU control/status vars
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
//uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
//uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]
VectorInt16 accel_MPU;         // [x, y, z]
VectorInt16 accelReal_MPU;     // [x, y, z]
VectorInt16 accelWorld_MPU;    // [x, y, z]
VectorFloat gravity_MPU;    // [x, y, z]
float ypr[3];           // [yaw, pitch, roll]

volatile bool mpuInterrupt=false;

void dmpDataReady() {
  mpuInterrupt=true;
}
void mpuInthandle() {
  mpuInterrupt = false; 
  
  if ((mpu.getIntStatus() & 0x01) && mpu.dmpPacketAvailable()) {
    
        mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
  
        mpu.dmpGetQuaternion(&q, fifoBuffer);

        mpu.dmpGetEuler(ypr, &q);
#ifdef OUTPUT_READABLE_EULER
        // display Euler angles in degrees
        Serial.print("ypr\t");
        Serial.print(ypr[0] * 180/M_PI);
        Serial.print("\t");
        Serial.print(ypr[1] * 180/M_PI);
        Serial.print("\t");
        Serial.println(ypr[2] * 180/M_PI);
#endif
        
#ifdef OUTPUT_READABLE_YAWPITCHROLL
        mpu.dmpGetGravity(&gravity_MPU, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity_MPU);
        // display Euler angles in degrees from yaw pitch and roll
        Serial.print("ypr\t");
        Serial.print(ypr[0] * 180/M_PI);
        Serial.print("\t");
        Serial.print(ypr[1] * 180/M_PI);
        Serial.print("\t");
        Serial.println(ypr[2] * 180/M_PI);
#endif

#ifdef OUTPUT_READABLE_REALACCEL
        //show acceleration without gravity 
        mpu.dmpGetAccel(&accel_MPU, fifoBuffer);
        mpu.dmpGetGravity(&gravity_MPU, &q);
        mpu.dmpGetLinearAccel(&accelReal_MPU, &accel_MPU, &gravity_MPU); //removing gravity
        Serial.print("Acceleration\t");
        Serial.print(accelReal_MPU.x);
        Serial.print("\t");
        Serial.print(accelReal_MPU.y);
        Serial.print("\t");
        Serial.println(accelReal_MPU.z);
#endif

#ifdef OUTPUT_READABLE_WORLDACCEL
        // display initial world-frame acceleration, adjusted to remove gravity
        // and rotated based on known orientation from quaternion
        mpu.dmpGetAccel(&accel_MPU, fifoBuffer);
        mpu.dmpGetGravity(&gravity_MPU, &q);
        mpu.dmpGetLinearAccel(&accelReal_MPU, &accel_MPU, &gravity_MPU); //removing gravity
        mpu.dmpGetLinearAccelInWorld(&accelWorld_MPU, &accelReal_MPU, &q);
        Serial.print("Acceleration world\t");
        Serial.print(accelWorld_MPU.x);
        Serial.print("\t");
        Serial.print(accelWorld_MPU.y);
        Serial.print("\t");
        Serial.println(accelWorld_MPU.z);
#endif
  }
}

float mpuGetAngle(){
  return ypr[1];
}

void MPUsetup() {        
    // Iniciar MPU6050
    Serial.println(F("Initializing I2C devices..."));
    Wire.begin();
    Wire.setClock(1000000UL);
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // Comprobar  conexion
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // Iniciar DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
        
    // Activar DMP
    if (!devStatus) //Initialized with no errors
    {    
        // Valores de calibracion
        //From calib: -2210  -36 1876  -79 -44 25 (accX accY accZ gyroX gyroY gyroZ)
        mpu.setXAccelOffset(-2213);
        mpu.setYAccelOffset(-45);
        mpu.setZAccelOffset(1892);
        mpu.setXGyroOffset(-79);
        mpu.setYGyroOffset(-45);
        mpu.setZGyroOffset(26);
    
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        
        // Activar interrupcion
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpu.getIntStatus(); //clean interruption

        Serial.println(F("DMP ready! Waiting for first interrupt..."));
   
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
        delay(1000);
        exit(0); //If fails before starting stop program
    }
}


#endif