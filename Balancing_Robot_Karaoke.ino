#include "Arduino.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"

#endif

#include <Math.h>

/*
float angle_kp=0;
float angle_kd=0;
float AngleSetPoint = 0;

void AnglePDInit(float kp, float kd, float setpoint)
{
  angle_kp = kp;
  angle_kd = kd;
  AngleSetPoint = setpoint;
}

long time_log = millis();
float AngleLastError = 0;
long AngleLastTime = -100;
const long A_sample_time = 0;
float AngleOutput = 0;

float AngleLoopUpdate(float angle_feedback)
{
  float AngleError = angle_feedback - AngleSetPoint;  // calculate the angle error 
  long AngleCurrentTime = millis();
  long delta_time = AngleCurrentTime - AngleLastTime;
  if (delta_time >= A_sample_time && delta_time > 0)
  {
    AngleLastTime = AngleCurrentTime;
    float AngleDTerm = (AngleError - AngleLastError) / delta_time;  // calculate the differential of angle error 
    AngleOutput = -1 * (angle_kp * AngleError + angle_kd * AngleDTerm); // calculate output 
    AngleLastError = AngleError;  // update AngleLastError 
  }
  return AngleOutput;
}
*/

//IMU

//#define OUTPUT_READABLE_EULER
//#define OUTPUT_READABLE_YAWPITCHROLL

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high
//IMU
//GND - GND
//VCC - VCC
//SDA - Pin A4
//SCL - Pin A5
//INT - Pin 2
#define INTERRUPT_PIN 2

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

void I2Csetup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
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

//STEPPERS
// defines pins numbers
const int stepPinR = 5; 
const int dirPinR = 7; 
const int stepPinL = 6; 
const int dirPinL = 8;
const int enablePin = 4; 

void SteppersSetup() {
  //STEPPERS
  // Sets the two pins as Outputs
  pinMode(stepPinR,OUTPUT); 
  pinMode(dirPinR,OUTPUT);
  pinMode(stepPinL,OUTPUT); 
  pinMode(dirPinL,OUTPUT);
  pinMode(enablePin,OUTPUT);
  digitalWrite(stepPinR, LOW);
  digitalWrite(stepPinL, LOW);
  digitalWrite(enablePin, HIGH);
}

void enableMoveForwards() {
  digitalWrite(dirPinR,HIGH); // Enables the motor to move in a particular direction
  digitalWrite(dirPinL,LOW);
  //PORTD |= _BV(7);
  //PORTB &= ~_BV(0); // digitalWrite(dirPinL, LOW);
}

void enableMoveBackwards() {
  digitalWrite(dirPinR,LOW); // Enables the motor to move in a particular direction
  digitalWrite(dirPinL,HIGH);
  //PORTD &= ~_BV(7);
  //PORTB |= _BV(0); // digitalWrite(dirPinL, HIGH);
}

void enableTurnRight() {
  digitalWrite(dirPinR,HIGH); // Enables the motor to move in a particular direction
  digitalWrite(dirPinL,HIGH);
  //PORTD |= _BV(7);
  //PORTB |= _BV(0);
}

void enableTurnLeft() {
  digitalWrite(dirPinR,LOW); // Enables the motor to move in a particular direction
  digitalWrite(dirPinL,LOW);
  //PORTD &= ~_BV(7);
  //PORTB &= ~_BV(0);
}

void enableSteppers(){
  digitalWrite(enablePin,LOW); // Enables movement
  //PORTD &= ~_BV(4);
}

void disableSteppers(){
  digitalWrite(enablePin,HIGH); // disables movement
  //PORTD |= _BV(4);
}

//PID

// Class (should be in diff files)
class PID {
    public:
        PID(float Kp, float Kd, float Ki, float target);        
        float getControl(float value, float dt_seconds);
        void setSettings(float Kp, float Kd, float Ki);
        void setTarget(float target);
    private:
        float _Kp;
        float _Kd;
        float _Ki;
        float _lastError;
        float _integralError;
        float _target;
        float _last_value;
        bool _has_last_value = false;
};

PID::PID(float Kp, float Kd, float Ki, float target) {
    _Kp = Kp;
    _Kd = Kd;
    _Ki = Ki;
    _target = target;
}

float PID::getControl(float value, float dt_seconds) {
    float lastValue = _has_last_value ? _last_value : value;
    float error = _target - value;
    float de = -(value - lastValue) / dt_seconds;
    _integralError += _Ki * error * dt_seconds;
    _lastError = error;
    _last_value = value;
    _has_last_value = true;
    return (_Kp * error + _Kd * de + _integralError);
}

void PID::setSettings(float Kp, float Kd, float Ki) {
  _Kp = Kp;
  _Kd = Kd;
  _Ki = Ki;
}

void PID::setTarget(float target) {
    _target = target;
    _integralError = .0;
}

#define PPR   1600 //Pulses per revolution or steps per revolution
// #define TICKS_PER_SECOND  40000 // 40kHz
#define TICKS_PER_SECOND  50000 // 50kHz
#define PULSE_WIDTH 1

#define MAX_ACCEL (200)
#define ANGLE_Kp  250 //450
#define ANGLE_Kd  10
#define ANGLE_Ki  10

#define VELOCITY_Kp  0.007
#define VELOCITY_Kd  0.0
#define VELOCITY_Ki  0.0005

#define WARMUP_DELAY_US (5000000UL)

float pid_settings[6] = {
  ANGLE_Kp, ANGLE_Kd, ANGLE_Ki,
  VELOCITY_Kp, VELOCITY_Kd, VELOCITY_Ki
};

volatile unsigned long currentTickLeft = 0UL;
volatile unsigned long currentTickRight = 0UL;
volatile unsigned long ticksPerPulseLeft = UINT64_MAX;
volatile unsigned long ticksPerPulseRight = UINT64_MAX;

bool isBalancing = false;

float angle = 0.0;
float targetAngle = 0.0;
volatile float velocity = 0.0;
float targetVelocity = 0.0;
volatile float accel = 0.0;
float steering = 0.0f;

PID anglePID(ANGLE_Kp, ANGLE_Kd, ANGLE_Ki, targetAngle);
PID velocityPID(VELOCITY_Kp, VELOCITY_Kd, VELOCITY_Ki, targetVelocity);

unsigned long lastUpdateMicros = 0;

void setTimer1(int ocra) {  
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1  = 0; // initialize counter value to 0  
  
  // ocra = 16MHz / prescaler / desired_f - 1
  OCR1A = ocra;
  TCCR1B |= (1 << WGM12); // turn on CTC mode
  TCCR1B |= (1 << CS11);  // set prescaler to 8  
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
}

void setTimers() {
  cli();
  // setTimer1(49); // 40kHz
  setTimer1(39); // 50kHz
  sei();
}
  
unsigned long getTicksPerPulse(float velocity) {
  if (abs(velocity) < 1e-3) {
    // TODO: disable motor
    disableSteppers();
    return UINT64_MAX;
  } else {
    enableSteppers();
    return (uint64_t)(2.0 * PI * TICKS_PER_SECOND / (abs(velocity) * PPR)) - PULSE_WIDTH;
  }
}

void updateVelocity(unsigned long nowMicros) {

  static unsigned long timestamp = micros();
  if (nowMicros - timestamp < 100 /* 10kHz */) {
    return;
  }

  float dt = ((float) (nowMicros - timestamp)) * 1e-6;
  velocity += accel * dt;
  
  float leftVelocity = velocity - steering;
  float rightVelocity = velocity + steering;
  ticksPerPulseLeft = getTicksPerPulse(leftVelocity);
  ticksPerPulseRight = getTicksPerPulse(rightVelocity);
  
  if (leftVelocity>0)
    digitalWrite(dirPinL, HIGH);    
  else
    digitalWrite(dirPinL, LOW);

  if (rightVelocity>0)
    digitalWrite(dirPinR, LOW);  
  else
    digitalWrite(dirPinR, HIGH);  

  timestamp = nowMicros;
}

void updateControl(unsigned long nowMicros) {
  
  //updateVelocity(nowMicros);
  
  /* Wait until IMU filter will settle */
  static unsigned long timestamp = micros();
  if ((nowMicros < WARMUP_DELAY_US) || (nowMicros - timestamp < 1000 /* 1kHz */) ) {
    return;
  }

  if (!mpuInterrupt) {
    return;
  }
  mpuInthandle();
  angle = mpuGetAngle();

  float dt = ((float) (nowMicros - timestamp)) * 1e-6;
  if (abs(angle - targetAngle) < PI / 15) {
    isBalancing = true;
  }

  if (abs(angle - targetAngle) > PI / 4) {
    isBalancing = false;
    accel = 0.0;
    velocity = 0.0;    
  }

  if (!isBalancing) {
    return;
  }

  //targetAngle = -velocityPID.getControl(velocity, dt);
  //anglePID.setTarget(targetAngle);

  velocity = anglePID.getControl(angle, dt);
  //accel = constrain(accel, -MAX_ACCEL, MAX_ACCEL);

  float leftVelocity = velocity;
  float rightVelocity = velocity;
  ticksPerPulseLeft = getTicksPerPulse(leftVelocity);
  ticksPerPulseRight = getTicksPerPulse(rightVelocity);
  
  if (leftVelocity<0)
    digitalWrite(dirPinL, HIGH);    
  else
    digitalWrite(dirPinL, LOW);

  if (rightVelocity<0)
    digitalWrite(dirPinR, LOW);  
  else
    digitalWrite(dirPinR, HIGH);  

  timestamp = nowMicros;
}

void setup() {
  
    I2Csetup();
    Serial.begin(115200);
    MPUsetup();
    SteppersSetup();
    enableSteppers();
    setTimers();
    //CONTROL!
    //PD
    //AnglePDInit(A_kp,A_kd,A_Setpoint); //p, d, setpoint?
    Serial.println("Invered Pendulum");
    Serial.print("Angle_KP:");
    Serial.print(ANGLE_Kp);
    Serial.print(" ");
    Serial.print("Angle_KD:");
    Serial.print(ANGLE_Kd);
    Serial.print(" ");
    Serial.print("Angle_KI:");
    Serial.print(ANGLE_Ki);
    Serial.print(" ");
    Serial.print("Velocity_KP:");
    Serial.print(VELOCITY_Kp);
    Serial.print(" ");
    Serial.print("Velocity_KD:");
    Serial.print(VELOCITY_Kd);
    Serial.print(" ");
    Serial.print("Velocity_KI:");
    Serial.print(VELOCITY_Ki);
    Serial.println("-----------------------------");
    //
}

void loop() {
     updateControl(micros());        
}

ISR(TIMER1_COMPA_vect) {
  if (currentTickLeft >= ticksPerPulseLeft) {
    currentTickLeft = 0;
  }
  if (currentTickLeft == 0) {
    PORTD |= _BV(PD6); // digitalWrite(stepPinL, HIGH);
  } else if (currentTickLeft == PULSE_WIDTH) {
    PORTD &= ~_BV(PD6); // digitalWrite(stepPinL, LOW);
  }
  currentTickLeft++;

  if (currentTickRight >= ticksPerPulseRight) {
    currentTickRight = 0;
  }
  if (currentTickRight == 0) {
    PORTD |= _BV(PD5); // digitalWrite(stepPinR, HIGH);
  } else if (currentTickLeft == PULSE_WIDTH) {
    PORTD &= ~_BV(PD5); // digitalWrite(stepPinR, LOW);    
  }
  currentTickRight++;
}
