#include "Arduino.h"
#include "I2Cdev.h"
#include "PID.h"
#include "IMU.h"
#include "stepper_control.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"

#endif

//#include "math.h"


void I2Csetup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
}

//PID
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
volatile unsigned long ticksPerPulseLeft = ULONG_MAX; //ULONG_MAX declared inside stepper_control.h
volatile unsigned long ticksPerPulseRight = ULONG_MAX;

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

void updateControl(unsigned long nowMicros) {
  
  /* Wait until IMU filter will settle */
  static unsigned long timestamp = micros();
  if ((nowMicros < WARMUP_DELAY_US) || (nowMicros - timestamp < 1000 /* 1kHz */) ) { //if (nowMicros - timestamp < 100) { //10kHz
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
  //velocity += accel * dt;

  float leftVelocity = velocity;
  float rightVelocity = velocity;
  //float leftVelocity = velocity - steering;
  //float rightVelocity = velocity + steering;
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
