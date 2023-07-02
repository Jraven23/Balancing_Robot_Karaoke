#include "Arduino.h"
#include "I2Cdev.h"
#include "PID.h"
#include "IMU.h"
#include "stepper_control.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

struct RobotState {
  bool Forward=false, Backward=false, Left=false, Right=false, Stop=true;
  unsigned short velocity = 0;
};

RobotState rStateData; // create robot state data

void I2Csetup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
}

//Bluetooth commands:

void interpretCommand(char cCommand, RobotState &rState)
{
  if (cCommand == 'F' || cCommand == 'G' || cCommand == 'I') //Forward
    rState.Forward = true;
  else
    rState.Forward = false;

  if (cCommand == 'B' || cCommand == 'H' || cCommand == 'J') //Backward
    rState.Backward = true;
  else
    rState.Backward = false;

  if (cCommand == 'L' || cCommand == 'G' || cCommand == 'H') //Left
    rState.Left = true;
  else
    rState.Left = false;

  if (cCommand == 'R' || cCommand == 'I' || cCommand == 'J') //Right
    rState.Right = true;
  else
    rState.Right = false;
  
  if (cCommand == 'S') //Stop
    rState.Stop=true;
  else
    rState.Stop=false;

  if(cCommand == '1' || cCommand == '2' || cCommand == '3' || cCommand == '4' || cCommand == '5' || cCommand == '6' || cCommand == '7' || cCommand == '8' || cCommand == '9')
    rState.velocity = cCommand-'0';
  else if(cCommand == 'q')
    rState.velocity = 10;
  else if(cCommand == '0')
    rState.velocity = 0;
    
}

//PID
#define MAX_ACCEL (200)
#define MAX_VEL (1500)
#define Position_Kp  0.05 //300 //450
#define Position_Kd  0.002 //10
#define Position_Ki  0.002 //10

#define STEERING_Kp  10 //450
#define STEERING_Kd  1
#define STEERING_Ki  1

#define VELOCITY_Kp  1//0.007
#define VELOCITY_Kd  0
#define VELOCITY_Ki  0//0.0005

#define WARMUP_DELAY_US (5000000UL)

/*float pid_settings[9] = {
  ANGLE_Kp, ANGLE_Kd, ANGLE_Ki,
  STEERING_Kp, STEERING_Kd, STEERING_Ki,
  VELOCITY_Kp, VELOCITY_Kd, VELOCITY_Ki
};*/

volatile unsigned long currentTickLeft = 0UL;
volatile unsigned long currentTickRight = 0UL;
volatile unsigned long ticksPerPulseLeft = ULONG_MAX; //ULONG_MAX declared inside stepper_control.h
volatile unsigned long ticksPerPulseRight = ULONG_MAX;

bool isBalancing = false;

float angle = 0.0;
float targetAngle = 0.0;
float Robot_Pos = 0.0;
float targetPos = 0.0;
float Robot_AccX = 0.0;
float Robot_VelX = 0.0;
volatile float wheel_velocity = 0.0;
float targetWheelVelocity = 0.0;
volatile float accel = 0.0;
float steering = 0.0f;
float targetSteering = 0.0;
float steeringVel = 0.0;
float leftVelocity = 0.0;
float rightVelocity =  0.0;

PID positionPID(Position_Kp, Position_Kd, Position_Ki, targetPos);
PID steeringPID(STEERING_Kp, STEERING_Kd, STEERING_Ki, targetSteering);
//PID velocityPID(VELOCITY_Kp, VELOCITY_Kd, VELOCITY_Ki, targetVelocity);

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
    wheel_velocity = 0.0;    
  }

  if (!isBalancing) {
    return;
  }

  /*
  if (rStateData.Forward)
    targetVelocity = rStateData.velocity;
  else if (rStateData.Backward)
    targetVelocity = rStateData.velocity;
  else
    targetVelocity = 0;

  //velocityPID.setTarget(targetVelocity);
  //targetAngle = -velocityPID.getControl(velocity, dt);
  //anglePID.setTarget(targetAngle);
  */

  if (rStateData.Forward)
  {
    targetPos += rStateData.velocity*dt;
    positionPID.setTarget(targetPos);
  }
  else if (rStateData.Backward)
  {
    targetPos -= rStateData.velocity*dt;
    positionPID.setTarget(targetPos);
  }

  
  // v(t) = v(t-1)+dt*a

  Robot_AccX = mpuGetXAcc();
  Robot_VelX += Robot_AccX*dt;
  //Robot_Pos += Robot_VelX*dt;
  Robot_Pos += Robot_VelX*dt + 0.5*Robot_AccX*dt*dt;

  //Serial.print("Robot_Pos:");
  //Serial.println(Robot_Pos);
  accel = positionPID.getControl(Robot_Pos, dt);
  accel = constrain(accel, -MAX_ACCEL, MAX_ACCEL);
  wheel_velocity += accel * dt;
  wheel_velocity = constrain(wheel_velocity, -MAX_VEL, MAX_VEL);
  
  if (rStateData.Right)
    targetSteering = -0.1*rStateData.velocity;
  else if (rStateData.Left)
    targetSteering = 0.1*rStateData.velocity;
  else
    targetSteering = 0;

  steering = mpuGetAngleYaw();
  steeringPID.setTarget(targetSteering);
  steeringVel = steeringPID.getControl(steering, dt);
 
  leftVelocity = wheel_velocity - steeringVel;
  rightVelocity = wheel_velocity + steeringVel;
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
    Serial.begin(9600);
    MPUsetup();
    SteppersSetup();
    enableSteppers();
    setTimers();
    //CONTROL!
    Serial.println("Inverted Pendulum");
    Serial.print("Position_Kp:");
    Serial.print(Position_Kp);
    Serial.print(" ");
    Serial.print("Position_Kd:");
    Serial.print(Position_Kd);
    Serial.print(" ");
    Serial.print("Position_Ki:");
    Serial.print(Position_Ki);
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
       
     if(Serial.available()) {
      // get the new byte:
      char inCommand = (char)Serial.read();
      interpretCommand(inCommand, rStateData); //read and interpret command
    }    
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
