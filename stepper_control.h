#ifndef STEPPER_CONTROL_H
#define STEPPER_CONTROL_H

const unsigned long ULONG_MAX = 0UL - 1UL;

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

#define PPR   1600 //Pulses per revolution or steps per revolution
// #define TICKS_PER_SECOND  40000 // 40kHz
#define TICKS_PER_SECOND  50000 // 50kHz
#define PULSE_WIDTH 1

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
#if TICKS_PER_SECOND == 40000
  setTimer1(49); // 40kHz
#elif TICKS_PER_SECOND == 50000
  setTimer1(39); // 50kHz
#endif
  sei();
}
  
unsigned long getTicksPerPulse(float velocity) {
  if (abs(velocity) < 1e-3) {
    // TODO: disable motor
    disableSteppers();
    return ULONG_MAX;
  } else {
    enableSteppers();
    return (uint64_t)(2.0 * PI * TICKS_PER_SECOND / (abs(velocity) * PPR)) - PULSE_WIDTH;
  }
}

#endif
