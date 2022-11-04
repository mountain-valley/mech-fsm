
#include "Arduino.h"

#define TOP_SPEED 200  // max motor speed in pwm

int state;
int stateDirection;

//for H-bridge
#define MOTOR_R_EN 9   // H-bridge enable pin:  needs to be a pwm pin
#define MOTOR_R_A 8  // H-bridge input pin A (HIGH or LOW)
#define MOTOR_R_B 7 // H-bridge input pin B (HIGH or LOW)
#define MOTOR_L_A 5  // H-bridge input pin A (HIGH or LOW)
#define MOTOR_L_B 4 // H-bridge input pin B (HIGH or LOW)
#define MOTOR_L_EN 3  // H-bridge enable pin:  needs to be a pwm pin

#define TRUE 1
#define FALSE 0

/********************************************************************
 * setup function - this gets executed at power up, or after a reset
 ********************************************************************/
void setup() {

  // set up serial connection at 9600 Baud
  Serial.begin(9600);

  //for H-bridge
  pinMode(MOTOR_R_EN, OUTPUT);  
  pinMode(MOTOR_R_A, OUTPUT);
  pinMode(MOTOR_R_B, OUTPUT);
  pinMode(MOTOR_L_EN, OUTPUT);  
  pinMode(MOTOR_L_A, OUTPUT);
  pinMode(MOTOR_L_B, OUTPUT);
  
  state = 0;

}

/********************************************************************
 * main loop - this gets executed in an infinite loop until power
 * off or reset.
 * 
 * 
 ********************************************************************/
void loop() 
{
  fsmCollision();
}

/********************************************************************
 * state machines
 ********************************************************************/

// state machine for detecting object in front of robot
int fsmCollision() 
{
  static int state=0;
  //Serial.println(state);
  switch (state)
  {
    case 0:  // no collision
      // in-state actions
      doDriveStraight(TRUE);

      //timer to run isCollision function every 20 iterations
      if (isCollision()==TRUE){
        state = 1; // next state
        // output actions
      }

      break;
      
    case 1:  // collision
      // in-state actions
      doStopRobot(TRUE);

      // state transition logic
      if (isCollision()==FALSE){ 
        state = 0; // next state
        // output actions
        doStopRobot(FALSE);
      }
      break;
  }
  return(state);
}


/********************************************************************
 * functions that test different conditions
 * 
 * 
 ********************************************************************/
//===================================================================
// Function that detects if there is an obstacle in front of robot
int isCollision()
{
  if(5 >= 100){
    return true;
    }
  else {
    return false;
  }
}

/********************************************************************
 * functions that take actions
 * 
 * 
 ********************************************************************/
//===================================================================
// used in making the wheels spin
float currentTime() {
  // t=0 is the first time function is called
  static unsigned int startTime = millis();  
  float time = (float)(millis() - startTime)/1000; 
    // divide by 1000 to convert milliseconds to seconds
  return(time);
}
 
//===================================================================
// used in making the wheels spin
void spinMotor(int speed, int en, int inA, int inB) {
  if (speed > 0){ // motor spins in + direction
    digitalWrite(inA, HIGH);
    digitalWrite(inB, LOW);
    analogWrite(en, speed);
  } 
  else if (speed < 0){  // motor spins in - direction
    digitalWrite(inA, LOW);
    digitalWrite(inB, HIGH);      
    analogWrite(en, -speed);    
  }
  else {  // motor won’t spin
    digitalWrite(inA, HIGH);
    digitalWrite(inB, HIGH);    
    analogWrite(en, 0);
  }
}
 
//===================================================================
// Function that drives the robot staight:
// Both wheels move at same speed.
void doDriveStraight(int enable)
{
  if (enable==TRUE) {
   spinMotor(TOP_SPEED, MOTOR_R_EN, MOTOR_R_A, MOTOR_R_B);
   spinMotor(TOP_SPEED, MOTOR_L_EN, MOTOR_L_A, MOTOR_L_B);
  }
  else {
  }
}

//===================================================================
// Function that causes that causes the robot to stop moving.
void doStopRobot(int enable)
{
  if (enable==TRUE) {
    spinMotor(0, MOTOR_L_EN, MOTOR_L_A, MOTOR_L_B);
    spinMotor(0, MOTOR_R_EN, MOTOR_R_A, MOTOR_R_B);
  }
  else {
    doDriveStraight(TRUE);
  }
}
