
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
  stateDirection = 0;
  stateServo = 0;
  stateBattery = 0;
  upDownAngle = 0;
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
      doDriveStraight(TRUE)

      //timer to run isCollision function every 20 iterations
      static int timer = 0;
      if (timer >= 20){
        // state transition logic
        if (isCollision()==TRUE){
          state = 1; // next state
          // output actions
        }
        timer = 0;
      }
      else {
        timer++;
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

//===================================================================
// State machine for detecting if light is to the 
// right or left, and steering the robot accordingly.
int fsmDriveRobot()
{
  // state machine for driving robot
  static int stateDirection=0;
  switch (stateDirection)
  {
    case 0:
      // in-state actions
      doStopRobot(TRUE);

      // state transition logic
      if (isLightUp()==TRUE) {
        stateDirection = 1; // next state
        // output actions  
        //doLeaveServo(FALSE);
      }
      if (isLightDown()==TRUE) {
        stateDirection = 2;  // next state
        // output actions  
        //doLeaveServo(FALSE);
      }
      if (isLightRight()==TRUE){
        stateDirection = 3;   // next state
        // output actions  
        doDriveStraight(FALSE);
      }
      if (isLightLeft()==TRUE){
        stateDirection = 4;  // next state
        // output actions  
        doDriveStraight(FALSE); 
      }
      
      break;

    case 1:
      // in-state actions
      doMoveServoUp(TRUE);

      // state transition logic
      if (isLightUp()==FALSE) {
        stateDirection = 0; // next state
        // output actions  
        doMoveServoUp(FALSE);  
      }
      break;
    case 2:
      // in-state actions
      doMoveServoDown(TRUE);

      // state transition logic
      if (isLightDown()==FALSE) {
        stateDirection = 0; // next state
        // output actions  
        doMoveServoDown(FALSE);  
      }
      break;
    case 3:
      // in-state actions
      doCurveRight(TRUE);

      // state transition logic
      if (isLightRight()==FALSE){
        stateDirection = 0; // next state
        // output actions  
        doCurveRight(FALSE);  
      }
      break;
    case 4:
      // in-state actions
      doCurveLeft(TRUE);

      // state transition logic
      if (isLightLeft()==FALSE){
        stateDirection = 0; // next state
        // output actions  
        doCurveLeft(FALSE); 
      }
      break;
  }
  return(state);
}  

int fsmBatteryMeter(){
  switch(stateBattery){ //batter voltage monitor FSM
    case 0: //fully charges
      //state actions
      doFullBattery();
      
      //transition logic
      if (isMediumBattery()){
        stateBattery = 1;
      }
      break;

    case 1: //medium charge
      //state actions
      doMediumBattery();

      //transition logic
      if(isLowBattery()){
        stateBattery = 2;
      };
      break;
      
     case 2: //low charge
      //state actions
      doLowBattery();

      //transition logic
      if(isEmptyBattery()){
        stateBattery = 3;
      };
      break;
      
     case 3: //empty
      //state actions
      doEmptyBattery();

      break;
  }
  return (stateBattery);
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
  float input_voltage = (float) analogRead(BUTTON_CENTER);
  //Serial.println(input_voltage);
  if(input_voltage >= 100){
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

