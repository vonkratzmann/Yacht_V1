#include "Arduino.h"
#include "Motor.h"

/* Diagnostics for motor
   If set to 1 and switch state changes prints out the motor speed
   Normally set to zero
*/
#define DEBUGMOTOR 1

/* Motor constructor
   constructor, set speed pin and direction pin
   set initial speed and direction
*/

Motor::Motor(uint8_t par_pwm_pin, uint8_t par_direction_pin)
{
  current_speed = 0;          //set initial speed
  requested_speed = 0;        //set initial speed
  requested_dir = FORWARD;    //set initial direction
  current_dir = FORWARD;      //set initial direction

  dir_Pin = par_direction_pin;            //store pin to be used to set the direction
  pwm_Pin = par_pwm_pin;                  //store pin to be used to control speed via PWM signal
  pinMode(dir_Pin, OUTPUT);               //sets the digital direction pin as output
  digitalWrite(dir_Pin, current_dir);     //sets the initial direction
}

/* set_Requested_Speed
   stores the new requested speed
   it is up to other logical routines to change the actual speed
*/
void  Motor::set_Requested_Speed(uint8_t par_new_speed)
{
  requested_speed = par_new_speed;
}

/* get_Requested_Speed
   retrurns the requested speed, not speed the motor is actually doing
*/
uint8_t Motor::get_Requested_Speed(void)
{
  return requested_speed;
}

/* set_Dir
   stores the requested direction
   it is up to other logical routines to change the actual speed and direction
*/
void  Motor::set_Requested_Dir(uint8_t par_new_direction)
{
  requested_dir = par_new_direction;
}

/* get_Dir
   returns the the requested direction of the motor
*/
uint8_t Motor::get_Requested_Dir(void)
{
  return requested_dir;
}


/* update_Speed
   updates the current speed from the requested speed
   updates are limited to the maximum rate of change to prevent abrupt stops and starts
   for when the chain hits the reed limit switches 
*/
void Motor::update_Speed()
{
  if (requested_speed != current_speed)    //Check if need to update speed
  {
    uint8_t diff;
    diff = requested_speed - current_speed;
    if (abs(diff) > Motor_Max_ROC)          //check if difference greater then max rate of change (ROC)
    { //limit max acceleration to max rate of change
      if (diff > 0)                         //check if reading is increasing or decreasing
        current_speed += Motor_Max_ROC;     //reading has gone up,so limit acceleration by adding max rate of change
      else
        current_speed -= Motor_Max_ROC;     //reading has gone down,so limit acceleration by subtracting max rate of change
    }
    else
      current_speed = requested_speed;      //change less than max rate of change, so accept new value
#ifdef DEBUGMOTOR
    // print out speed values if in debug
    Serial.print("requestedSpeed: ");
    Serial.println(requested_speed);
    Serial.print("curSpeed: ");
    Serial.println(current_speed);
#endif
  }
}

