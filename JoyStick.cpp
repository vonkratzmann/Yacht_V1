#include "Yacht.h"
#include "JoyStick.h"

/* Diagnostics for joystick
   If set to 1 prints out x and y position on the serial monitor for the joystick
   Normally set to zero
*/
#define DEBUGJOYSTICK 0

/* JoyStick constructor
     initialses current x and y positions to the centre of the joystick
*/
JoyStick::JoyStick ()
{
  x_Cur = y_Cur = 511;                       //10 bit ADC, forwards 0 to 511,backwards 512 to 1023
  x_New = y_New = 511;                       //set joy stick to center position ie stopped
}

/* JoyStick check x position
    reads joystick, compares current and new
    masks bottom bits to prevent unwanted noise and doing unnecessary processing
    limits rate of change
    returns true if position has changed
*/
bool JoyStick::check_X_Pos (void)               //check joystick for any changes
{
  bool x_Chnged = false;                        //flag joy stick position has changed
  x_New = analogRead(rudder_JoystickAnalogPin); //read joystick x position and put into x_New
  x_New &= noise_Mask;                          //zero bottom bits to prevent unnecessary calls in case of noise on ADC input
  if ( x_Cur != x_New)                          //Check if changed from last read
  {
    x_Chnged = true;                            //yes, set flag to say it has changed
    // print out x values if in debug
#ifdef DEBUGJOYSTICK
    Serial.println("JoyStick::check_X_Pos");
    Serial.print("jsxcur: ");
    Serial.println(x_Cur);
    Serial.print("jsxnew: ");
    Serial.println(x_New);
#endif
    diff = x_New - x_Cur;
    if (abs(diff) > JoyStick_Max_ROC)         //check if difference greater then max rate of change (ROC)
    { //limit max acceleration to max rate of change
      if (diff > 0)                           //check if reading is increasing or decreasing
        x_Cur += JoyStick_Max_ROC;            //reading has gone up,so limit acceleration by adding max rate of change
      else
        x_Cur -= JoyStick_Max_ROC;            //reading has gone down,so limit acceleration by subtracting max rate of change
    }
    else
    {
      x_Cur = x_New;                          // change less than max rate of change, so accept new value
    }
#ifdef DEBUGJOYSTICK
    Serial.print("updated jsxcur: ");
    Serial.println(x_Cur);
#endif
  }
  return x_Chnged;
}

/* JoyStick check y position
   reads joystick, compares current and new
   masks bottom bits to prevent unwanted noise and doing unnecessary processing
   limits rate of change
   returns true if position has changed
*/
bool JoyStick::check_Y_Pos (void)               //check joystick for any changes
{
  bool y_Chnged = false;                        //flag joy stick position has changed
  y_New = analogRead(boom_JoystickAnalogPin);//read joystick y position and put into y_new
  y_New &= noise_Mask;                          //zero bottom bits to prevent unnecessary calls in case of noise on ADC input
  if ( y_Cur != y_New)                          //Check if changed from last read
  {
    // print out y values if in debug
#ifdef DEBUGJOYSTICK
    Serial.println("JoyStick::check_Y_Pos");
    Serial.print("jsycur: ");
    Serial.println(y_Cur);
    Serial.print("jsynew: ");
    Serial.println(y_New);
#endif
    diff = y_New - y_Cur;
    if (abs(diff) > JoyStick_Max_ROC)       //check if difference greater then max rate of change (ROC)
    { //limit max acceleration to max rate of change
      if (diff > 0)                         //check if reading is increasing or decreasing
        y_Cur += JoyStick_Max_ROC;          //reading has gone up,so limit acceleration by adding max rate of change
      else
        y_Cur -= JoyStick_Max_ROC;          //reading has gone down,so limit acceleration by subtracting max rate of change
    }
    else
    {
      y_Cur = y_New;                        // change less than max rate of change, so accept new value
    }
#ifdef DEBUGJOYSTICK
    Serial.print("updated jsycur: ");
    Serial.println(y_Cur);
#endif
  }
  return y_Chnged;
}

/* JoyStick::process_X
   accepts two pointer arguments to allow update of new speed and direction
   checks if its in stopped range, if yes sets speed to zero
   else checks requested direction and updates direction
   then scales the new speed bewteen the min and max speeds based on joystick position
*/
void JoyStick::process_X(int *new_Spd, uint8_t *new_Dir)    //process change for x axis of joystick
{
  if (x_Cur <= Stopped_High && x_Cur >= Stopped_Low)            //check if in the stopped range
  {
    *new_Spd = 0;                                               //yes, stopped so update speed to say stopped
#ifdef DEBUGJOYSTICK
    Serial.println("JoyStick::process X (stopped)");
    Serial.print("new_Spd: ");
    Serial.println(*new_Spd);
#endif
  }
  else                                                          //no, joystick requesting movement
  {
    if (x_Cur < Stopped_Low)                                    //is joystick asking to move to starboard
    { 
      *new_Dir = TOSTARBOARD;                                   //yes, moving to starboard
      *new_Spd = map(x_Cur, Stopped_Low - 1, 0, MINSPEED, MAXSPEED); //Scale joystick position to speed range for motor
#ifdef DEBUGJOYSTICK
      Serial.println("JoyStick::process X (low)");
      Serial.print("new_Spd: ");
      Serial.println(*new_Spd);
      Serial.print("new_Dir: ");
      Serial.println(*new_Dir);
#endif
    }
    else                                                        //no, request to move to port
    {
      *new_Dir = TOPORT;
      *new_Spd = map(x_Cur, Stopped_High + 1, 1023, MINSPEED, MAXSPEED); //Scale joystick position to speed range for motor
#ifdef DEBUGJOYSTICK
      Serial.println("JoyStick::process X (high)");
      Serial.print("new_Spd: ");
      Serial.println(*new_Spd);
      Serial.print("new_Dir: ");
      Serial.println(*new_Dir);
#endif
    }
  }
}
/* JoyStick::process_Y
   accepts two pointer arguments to allow update of new speed and direction
   checks if its in stopped range, if yes sets speed to zero
   else checks requested direction and updates direction
   then scales the new speed bewteen the min and max speeds based on joystick position
*/
void JoyStick::process_Y(int *new_Spd, uint8_t *new_Dir)    //process change for Y axis of joystick
{
  if (y_Cur <= Stopped_High && y_Cur >= Stopped_Low)            //check if in the stopped range
    *new_Spd = 0;                                                    //yes, stopped so update speed to say stopped
  else                                                          //no, joystick requesting movement
  {
    if (y_Cur < Stopped_Low)                                    //is joystick asking to tighten the ripe to the boom
    { //yes
      *new_Dir = TIGHTENING;                                    //yes, tightening rope to boom
      *new_Spd = map(y_Cur, Stopped_Low - 1, 0, MINSPEED, MAXSPEED); //Scale joystick position to speed range for motor
    }
    else                                                        //no, request to move to loosen the rope to the boom
    {
      *new_Dir = LOOSENING;
      *new_Spd = map(y_Cur, Stopped_High + 1, 1023, MINSPEED, MAXSPEED); //Scale joystick position to speed range for motor
    }
  }
}
