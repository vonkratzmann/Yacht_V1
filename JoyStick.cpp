#include "Arduino.h"
#include "JoyStick.h"

/** JoyStick constructor
    initialses current x and y positions to the centre of the joystick

*/
/* Diagnostics for joystick 
 *
 * If set to 1 prints out x and y position on the serial monitor for the joystick
 * Normally set to zero
 */
#define DEBUGJOYSTICK 1

JoyStick::JoyStick ()
{
  x_Cur = y_Cur = 511;                       //10 bit ADC, forwards 0 to 511,backwards 512 to 1023
  x_New = y_New = 511;                       //set joy stick to center position ie stopped
}
//end of JoyStick::JoyStick

/** JoyStick check x position


*/
boolean JoyStick::check_X_Pos (void)            //check joystick for any changes
{
  boolean x_Chnged;                             //flag joy stick position has changed
  x_Chnged = false;                             //reset flag
  x_New = analogRead(rudder_JoystickAnalogPin); //read joystick x position and put into x_New
  x_New &= noise_Mask;                          //zero bottom bits to prevent unnecessary calls in case of noise on ADC input
  if ( x_Cur != x_New)                          //Check if changed from last read
  {
    x_Chnged = true;                            //yes, set flag to say it has changed
    // print out x values if in debug
#ifdef DEBUGJOYSTICK
    Serial.print("jsxcur: ");
    Serial.println(x_Cur);
    Serial.print("jsxnew: ");
    Serial.println(x_New);
#endif
    diff = x_New - x_Cur;
    if (abs(diff) > JoyStick_Max_ROC)                    //check if difference greater then max rate of change (ROC)
    { //limit max acceleration to max rate of change
      if (diff > 0)                             //check if reading is increasing or decreasing
        x_Cur += JoyStick_Max_ROC;                       //reading has gone up,so limit acceleration by adding max rate of change
      else
        x_Cur -= JoyStick_Max_ROC;                       //reading has gone down,so limit acceleration by subtracting max rate of change
    }
    else
      x_Cur = x_New;                            // change less than max rate of change, so accept new value
  }
  return x_Chnged;
}
//end of JoyStick::chk_X_Pos()

/** JoyStick check y position


*/
boolean JoyStick::check_Y_Pos (void)            //check joystick for any changes
{
  boolean y_Chnged;                             //flag joy stick position has changed
  y_Chnged = false;                             //reset flag
  y_New = analogRead(mainsail_JoystickAnalogPin);       //read joystick y position and put into y_new
  y_New &= noise_Mask;                          //zero bottom bits to prevent unnecessary calls in case of noise on ADC input
  if ( y_Cur != y_New)                          //Check if changed from last read
  {
    // print out y values if in debug
#ifdef DEBUGJOYSTICK
    Serial.print("jsycur: ");
    Serial.println(y_Cur);
    Serial.print("jsynew: ");
    Serial.println(y_New);
 
#endif
    diff = y_New - y_Cur;
    if (abs(diff) > JoyStick_Max_ROC)                    //check if difference greater then max rate of change (ROC)
    { //limit max acceleration to max rate of change
      if (diff > 0)                             //check if reading is increasing or decreasing
        y_Cur += JoyStick_Max_ROC;                       //reading has gone up,so limit acceleration by adding max rate of change
      else
        y_Cur -= JoyStick_Max_ROC;                       //reading has gone down,so limit acceleration by subtracting max rate of change
    }
    else
      y_Cur = y_New;                           // change less than max rate of change, so accept new value
  }
  return y_Chnged;
}
//end of JoyStick::check_Y_Pos()


void JoyStick::process_X(uint8_t &spd, uint8_t &dir)         // process change for x axis of joystick
{
  /*
  if (x_Cur <= stopped_High && x_Cur >= stopped_Low)         //check if in the stopped range
  {
    spd = 0;                                  //yes, stopped so update speed of motor to say stopped
    dir = forWard;                            //set default direction as forward
  }
  else                                        //no, requesting movement
  {
    if (x_Cur < stopped_Low)                  //is joystick asking to move forwards
    { //yes
      dir = forWard;
      spd = map(x_Cur, stopped_Low - 1, 0, minCountSpeed, maxCountSpeed); //Scale joystick position to speed count for stepper
    }
    else                                      //no, request to move backward
    {
      dir = backWard;
      spd = map(x_Cur, stopped_High + 1, 1023, minCountSpeed, maxCountSpeed); //Scale joystick position to speed count for stepper
    }
  }
*/
  return;
}
//end of JoyStick::process_X()

boolean JoyStick::process_Y(uint8_t cur_X_Spd, uint8_t &left_Spd, uint8_t &right_Spd)     // process change for y axis of joystick
{
  /*
  turning_Flag = false;
  left_Spd     = cur_X_Spd;                              //initialise speeds, set left and righ to same, ie moving straight,not turning
  right_Spd    = cur_X_Spd;
  if (y_Cur <= stopped_High && y_Cur >= stopped_Low)          //joystcick turning left or right?
  {
  }                                                           //no do nothing, leave speed unchanged
  else                                                        //request to turn
  {
    turning_Flag = true;
    if (y_Cur < stopped_Low)                                  //is joystick asking to move left?
    { //yes, slow left wheels & leave right wheels unchanged
      spd = map(y_Cur, 0, stopped_Low - 1, cur_X_Spd, 0);     //get amount to reduce speed, use y joystick & current x speed

      left_Spd += spd;                                      //slow left wheel speed, by adding 'spd', as a higher count is a lower speed
    }
    else                                                       //no, must be request to move right
    { //slow right wheels & leave left wheels unchanged
      spd = map(y_Cur, stopped_High + 1, 1023, 0, cur_X_Spd);  //get amount to reduce speed, use y joystick & current x speed
      right_Spd += spd;                                     //slow left wheel speed, by adding 'spd', as a higher count is a lower speed
    }
  }

  return turning_Flag;
  */
 return;
}
//end of JoyStick::process_Y()

