#include "Arduino.h"
#include "JoyStick.h"

/* Diagnostics for joystick
 *
 * If set to 1 prints out x and y position on the serial monitor for the joystick
 * Normally set to zero
 * 
 */
#define DEBUGJOYSTICK 0

/** JoyStick constructor
 *
 *   initialses current x and y positions to the centre of the joystick
 *
 */
JoyStick::JoyStick ()
{
  x_Cur = y_Cur = 511;                       //10 bit ADC, forwards 0 to 511,backwards 512 to 1023
  x_New = y_New = 511;                       //set joy stick to center position ie stopped
}

/** JoyStick check x position
 *
 *  reads joystick, compares current and new
 *  masks bottom bits to prevent unwanted noise and doing unnecessary processing
 *  limits change
 *  returns true if position has changed
 *
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
    if (abs(diff) > JoyStick_Max_ROC)         //check if difference greater then max rate of change (ROC)
    {                                         //limit max acceleration to max rate of change
      if (diff > 0)                           //check if reading is increasing or decreasing
        x_Cur += JoyStick_Max_ROC;            //reading has gone up,so limit acceleration by adding max rate of change
      else
        x_Cur -= JoyStick_Max_ROC;            //reading has gone down,so limit acceleration by subtracting max rate of change
    }
    else
      x_Cur = x_New;                           // change less than max rate of change, so accept new value
  }
  return x_Chnged;
}
//end of JoyStick::chk_X_Pos()

/** JoyStick check y position
 *
 * reads joystick, compares current and new
 * masks bottom bits to prevent unwanted noise and doing unnecessary processing
 * limits change
 * returns true if position has changed
 *
*/
boolean JoyStick::check_Y_Pos (void)            //check joystick for any changes
{
  boolean y_Chnged;                             //flag joy stick position has changed
  y_Chnged = false;                             //reset flag
  y_New = analogRead(boom_JoystickAnalogPin);//read joystick y position and put into y_new
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
    if (abs(diff) > JoyStick_Max_ROC)       //check if difference greater then max rate of change (ROC)
    {                                       //limit max acceleration to max rate of change
      if (diff > 0)                         //check if reading is increasing or decreasing
        y_Cur += JoyStick_Max_ROC;          //reading has gone up,so limit acceleration by adding max rate of change
      else
        y_Cur -= JoyStick_Max_ROC;          //reading has gone down,so limit acceleration by subtracting max rate of change
    }
    else
      y_Cur = y_New;                        // change less than max rate of change, so accept new value
  }
  return y_Chnged;
}


