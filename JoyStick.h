/** Class joyStick

*/
#ifndef JoyStick_h
#define JoyStick_h

#include "Arduino.h"

/* ADC I/O for Joystick*/
const uint8_t rudder_JoystickAnalogPin 		= 1;    //x axis of joystick
const uint8_t boom_JoystickAnalogPin 	= 0;    //y xis of joystick


/* joystick Parameters
   Combination of scan rate and maximum Rate of Change (ROC) limit speed of response of system
   Restrict 0 to max change to approximately 2 seconds, ie 0 to 512
   as scan rate is 100 millseconds,
   and max change on each joystick scan is 25
*/
const unsigned long JoyStick_Scan_Rate  = 100;    //scan every 100 ms or 1/10 of a second, (see comments above)
const unsigned int  JoyStick_Max_ROC    = 25;     //limit rate of change allowable by the joystick (see comments above)
const unsigned int  noise_Mask          = 0xFFF0; //clear bottom bits to mask any noise on signal

/* regard joystick in stopped position if equal to or between Joys_Stopped_High & Joys_Stopped_Low */
const int     Joys_Stopped_High = 523;      //setjoystick high range for stopped
const int     Joys_Stopped_Low  = 500;      //setjoystick low range for stopped
const int     forWard = 1;                  //set initial direction to forward
const int     backWard = 0;

class JoyStick
{
  private:
    int     x_Cur, y_Cur;            //current position of joystick
    int     x_New, y_New;            //new position of joystick
    int     diff;
    bool x_Chnged, y_Chnged;        //flag joy stick position has changed

  public:
    JoyStick ();
    bool    check_X_Pos (void);  //check if change in joystick x position
    bool    check_Y_Pos (void);  //check if change in joystick y position
};
#endif
