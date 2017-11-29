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
   as scan rate is 100 millseconds, that is 20 scans in 2 seconds
   therefore max change on each joystick scan is 26, ie 26 X 20
   from max speed in one direction to maximum speed in other direction will take 4 seconds

   desirable that max change on each joystick scan greater than stopped range on joystick,
   so as the joystick values change there will always be one position of the joystick where the motor is stopped
   so the chnge of direction will occur when the motor is stopped
*/
const unsigned long JoyStick_Scan_Rate  = 100;    //scan every 100 ms or 1/10 of a second, (see comments above)
const int  JoyStick_Max_ROC    = 26;     //limit rate of change allowable by the joystick (see comments above)
const int  noise_Mask          = 0xFFF0; //clear bottom bits to mask any noise on signal

/* regard joystick in stopped position if equal to or between Joys_Stopped_High & Joys_Stopped_Low
    NOte difference between Sstopped_High and Stopped_low has to be greater than JoyStick_Max_ROC
    see abovecomment
*/
const int     Stopped_High = 532;      //setjoystick high range for stopped
const int     Stopped_Low  = 491;      //setjoystick low range for stopped

class JoyStick
{
  private:
    int     x_Cur, y_Cur;            //current position of joystick
    int     x_New, y_New;            //new position of joystick
    int     diff;
    bool x_Chnged, y_Chnged;                  //flag joy stick position has changed

  public:
    JoyStick ();
    bool    check_X_Axis (void);  //check if change in joystick x position
    bool    check_Y_Axis (void);  //check if change in joystick y position
    void    process_X(int *spd, uint8_t *dir); // process change for x axis of joystick
    void    process_Y(int *spd, uint8_t *dir); // process change for y axis of joystick
};
#endif
