/** Class joyStick


*/
#ifndef JoyStick_h
#define JoyStick_h

#include "Arduino.h"

/* ADC I/O for Joystick*/
const uint8_t rudder_JoystickAnalogPin 		= 1;    //x axis of joystick
const uint8_t mainsail_JoystickAnalogPin 	= 0;    //y xis of joystick


/* joystick Parameters */
const unsigned long JoyStick_Scan_Rate  = 100;      //scan every 100 ms or 1/10 of a second, (Arbirtary choice)
const unsigned int  JoyStick_Max_ROC    = 511/10;   //limit rate of change allowable by the joystick to 10% (Arbirtary choice)
const unsigned int  noise_Mask        = 0xFFF0;     //clear bottom bits to mask any noise on signal

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
    boolean x_Chnged, y_Chnged;      //flag joy stick position has changed
    
  public:
    JoyStick ();
    boolean    check_X_Pos (void);
    boolean    check_Y_Pos (void);
    void       process_X (uint8_t &, uint8_t &);
    boolean    process_Y (uint8_t, uint8_t &, uint8_t &);
};
#endif
