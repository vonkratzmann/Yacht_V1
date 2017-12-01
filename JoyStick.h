/** Class joyStick

*/
#ifndef JoyStick_h
#define JoyStick_h

#include "Arduino.h"

/* joystick Parameters */

/* Regard joystick in stopped position if equal to or between Joys_Stopped_High & Joys_Stopped_Low
     Note difference between Stopped_High and Stopped_low has to be greater than JoyStick_Max_ROC
     as desirable that max change on each joystick scan less than stopped range on joystick,
     so as the joystick values change there will always be one position of the joystick where the motor is stopped
     so the change of direction will occur when the motor is stopped
*/
const int     Stopped_High = 542;      //setjoystick high range for stopped
const int     Stopped_Low  = 481;      //setjoystick low range for stopped
//stopped range is 62

// Therefore joystick range is:
// | 0-480 | 481 - 542 | 543 - 1023 |
// |  Low  |  Stopped  |    High    |

/*
   Combination of scan rate and maximum Rate of Change (ROC) limit speed of response of system
   Restrict 0 to max change to approximately 2 seconds, ie 0 to 489 or 543 to 1023, range of 490
   as scan rate is 100 millseconds, that is 20 scans in 2 seconds
   therefore max change on each joystick scan is 24, ie 24 X 20 = 480 (approximately 490)
   from max speed in one direction to maximum speed in other direction will take 4 seconds
*/
const unsigned long JoyStick_Scan_Rate    = 100;   //scan every 100 ms or 1/10 of a second, (see comments above)
const int  JoyStick_Max_ROC              = 24;     //limit rate of change allowable by the joystick (see comments above)
const int  noise_Mask                    = 0xFFF0; //clear bottom bits to mask any noise on signal

class JoyStick
{
  private:
    int     x_Cur, y_Cur;            //current position of joystick
    int     x_New, y_New;            //new position of joystick
    int     diff;
    bool x_Chnged, y_Chnged;         //flag joy stick position has changed

  public:
    JoyStick ();
    bool    check_X_Axis (void);  //check if change in joystick x position
    bool    check_Y_Axis (void);  //check if change in joystick y position
    void    process_X(int *spd, int *dir); // process change for x axis of joystick
    void    process_Y(int *spd, int *dir); // process change for y axis of joystick
};
#endif
