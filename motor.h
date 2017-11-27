/** Class motor
 * 
 *
 *
*/
#ifndef Motor_h
#define Motor_h

#include "Arduino.h"

#define FORWARD 0;
#define BACKWARD 1;

/* Motor Parameters
   Combination of scan rate and maximum Rate of Change (ROC) limit speed of response of system
   Restrict 0 to max change to approximately 2 seconds, ie 0 to 512
   as scan rate is 100 millseconds,
   and max change on each joystick scan is 25
*/
const unsigned long Motor_Scan_Rate  = 100;    //scan every 100 ms or 1/10 of a second
const unsigned int  Motor_Max_ROC    = 25;     //limit rate of change allowable by the joystick (see comments above)

class Motor
{
  private:
	  uint8_t	current_speed;				//store current speed
    uint8_t requested_speed;      //store the new requested speed
	  uint8_t	current_dir;				  //store current direction
    uint8_t requested_dir;        //store requested direction
    boolean stopped;              //flag to say if motor stopped or moving. true is stopped, false indicates moving

	  uint8_t pwm_Pin;			        //Ouput pin to pulse to drive the motor using PWM
	  uint8_t dir_Pin;			        //output pin to set direction to the motor
 
  public:
	  Motor(uint8_t, uint8_t);	    //constructor, set speed pin and direction pin
	  void	  set_Requested_Speed(uint8_t);
	  uint8_t	get_Requested_Speed(void);
	  void	  set_Requested_Dir(uint8_t);
	  uint8_t	get_Requested_Dir(void);
    void    update_Speed(void);
};

#endif
