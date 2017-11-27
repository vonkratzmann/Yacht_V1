/** Class motor
 *
 *
 *
*/
#ifndef Motor_h
#define Motor_h

#include "Arduino.h"

class motor
{
	uint8_t	spd;				//store current speed
	uint8_t	dir;				//store current direction

	uint8_t pwm_Pin;			//Ouput pin to pulse to drive the motor using PWM
	uint8_t dir_Pin;			//output pin to set direction to the motor
	
	boolean rudder_port_limit_switch;		//reed switches to detect end of travel of rudder chain
	boolean rudder_startboard_limit_switch;
	boolean boom_tight_limit_switch;		//reed switches to detect end of travel of boom chain
	boolean boom_loose_limit_switch;
 
public:
	motor(uint8_t, uint8_t);	//constructor, set speed pin and direction pin
	void	set_Speed(uint8_t);
	uint8_t	get_Speed(void);
	void	set_Dir(uint8_t);
	uint8_t	get_Dir(void);
}

#endif