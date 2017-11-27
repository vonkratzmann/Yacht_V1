/** Class motor
 *
 *
 *
*/
#ifndef Motor_h
#define Motor_h

#include "Arduino.h"

class Motor
{
  private:
	  uint8_t	spd;				//store current speed
	  uint8_t	dir;				//store current direction

	  uint8_t pwm_Pin;			//Ouput pin to pulse to drive the motor using PWM
	  uint8_t dir_Pin;			//output pin to set direction to the motor
 
  public:
	  Motor(uint8_t, uint8_t);	//constructor, set speed pin and direction pin
	  void	set_Speed(uint8_t);
	  uint8_t	get_Speed(void);
	  void	set_Dir(uint8_t);
	  uint8_t	get_Dir(void);
};

#endif
