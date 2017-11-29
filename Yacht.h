/** Yacht parameters
 * 
 *
 *
*/

#ifndef Yacht_h
#define Yacht_h



#include "arduino.h"

/* Set up speed range for motors */
#define MINSPEED 0
#define MAXSPEED 512

/* set up directions for motors */

#define FORWARD		1
#define REVERSE		0
#define TOPORT		1
#define TOSTARBOARD	0
#define TIGHTENING	0
#define LOOSENING 	1

const long Debounce = 100;                    //debounce time for switch in millisecs

/** motors
   define i/o for each motor driver board, each board has 2 inputs: direction & pwm
*/
const uint8_t  rudder_Dir_Pin	= 8;      //sets direction rudder motor turns
const uint8_t  rudder_Pwm_Pin	= 9;      //PWM pulse to set the speed of the rudder motor
const uint8_t  boom_Dir_Pin		= 7;          //sets the direction the boom motor turns
const uint8_t  boom_Pwm_Pin		= 6;          //PWM pulse to set the speed of the boom motor

/** end of travel detectors
   define i/O for reed switches to detect end of travel for the chain on each motor
*/
const uint8_t  rudder_Port_EndofTravel_Pin		= 2;
const uint8_t  rudder_Starboard_EndofTravel_Pin	= 3;
const uint8_t  boom_Tight_EndofTravel_Pin		= 4;
const uint8_t  boom_Loose_EndofTravel_Pin		= 5;

/* define i/O for led */
const uint8_t LedPin =  13; //LED connected to digital pin 13

#endif
