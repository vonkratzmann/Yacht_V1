/* Yacht parameters */

#ifndef Yacht_h
#define Yacht_h

#include "arduino.h"

/* define to run diagnostics which print to the serial monitor
  comment out once code is finished

  #define  DEBUG_FILE(x)     Serial.print(x)
*/
#define  DEBUG

#ifdef   DEBUG
#define  DEBUG_PRINT(x)    Serial.print(x)
#define  DEBUG_PRINTLN(x)  Serial.println(x)
#define  DEBUG_FILE(x)     
#else
#define  DEBUG_PRINT(x)
#define  DEBUG_PRINTLN(x)
#define  DEBUG_FILE(x)     
#endif

/* define to run joystick diagnostics which force a value from the joystick
  comment out once code is finished

  #define  JOYSTICKDEBUG
*/
#ifdef   JOYSTICKDEBUG
#define  DEBUG_JOYSTICK_Y_EQUALS_256     y_New = 256;
#define  DEBUG_JOYSTICK_X_EQUALS_256     x_New = 256;
#else
#define  DEBUG_JOYSTICK_Y_EQUALS_256      
#define  DEBUG_JOYSTICK_X_EQUALS_256     
#endif

/* set up directions for motors */
#define FORWARD    1
#define REVERSE   0
#define TOPORT    1
#define TOSTARBOARD 0
#define TIGHTENING  1
#define LOOSENING   0

/* Set up speed range for motors */
const int MINSPEED = 0;
const int MAXSPEED = 511;                     //as joystick reads 0 to 1023

const long Debounce = 100;                    //debounce time for switch in millisecs

/** motors
   define i/o for each motor driver board, each board has 2 inputs: direction & pwm
*/
const uint8_t  rudder_Dir_Pin	= 8;          //sets direction rudder motor turns
const uint8_t  rudder_Pwm_Pin	= 10;         //PWM pulse to set the speed of the rudder motor
const uint8_t  boom_Dir_Pin		= 7;          //sets the direction the boom motor turns
const uint8_t  boom_Pwm_Pin		= 6;         //PWM pulse to set the speed of the boom motor

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
