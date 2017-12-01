/* Yacht parameters */

#ifndef Yacht_h
#define Yacht_h

#include "arduino.h"

/* define to run diagnostics which print to the serial monitor
  comment out before code is released
*/
//#define  DEBUG

#ifdef   DEBUG
#define  DEBUG_PRINT(x)    Serial.print(x)
#define  DEBUG_PRINTLN(x)  Serial.println(x)
#define  DEBUG_FILE(x)
#else
#define  DEBUG_PRINT(x)
#define  DEBUG_PRINTLN(x)
#define  DEBUG_FILE(x)
#endif

/* define to run ISR diagnostics which attempt to determine the overhead of the ISR
 * and print out te results in the main loop
  comment out before code is released
*/
//#define  ISR_DEBUG

#ifdef   ISR_DEBUG
#define  ISR_DEBUG_ENTRY      entry_Time=micros();
#define  ISR_DEBUG_EXIT       exit_Time=micros();
#define  ISR_DEBUG_PRINT(x)   Serial.print(x)
#define  ISR_DEBUG_PRINTLN(x) Serial.println(x)
#else
#define  ISR_DEBUG_ENTRY   
#define  ISR_DEBUG_EXIT 
#define  ISR_DEBUG_PRINT(x)
#define  ISR_DEBUG_PRINTLN(x) Serial.println(x)
#endif

/* define to run joystick diagnostics which which print to the serial monitor
  comment out before code is released
*/
//#define  JOYSTICK_DEBUG

#ifdef   JOYSTICK_DEBUG
#define  JOYSTICK_DEBUG_PRINT(x)    Serial.print(x)
#define  JOYSTICK_DEBUG_PRINTLN(x)  Serial.println(x)
#define  JOYSTICK_DEBUG_FILE(x)
#else
#define  JOYSTICK_DEBUG_PRINT(x)
#define  JOYSTICK_DEBUG_PRINTLN(x)
#define  JOYSTICK_DEBUG_FILE(x)
#endif

/* define to run joystick diagnostics which force a value from the joystick
  comment out before code is released
*/
//#define  JOYSTICK_FORCEDEBUG
 
#ifdef   JOYSTICK_FORCEDEBUG
#define  JOYSTICK_DEBUG_Y_EQUALS_256     y_New = 256;
#define  JOYSTICK_DEBUG_X_EQUALS_256     x_New = 256;
#else
#define  JOYSTICK_DEBUG_Y_EQUALS_256
#define  JOYSTICK_DEBUG_X_EQUALS_256
#endif

/* define to run motor diagnostics which print to the serial monitor
  comment out before code is released
*/
#define  MOTOR_DEBUG
 
#ifdef   MOTOR_DEBUG
#define  MOTOR_DEBUG_PRINT(x)    Serial.print(x)
#define  MOTOR_DEBUG_PRINTLN(x)  Serial.println(x)
#define  MOTOR_DEBUG_FILE(x)
#else
#define  MOTOR_DEBUG_PRINT(x)
#define  MOTOR_DEBUG_PRINTLN(x)
#define  MOTOR_DEBUG_FILE(x)
#endif


/* define to run switch diagnostics which print to the serial monitor
  comment out before code is released
*/
#define  SWITCH_DEBUG

#ifdef   SWITCH_DEBUG
#define  SWITCH_DEBUG_PRINT(x)    Serial.print(x)
#define  SWITCH_DEBUG_PRINTLN(x)  Serial.println(x)
#define  SWITCH_DEBUG_FILE(x)
#else
#define  SWITCH_DEBUG_PRINT(x)
#define  SWITCH_DEBUG_PRINTLN(x)
#define  SWITCH_DEBUG_FILE(x)
#endif


/* set up directions for motors */
#define FORWARD     1
#define REVERSE     0
#define TOPORT      1
#define TOSTARBOARD 0
#define LOOSENING   1
#define TIGHTENING  0

const int One_Sec = 1960;            //used in main loop to show the ISR is running, flip flops led off and on each second
const int Qtr_Sec =  490;            //used in main loop to flash led show for a quarter of a second

/* Set up speed range for joystick */
const int MINSPEED = 0;
const int MAXSPEED = 511;                     //as joystick reads 0 to 1023

/* Set up speed range for motor*/
const int MOTOR_MINSPEED = 0;
const int MOTOR_MAXSPEED = 511;               //may not want to let the motor get to maximum speed

const long Debounce = 100;                    //debounce time for switch in millisecs

/* ADC I/O for Joystick*/
const uint8_t rudder_JoystickAnalogPin 		= 1;    //x axis of joystick
const uint8_t boom_JoystickAnalogPin 	        = 0;    //y xis of joystick

/** motors
   define i/o for each motor driver board, each board has 2 inputs: direction & pwm
*/
const uint8_t  rudder_Dir_Pin	  = 8;          //sets direction rudder motor turns
const uint8_t  rudder_Pwm_Pin	  = 11;         //PWM pulse to set the speed of the rudder motor, this is ATmega PB3 OC2A, UNO pin 11
const int      rudder_Pwm_Reg     = 0xB3;       //Atmega register to set the duty cycle of the PWM, write 0 to 255
              
const uint8_t  boom_Dir_Pin       = 7;          //sets the direction the boom motor turns
const uint8_t  boom_Pwm_Pin	  = 3;          //PWM pulse to set the speed of the boom motor, this is ATmega PD3 OC2B, UNO pin 3
const int      boom_Pwm_Reg       =0xB4;        //Atmega register to set the duty cycle of the PWM, write 0 to 255
             
/** end of travel detectors
   define i/O for reed switches to detect end of travel for the chain on each motor
*/
const uint8_t  rudder_Port_EndofTravel_Pin      = 2;
const uint8_t  rudder_Starboard_EndofTravel_Pin	= 4;
const uint8_t  boom_Loose_EndofTravel_Pin       = 5;
const uint8_t  boom_Tight_EndofTravel_Pin       = 6;

/* define i/O for led */
const uint8_t LedPin =  13; //LED connected to digital pin 13
#endif
