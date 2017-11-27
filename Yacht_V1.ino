/** Drives a yacht controller for Brad Connelly
   Author:    Kirk Kratzmann
   Version:   1.0
   Date:      26/11/2017

   History:
*/

/** System Summary

   System consists of:
   Two 12v DC motors:
    Rudder Motor which drives the rudder
    boom Motor which tightens or loosens the boom
   A joystick connected to two 10 bit ADCS, forwards 0 to 511,backwards 512 to 1023
   On/Off switch
   12V DC battery
   Microprocessor
   Driver Printed Circuit Boards for the two DC motors
   5v DC power supply for the microprocessor
   Each motor via a sprocket drives a loop to drive the rudder or boom
   The rudder loop consists of chain and stainless steel wire
   The boom loop consists of a chain and shock cord which via a series of fixed pulleys and cables tighten or release the boom
   The travel of the loop is limited in both directions so only the chain is on the sprocket at all times
   To limit the travel of the loop, each loop has two reed switches to detect the end of the chain
   Connected to the end of the chain is a magnet to activate the reed switch

   The program reads the joystick at a preddefined rate
   then converts these readings into a Pulse width Modulated output and direction to drive the rudder and boom motors direction and speed
   on joystick x is the rudder, y is the boom

   Normal operation the diagnostic led fashes on and off every second

   h/w diagnostics are started if PCM 1 input is grounded
   the diagnostics can be run and stopped at any time while the program is running
   while running diagnostics all normal motor operation stops
*/

/** program classes

   classes              Indentifiers            Comments

    joystick               js                    reads x & y axis, computes speed & dir, loaded into wheel structures, for ISR to process

   Other functions:

    ISR () Interrupt Service Routine             process the PWM signals for the motors
   setup()
   loop()
*/

/* ISR */
//uses counter 2 to generate a 500 micro second interrupt which pulses the stepper motors, max pulse freq is 2kHz

/** I/O
   all inputs have internal pullups enabled
   so switches are 0 when pressed, and a 1 when released
   all pins defined at the beginning of the program, rather than in the classes, to see in one spot what i/o pins are allocatted or free
*/

#include "JoyStick.h"
#include "Switch.h"

#define DEBUGSTARTUP 0

#define DEBUGISR1 0
#define DEBUGISR2 0

unsigned int interrupt_Counter = 0;         //used in main loop to show the ISR is running
const unsigned int one_Sec = 2000;          //used in main loop to show the ISR is running, flashes led off and on each second

unsigned long  joys_Time_Of_Last_Scan = 0;     //track when we last scanned for joystick changes

const long Debounce = 100;                    //debounce time for switch in millisecs

/* Rudder Parameters */

/* boom Parameters */

/* all inputs & outputs (I/O) allocated below rather than in classes to ensure no overlaps or errors */

/** motors

   define i/o for each motor driver board, each board has 2 inputs: direction & pwm
*/
const uint8_t  rudder_Dir_Pin     = 8;      //sets direction rudder motor turns
const uint8_t  rudder_Pwm_Pin     = 9;      //PWM pulse to set the speed of the rudder motor
const uint8_t  boom_Dir_Pin   = 7;          //sets the direction the boom motor turns
const uint8_t  boom_Pwm_Pin   = 6;          //PWM pulse to set the speed of the boom motor

/** end of travel detectors

   define i/O for reed switches to detect end of travel for the chain on each motor
*/
const uint8_t  rudder_Port_EndofTravel_Pin      = 2;
const uint8_t  rudder_Starboard_EndofTravel_Pin = 3;
const uint8_t  boom_Tight_EndofTravel_Pin       = 4;
const uint8_t  boom_Loose_EndofTravel_Pin       = 5;
/* define i/O for led */
const uint8_t ledPin =  13; //LED connected to digital pin 13
uint8_t led = LOW;  //initial state of led

#ifdef DEBUGISR1
unsigned long entry_Time, exit_Time;        //used to check overhead of ISR
unsigned long tmp1, tmp2;
#endif

/* define objects */

/* define motors */
JoyStick js;  //define joystick

/* define reed switches */
Switch switch_rudder_Port_EndofTravel(rudder_Port_EndofTravel_Pin, Debounce);
Switch switch_rudder_Starboard_EndofTravel(rudder_Starboard_EndofTravel_Pin, Debounce);
Switch switch_boom_Tight_EndofTravel(boom_Tight_EndofTravel_Pin, Debounce);
Switch switch_boom_Loose_EndofTravel(boom_Loose_EndofTravel_Pin, Debounce);


/* Interrupt Service Routine for timer 2

  Each stepper has a counter which determines the rate at which the stepper motor is pulsed.
  The counter is decremented, if zero the step input into the steeper motor driver is set high and the counter reloaded;
  on the next timer increment the step input is set low.

  ISR also checks if the step pulse ouput is high, if so sets it low

  Increments activity counter, for processing by main loop */

ISR(TIMER2_COMPA_vect)
{
#ifdef DEBUGISR1
  //attempt to check the overhead of the ISR, by recording times of entry into two consecutive ISR calls,
  //then main loop can then print these out
  entry_Time = micros();
#endif

  interrupt_Counter = interrupt_Counter + 1;        // used to show we are alive and ISR running
#ifdef DEBUGISR1
  exit_Time = micros();
#endif
  return;
}  //end of ISR


/** Setup


*/
void setup(void)
{
  /* set up inputs using internal pull up resistors and set up outputs */
  pinMode(ledPin,          OUTPUT);
  digitalWrite(ledPin,       HIGH);     // sets the LED on

  Serial.begin(9600);                   //set up serial port for any debug prints
#ifdef DEBUGSTARTUP
  Serial.println("started");
#endif
  /* Set up timer interrupt */

  /* Timer 0, is 8 bits used by fuction millis();  Timer 1, 3, 4, 5 are 16 bits;  Timer 0, 2 are 8 bits
    timers used by anolgue write

    Use Timer 2 to generate an interrupt every 500 microseconds to process step pulses to stepper mnotors
    for timer is 16x10^6 (clock speed) / [prescaler x freq] -1
    for 1kHz, prescaler = 64, cont = 124  */

  cli();                                 //stop interrupts
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2  = 0;
  OCR2A = 124;                          //set compare register
  TCCR2A |= (1 << WGM21);               //turn on CTC mode
  TCCR2B |= (1 << CS22);                //sets prescaler for 64
  TIMSK2 |= (1 << OCIE2A);
  sei();                                //enable interrupts

  return;
}  //  end of setup()

/** Main Loop


*/
void loop(void)
{
#ifdef DEBUGISR1
  unsigned long entry_Time, exit_Time;        //used to check overhead of ISR
  unsigned long tmp1, tmp2;
#endif
  if (interrupt_Counter >= one_Sec )  //check if one second has expired
  {
    digitalWrite(ledPin, led);    // continually turn led on for 1 sec, then off for 1 sec - shows we alive & interrupts working
    led ? led = LOW : led = HIGH; // swap led state from high to low or low to high
    cli();                        //interrupts off
    interrupt_Counter = 0;        //reset counter
    sei();                        //interrupts on
  }

  if ((millis() - joys_Time_Of_Last_Scan) > JoyStick_Scan_Rate) //check if time to scan joystick for changes to x and Y axis
  {
    joys_Time_Of_Last_Scan = millis();    //yes, reset timer
    bool flag;                         //flag to indicate joystick position has changed
    flag = js.check_Y_Pos();              //must run both functions to ensure current x & y positions are updated
    flag |= js.check_X_Pos();
    if (flag)                             //check if x or y axis changed,
    {
      //yes, do something
    }
  }

  if (switch_rudder_Port_EndofTravel.switch_Changed())
  {
    //yes, do something
  }

  if (switch_rudder_Starboard_EndofTravel.switch_Changed())
  {
    //yes, do something
  }
  if (switch_boom_Tight_EndofTravel.switch_Changed())
  {
    //yes, do something
  }
  if (switch_boom_Loose_EndofTravel.switch_Changed())
  {
    //yes, do something
  }
}
//end of loop()


/* end */

