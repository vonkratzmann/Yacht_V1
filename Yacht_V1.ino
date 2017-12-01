/** Drives a yacht controller for Brad Connelly
   Author:    Kirk Kratzmann
   Version:   1.0
   Date:      01/12/2017
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
   On the joystick x is the rudder, y is the boom

   Uses counter 2 and interrupts to generate fast mode pwm pulses, freq is 1.960kHz
   Normal operation the diagnostic led fashes on and off every second via the ISR

   All inputs have internal pullups enabled
   so switches are 0 when pressed, and a 1 when released

   There are a number of simple diagnostics which can be enable by the #DEFINEs in Yacht.h

*/

/** Software Structure Overview
   The system consists of 8 files:
    Yacht_V1.ino:  Variable declarations, oject definitions, ISR, Setup and main loop
    Yacht_V1.h:    Diagnostic definitions, I/O and constants
    JoyStick.cpp:  Joystick class member functions
    JoyStick.h:    Joystick class decelerations
    Motor.cpp:     Motor class member functions
    Motor.h:       Motor class decelerations
    Switch.cpp:    Swtch class member functions
    Switch.h:      Switch class decelerations

    The main loop logic for the x axis is:
      check if time to flash the onboard led on or off, used to show program and isr are running
	  if time to read the joystick again, then
	    calls "check_X_Axis()" to check for a change in the x axis of the joystick current reading compared to the last read
	    if there is a change, calls "process_X()" which returns an updated speed and direction via call by reference paramaters.
	      Then with the new speed and direction,
	        checks if moving towards starboard and movement to starboard is inhibited, if yes then stops the motor
            checks if moving towards port and movement to port is inhibited, if yes then stops the motor
		    calls "rudder_Motor.set_Requested_Speed(spd)"  to set new speed
            calls "rudder_Motor.set_Requested_Dir()" to set new direction

		    Then checks if moving towards starboard and moved off the limit switch, then clear the inhibit movement flag for port,
            Then checks if moving towards port and moved off the limit switch, then clear the inhibit movement flag for starboard,

	   Repeats for the y axis.
	   Then calls:
	    "rudder_Motor.update_Speed() to update rudder motor speed from the requested speed
		"rudder_Motor.update_Dir()" to update rudder motor direction from the requested direction
		"boom_Motor.update_Speed()" to update boom motor speed from the requested speed
		"boom_Motor.update_Dir()" to update boom motor direction from the requested direction
 */

#include "Yacht.h"

#include "JoyStick.h"
#include "Switch.h"
#include "Motor.h"

int interrupt_Counter = 0;           //used in main loop to show the ISR is running

unsigned long  joys_Time_Of_Last_Scan = 0;    //track when we last scanned for joystick changes

uint8_t led = LOW;                            //state of led, initially off

/* define objects */

/* define joystick */
JoyStick js;  //define joystick

/* define reed switches */
Switch switch_Rudder_Port(rudder_Port_EndofTravel_Pin, Debounce);
Switch switch_Rudder_Starboard(rudder_Starboard_EndofTravel_Pin, Debounce);
Switch switch_Boom_Tight(boom_Tight_EndofTravel_Pin, Debounce);
Switch switch_Boom_Loose(boom_Loose_EndofTravel_Pin, Debounce);

/* define motors */
Motor rudder_Motor(rudder_Pwm_Reg, rudder_Dir_Pin);
Motor boom_Motor(boom_Pwm_Reg, boom_Dir_Pin);

/* Interrupt Service Routine for when counter overflows in timer 2 */

ISR(TIMER2_OVF_vect)
{
  ISR_DEBUG_ENTRY                                   //Diagnostics
  interrupt_Counter = interrupt_Counter + 1;        //used to show we are alive and ISR running
  ISR_DEBUG_EXIT                                    //Diagnostics
  return;
}

/** Setup */
void setup(void)
{
  /* set up inputs using internal pull up resistors and set up outputs */
  pinMode(LedPin,          OUTPUT);
  digitalWrite(LedPin,       HIGH);     // sets the LED on

  Serial.begin(9600);                   //set up serial port for any debug prints

  DEBUG_PRINTLN("Started");

  /* Set up timer interrupt */

  /* Timer 0, is 8 bits used by fuction millis();
    Timer 2, is 8 bits and is used to generate an interrupt approximately every 500 microseconds and to process pwm pulses to motors
    Timer is 16x10^6 (clock speed) / [prescaler x 255];  for prescaler of 32, frequeny of interrupts and PWM is 1.960kHz
    Use fast PWM mode, where the timer repeatedly counts from 0 to 255. The output turns on when the timer is at 0, and turns off when the timer matches the output compare register.
    The higher the value in the output compare register, the higher the duty cycle.

    // ----- TIMER 2 -----
    // TCCR2A - Timer/Counter control register A
    // Bit  |    7     |    6     |    5     |    4     |  3  |  2  |    1    |    0    |
    //      |  COM2A1  |  COM2A0  |  COM2B1  |  COM2B0  |  -  |  -  |  WGM21  |  WGM20  |

    // TCCR2B - Timer/Counter control register B
    // Bit  |    7     |    6     |    5     |    4     |    3    |    2    |    1   |    0   |
    //      |  FOC2A   |  FOC2B   |    -     |    -     |  WGM22  |  CS22   |  CS21  |  CS20  |

    //TIMSK2 – Timer/Counter2 Interrupt Mask Register
    // Bit  |    7     |    6     |    5     |    4     |     3   |  2       |    1     |    0    |
    //      |    -     |    -     |    -     |    -     |    -    | OCIE2B   |  OCIE2A  |  TOIE2  |
  */
  cli();                                      //stop interrupts
  pinMode(11, OUTPUT);                        //set PWM pin as an output. On Micro - pin OC2A, PB3, on UNO pin 11
  pinMode(3, OUTPUT);                         //set PWM pin as an output. On Micro - pin OC2B, PD3, on UNO pin  3
  /*
    On TCCR2A, setting the COM2A bits and COM2B bits to 10 provides non-inverted PWM for outputs A and B,
    Clears OC2A & OC2B on Compare Match, set OC2A & OC2B at BOTTOM,
    setting the waveform generation mode bits WGM to 011 selects fast PWM
  */
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  /* On TCCR2B, setting the CS bits to 011 sets the prescaler to divide the clock by 32 */
  TCCR2B  = _BV(CS21) | _BV(CS20);
  /* On TIMSK2, setting the TOIE2 bits to 1 enables the Timer/Counter2 Overflow interrupt. */
  TIMSK2 = _BV(TOIE2);

  sei();                                      //enable interrupts  */
  return;
}  //  end of setup()

/** Main Loop

*/
void loop(void)
{ if (interrupt_Counter >= one_Sec )          //check if one second has expired
    flash_Led();
  /* check for any joystick movement and update motor speeds */
  if ((millis() - joys_Time_Of_Last_Scan) > JoyStick_Scan_Rate) //check if time to scan joystick for changes to x and Y axis
  {
    joys_Time_Of_Last_Scan = millis();        //yes, reset timer, check x axis or rudder
    if (js.check_X_Axis())                     //check if x axis of joystick has changed
    { //yes, process the X change
      int spd;                                //local variabe to store new speed
      int dir;                                //local variabe to store new direction
      js.process_X(&spd, &dir);               //get new speed and direction

      /* check if moving towards starboard and have hit limit switch for movement to starboard */
      if (spd != 0 && dir == TOSTARBOARD && switch_Rudder_Starboard.get_Inhibit_Movement_Flag())
        spd = 0;                               //yes, stop the motor

      /* check if moving towards port and have hit limit switch for movement to port */
      if (spd != 0 && dir == TOPORT && switch_Rudder_Port.get_Inhibit_Movement_Flag())
        spd = 0;                                //yes, stop the motor

      rudder_Motor.set_Requested_Speed(spd);    //set new speed
      rudder_Motor.set_Requested_Dir(dir);      //set new direction
      check_Moved_Off_Rudder_Switches();        //Now check if moved off reed switches
    }
    if (js.check_Y_Axis())                     //check if y axis or boom of joystick has changed
    { //yes, process the Y change
      int spd;                                //local variabe to store new speed
      int dir;                                //local variabe to store new direction
      js.process_Y(&spd, &dir);               //get new speed and direction

      /* check if boom tightening and have hit the boom tight limit */
      if (spd != 0 && dir == TIGHTENING && switch_Boom_Tight.get_Inhibit_Movement_Flag())
        spd = 0;                                 //yes, stop the motor

      /* check if  boom loosening and have hit the boom loose limit switch */
      if (spd != 0 && dir == LOOSENING && switch_Boom_Loose.get_Inhibit_Movement_Flag())
        spd = 0;                                //yes, stop the motor

      boom_Motor.set_Requested_Speed(spd);    //set new speed
      boom_Motor.set_Requested_Dir(dir);      //set new direction
      check_Moved_Off_Boom_Switches();        // Now check if moved off reed switches
    }
    /* now have speed and direction update the motors */
    rudder_Motor.update_Speed();              //update rudder motor speed from the requested speed
    rudder_Motor.update_Dir();                //update rudder motor direction from the requested direction
    boom_Motor.update_Speed();                //update boom motor speed from the requested speed
    boom_Motor.update_Dir();                  //update boom motor direction from the requested direction
  }
  /* Check for and process any changes to the end of travel reed switches,   */
  check_Rudder_Starboard_Switch();
  check_Rudder_Port_Switch();
  check_Boom_Tight_Switch();
  check_Boom_Loose_Switch();
}
//end of loop()
//-------------------------------------


/* check_Moved_Off_Rudder_Switches
   Similiar test when change in reed switch is detected, but because chain bounces,
   can get situation where the logic in change in reed switch does not clear the
   inhibit flag in the "check_xxx_xxx_Switch" function, eg
   chain has reached limit and inhibit flag is set, chain could be stopped and chain bounces,
   reed switch opens and if it remains open the inhbit flag is not cleared because the chain is not moving.

   function checks if moving towards starboard and moved off the limit switch, then clear the inhibit movement flag for port,
   use the actual speed rather than required speed to ensure motor is moving.
   Only reason code is in seperate function is to make the main loop easier to read.
*/
void check_Moved_Off_Rudder_Switches(void)
{
  if (rudder_Motor.get_Current_Speed() != 0 && rudder_Motor.get_Current_Dir() == TOSTARBOARD &&
      switch_Rudder_Port.get_Inhibit_Movement_Flag() && !switch_Rudder_Port.get_Switch_State())
  {
    switch_Rudder_Port.set_Inhibit_Movement_Flag(false);
  }

  /* check if moving towards port and moved off the limit switch, then clear the inhibit movement flag for starboard,
     use the actual speed rather than required speed to ensure motor is moving */
  if (rudder_Motor.get_Current_Speed() != 0 && rudder_Motor.get_Current_Dir() == TOPORT &&
      switch_Rudder_Starboard.get_Inhibit_Movement_Flag() && !switch_Rudder_Starboard.get_Switch_State())
  {
    switch_Rudder_Starboard.set_Inhibit_Movement_Flag(false);
  }
}
//-------------------------------------

/* check_Moved_Off_Boom_Switches()
   function checks if boom tightening and moved off the loose limit switch, then clear the inhibit movement flag for the boom loose limit switch,
   use the actual speed rather than required speed to ensure motor is moving
   Only reason code is in seperate function is to make the main loop easier to read.
*/
void check_Moved_Off_Boom_Switches(void)
{
  if (boom_Motor.get_Current_Speed() != 0 && boom_Motor.get_Current_Dir() == TIGHTENING &&
      switch_Boom_Loose.get_Inhibit_Movement_Flag() && !switch_Boom_Loose.get_Switch_State())
  {
    switch_Boom_Loose.set_Inhibit_Movement_Flag(false);
  }

  /* check if boom loosening and moved off the tight limit switch, then clear the inhibit movement flag for the boom tight limit switch,
     use the actual speed rather than required speed to ensure motor is moving */

  if (boom_Motor.get_Current_Speed() != 0 && boom_Motor.get_Current_Dir() == LOOSENING &&
      switch_Boom_Tight.get_Inhibit_Movement_Flag() && !switch_Boom_Tight.get_Switch_State())
  {
    switch_Boom_Tight.set_Inhibit_Movement_Flag(false);
  }
}
//-------------------------------------

/* Check Rudder starboard switch
    if switch closed and chain moving in that direction set flag to inhibit movement and stop the motor
    if switch released only clear inhibit flag if chain is moving away from the switch. This is to deal with overshoot.
    Only reason code is in seperate function is to make the main loop easier to read.
*/
void check_Rudder_Starboard_Switch(void)
{
  if (switch_Rudder_Starboard.switch_Changed())                   //check if switch has changed state
  {
    if (switch_Rudder_Starboard.get_Switch_State())               //yes, check if switch now closed
    {
      switch_Rudder_Starboard.set_Inhibit_Movement_Flag(true);    //yes, set flag to say can't move to starboard
      if (rudder_Motor.get_Requested_Speed() != 0 && rudder_Motor.get_Requested_Dir() == TOSTARBOARD) //check if not stopped and moving towards starboard
        rudder_Motor.set_Requested_Speed(0);                      //Yes, stop the motor
    }
    else                                                          //switch now open
    {
      if (rudder_Motor.get_Requested_Dir() == TOPORT && rudder_Motor.get_Requested_Speed() != 0) //check if not stopped and moving towards port
        switch_Rudder_Starboard.set_Inhibit_Movement_Flag(false); //yes, then clear the flag. Only clears flag if moving to port in case of overshoot
    }
  }
}
//-------------------------------------

/*  Check Rudder port switch
    if switch closed and chain moving in that direction set flag to inhibit movement and stop the motor
    if switch released only clear inhibit flag if chain is moving away from the switch. This is to deal with overshoot.
    Only reason code is in seperate function is to make the main loop easier to read.
*/
void check_Rudder_Port_Switch(void)
{
  if (switch_Rudder_Port.switch_Changed())                       //check if switch has changed state
  {
    if (switch_Rudder_Port.get_Switch_State())                   //yes, check if switch now closed
    {
      switch_Rudder_Port.set_Inhibit_Movement_Flag(true);        //yes, set flag to say can't move to port
      if (rudder_Motor.get_Requested_Speed() != 0 && rudder_Motor.get_Requested_Dir() == TOPORT)  //check if not stopped and moving towards port
        rudder_Motor.set_Requested_Speed(0);                     //Yes, stop the motor
    }
    else                                                          //switch now open
    {
      if (rudder_Motor.get_Requested_Dir() == TOSTARBOARD && rudder_Motor.get_Requested_Speed() != 0) //check if not stopped and moving towards starboard
        switch_Rudder_Port.set_Inhibit_Movement_Flag(false);      //yes, then clear the flag. Only clears flag if moving to starboard in case of overshoot
    }
  }
}
//-------------------------------------

/* Check Boom loose switch
  if switch closed and chain moving in that direction set flag to inhibit movement and stop the motor
  if switch released only clear inhibit flag if chain is moving away from the switch. This is to deal with overshoot.
  Only reason code is in seperate function is to make the main loop easier to read.
*/
void check_Boom_Loose_Switch(void)
{
  if (switch_Boom_Loose.switch_Changed())                         //check if switch has changed state
  {
    if (switch_Boom_Loose.get_Switch_State())                     //yes, check if switch now closed
    {
      switch_Boom_Loose.set_Inhibit_Movement_Flag(true);          //yes, set flag to say can't loosen the boom
      if (boom_Motor.get_Requested_Speed() != 0 && boom_Motor.get_Requested_Dir() == LOOSENING)  //check if not stopped and the boom loosening
        boom_Motor.set_Requested_Speed(0);                        //Yes, stop the motor
    }
    else                                                          //switch now open
    {
      if (boom_Motor.get_Requested_Dir() == TIGHTENING && boom_Motor.get_Requested_Speed() != 0) //check if not stopped and the boom tightening
        switch_Boom_Loose.set_Inhibit_Movement_Flag(false);      //yes, then clear the flag. Only clears flag if the boom tightening in case of overshoot
    }
  }
}
//-------------------------------------

/* Check Boom Tight Switch
  if switch closed and chain moving in that direction set flag to inhibit movement and stop the motor
  if switch released only clear inhibit flag if chain is moving away from the switch. This is to deal with overshoot.
  Only reason code is in seperate function is to make the main loop easier to read.
*/
void check_Boom_Tight_Switch(void)
{
  if (switch_Boom_Tight.switch_Changed())                         //check if switch has changed state
  {
    if (switch_Boom_Tight.get_Switch_State())                     //yes, check if switch now closed
    {
      switch_Boom_Tight.set_Inhibit_Movement_Flag(true);          //yes, set flag to say can't tighten the boom
      if (boom_Motor.get_Requested_Speed() != 0 && boom_Motor.get_Requested_Dir() == TIGHTENING)  //check if not stopped and the boom tightening
        boom_Motor.set_Requested_Speed(0);                        //Yes, stop the motor
    }
    else                                                          //switch now open
    {
      if (boom_Motor.get_Requested_Dir() == LOOSENING && boom_Motor.get_Requested_Speed() != 0) //check if not stopped and the boom loosening
        switch_Boom_Tight.set_Inhibit_Movement_Flag(false);       //yes, then clear the flag. Only clears flag if boom loosening in case of overshoot
    }
  }
}
//-------------------------------------

/* Flash Led
   Only reason code is in seperate function is to make the main loop easier to read.
*/
void flash_Led(void)
{
  digitalWrite(LedPin, led);                 // continually turn led on and off every second - shows we alive & interrupts working
  led ? led = LOW : led = HIGH;              // swap led state from high to low or low to high
  cli();                                     //interrupts off
  interrupt_Counter = 0;                     //reset counter
  sei();                                     //interrupts on
}


/* end Yacht_V1 */

