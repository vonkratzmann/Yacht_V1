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

/* ISR */
//uses counter 2 to generate pwm pulses freq is 2kHz

/** I/O
   all inputs have internal pullups enabled
   so switches are 0 when pressed, and a 1 when released
*/

#include "Yacht.h"
#include "JoyStick.h"
#include "Switch.h"
#include "Motor.h"

int interrupt_Counter = 0;           //used in main loop to show the ISR is running
const int one_Sec = 1000;            //used in main loop to show the ISR is running, flashes led off and on each second

unsigned long  joys_Time_Of_Last_Scan = 0;    //track when we last scanned for joystick changes
unsigned long  motor_Time_Of_Last_Scan = 0;   //track when we last updated motor speeds

uint8_t led = LOW;                            //state of led, initially off

#ifdef DEBUGISR1
unsigned long entry_Time, exit_Time;        //used to check overhead of ISR
unsigned long tmp1, tmp2;
#endif

/* define objects */

/* define joystick */
JoyStick js;  //define joystick

/* define reed switches */
Switch switch_Rudder_Port(rudder_Port_EndofTravel_Pin, Debounce);
Switch switch_Rudder_Starboard(rudder_Starboard_EndofTravel_Pin, Debounce);
Switch switch_Boom_Tight(boom_Tight_EndofTravel_Pin, Debounce);
Switch switch_Boom_Loose(boom_Loose_EndofTravel_Pin, Debounce);

/* define motors */
Motor rudder_Motor(rudder_Pwm_Pin, rudder_Dir_Pin);
Motor boom_Motor(boom_Pwm_Pin, boom_Dir_Pin);


/* Interrupt Service Routine for timer 2
  to be used for motor
*/

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
  pinMode(LedPin,          OUTPUT);
  digitalWrite(LedPin,       HIGH);     // sets the LED on

  /* Diagnostics for startup
     If set to 1 prints out startup maessage
     Normally set to zero
  */
  Serial.begin(9600);                   //set up serial port for any debug prints

  DEBUG_PRINTLN("Started");

  /* Set up timer interrupt */

  /* Timer 0, is 8 bits used by fuction millis();  Timer 1, 3, 4, 5 are 16 bits;  Timer 0, 2 are 8 bits
    timers used by anolgue write

    Use Timer 2 to generate an interrupt every 500 microseconds to process pwm pulses to motors
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
  if (interrupt_Counter >= one_Sec )  //check if one second has expired
  {
    digitalWrite(LedPin, led);                 // continually turn led on and off every second - shows we alive & interrupts working
    led ? led = LOW : led = HIGH;              // swap led state from high to low or low to high
    cli();                                     //interrupts off
    interrupt_Counter = 0;                     //reset counter
    sei();                                     //interrupts on
  }
  /* check for any joystick movement and update motor speeds */
  if ((millis() - joys_Time_Of_Last_Scan) > JoyStick_Scan_Rate) //check if time to scan joystick for changes to x and Y axis
  {
    joys_Time_Of_Last_Scan = millis();        //yes, reset timer

    /* check x axis or rudder */
    if (js.check_X_Axis())                     //check if x axis of joystick has changed
    { //yes, process the X change
      int spd;                                //local variabe to store new speed
      uint8_t dir;                            //local variabe to store new direction
      js.process_X(&spd, &dir);               //get new speed and direction

      /* check if moving towards starboard and have hit limit switch for movement to starboard */
      if (spd != 0 && dir == TOSTARBOARD && switch_Rudder_Starboard.get_Inhibit_Movement_Flag())
      {
        spd = 0;                               //yes, stop the motor
      }

      /* check if moving towards port and have hit limit switch for movement to port */
      if (spd != 0 && dir == TOPORT && switch_Rudder_Port.get_Inhibit_Movement_Flag())
      {
        spd = 0;                                //yes, stop the motor
      }

      rudder_Motor.set_Requested_Speed(spd);    //set new speed
      rudder_Motor.set_Requested_Dir(dir);      //set new direction

      /* check if moving towards starboard and moved off the limit switch, then clear the inhibit movement flag for port,
        use the actual speed rather than required speed to ensure motor is moving */

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

    /* check y axis or boom */
    if (js.check_Y_Axis())                     //check if y axis of joystick has changed
    { //yes, process the Y change
      int spd;                                //local variabe to store new speed
      uint8_t dir;                            //local variabe to store new direction
      js.process_Y(&spd, &dir);               //get new speed and direction

      /* check if boom tightening and have hit the boom tight limit */
      if (spd != 0 && dir == TIGHTENING && switch_Boom_Tight.get_Inhibit_Movement_Flag())
      {
        spd = 0;                                 //yes, stop the motor
      }

      /* check if  boom loosening and have hit the boom loose limit switch */
      if (spd != 0 && dir == LOOSENING && switch_Boom_Loose.get_Inhibit_Movement_Flag())
      {
        spd = 0;                                //yes, stop the motor
      }

      boom_Motor.set_Requested_Speed(spd);    //set new speed
      boom_Motor.set_Requested_Dir(dir);      //set new direction

      /* check if boom tightening and moved off the loose limit switch, then clear the inhibit movement flag for the boom loose limit switch,
        use the actual speed rather than required speed to ensure motor is moving */
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

    /* now have speed and direction update the motors */
    rudder_Motor.update_Speed();              //update rudder motor speed from the requested speed
    rudder_Motor.update_Dir();                //update rudder motor direction from the requested direction

    boom_Motor.update_Speed();                //update boom motor speed from the requested speed
    boom_Motor.update_Dir();                  //update boom motor direction from the requested direction

    motor_Time_Of_Last_Scan = millis();       //yes, reset timer
  }

  /* check for any changes to end of travel reed switches
     if switch closed
      and chain moving in that direction set flag to inhibit movement and stop the motor
     if switch released
      only clear inhibit flag if chain is moving away from the switch. This is to deal with overshoot
  */

  /* Rudder starboard switch */
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

  /* Rudder port switch */
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

  /* Boom tight switch */
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

  /* Boom loose switch */
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
//end of loop()

/* end */

