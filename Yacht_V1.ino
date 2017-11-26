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
    Mainsail Motor which tightens or loosens the mainsail
   A joystick connected to two 10 bit ADCS, forwards 0 to 511,backwards 512 to 1023
   On/Off switch
   12V DC battery
   Microprocessor
   Driver Printed Circuit Boards for the two DC motors
   5v DC power supply for the microprocessor
   Each motor via a sprocket drives a loop to drive the rudder or mainsail
   The rudder loop consists of chain and stainless steel wire
   The mainsail loop consists of a chain and shock cord which via a series of fixed pulleys and cables tighten or release the mainsail
   The travel of the loop is limited in both directions so only the chain is on the sprocket at all times
   To limit the travel of the loop, each loop has two reed switches to detect the end of the chain
   Connected to the end of the chain is a magnet to activate the reed switch

   The program reads the joystick at a preddefined rate
   then converts these readings into a Pulse width Modulated output and direction to drive the rudder and mainsail motors direction and speed
   on joystick x is the rudder, y is the mainsail

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

//#include "arduino2.h"  // include the fast I/O 2 functions
#include "JoyStick.h"
#define DEBUGSTARTUP 0

#define DEBUGISR1 0
#define DEBUGISR2 0

volatile unsigned int interrupt_Counter = 0;     //used in main loop to show the ISR is running
const unsigned int one_Sec = 2000;               //used in main loop to show the ISR is running, flashes led off and on each second

unsigned long  joys_Time_Of_Last_Scan = 0;   //track when we last scanned for joystick changes

unsigned long debounce = 50;            //debounce time for switch in millisecs

/* Rudder Parameters */

/* Mainsail Parameters */

/* all inputs & outputs (I/O) allocated below rather than in classes to ensure no overlaps or errors */

/** motors
 *
 * define i/o for each motor driver board, each board has 2 inputs: direction & pwm
 *
*/
const uint8_t  rudder_Dir_Pin     = 8;       //sets direction rudder motor turns
const uint8_t  rudder_Pwm_Pin     = 9;       //PWM pulse to set the speed of the rudder motor
const uint8_t  mainsail_Dir_Pin   = 7;       //sets the direction the mainsail motor turns
const uint8_t  mainsail_Pwm_Pin   = 6;       //PWM pulse to set the speed of the mainsail motor

/** end of travel detectors
 *
 * define i/O for reed switches to detect end of travel for the chain on each motor
 *
*/
const uint8_t  rudder_Port_EndofTravel_Pin      = 2;        
const uint8_t  rudder_Starboard_EndofTravel_Pin = 3;        
const uint8_t  mainsail_Tight_EndofTravel_Pin   = 4;        
const uint8_t  mainsail_Loose_EndofTravel_Pin   = 5;        

const uint8_t ledPin =  13; //LED connected to digital pin 13
uint8_t led = LOW;  //initial state of led

#ifdef DEBUGISR1
unsigned long entry_Time, exit_Time;        //used to check overhead of ISR
unsigned long tmp1, tmp2;
#endif

/*  motor */
/*
  class motor
  {
    uint8_t spd;                              //store current speed
    uint8_t dir;                              //store current direction
    volatile uint8_t isr_Spd;                 //used by function called from ISR
    volatile uint8_t isr_Dir;                 //used by function called from ISR

    uint8_t pwm_Pin;                         //Ouput pin to pulse to drive the stepper motor
    uint8_t dir_Pin;                          //output pin to set direction to the stepper motor
    boolean pulse_Flag;                       //set if pulse output is high
  public:
    motor(uint8_t, uint8_t);
    void    set_Speed(uint8_t);
    uint8_t get_Speed(void);
  #ifdef DEBUGISR2
    uint8_t get_Isr_Speed(void);
  #endif
    void    set_Dir(uint8_t);
    uint8_t get_Dir(void);
    boolean chk4_Fault(void);
    void    copy2_Isr(void);
    //start of stepper::chk_Pulse              //define here to make "inline' function for ISR
    void    chk_Pulse(void)                   // called by ISR, so interrupts are off
    {
      if (pulse_Flag)                         //firstly check if the step o/p pulse to the steeper motor driver was set in last ISR
      {
        digitalWrite(step_Pin, LOW);          //yes then set it low
        pulse_Flag = false;                   //clear flag to say it was high
        isr_Spd =  spd;                       //update counter
      }
      else                                    //step pulse not high, so check the counter
      {
        if (isr_Spd)                          //only process if stepper is moving, ie non zero
        { //decrement counter and check if zero
          isr_Spd = isr_Spd - 1;              //used this rather '--' for efficiency
          if (!isr_Spd)                       //has counter gone to zero
          { //yes
            digitalWrite(dir_Pin, isr_Dir);   //update in case direction changed
            digitalWrite(step_Pin, HIGH);     //pulse the steeper motor
            pulse_Flag = true;                //set flag, so next time ISR is called it checks if o/p is high
          }
        }
      }
    } //end of stepper::chk_Pulse()
  };//end of stepper class
*/
class Switch
{
    unsigned long debounce_Delay;        //the debounce time in milliseconds
    uint8_t  pin;                        //the input on the microprocessor for the switch
    boolean  switch_State;               //the current state of the switch
    boolean  last_Switch_State;          //previous reading of the switch
    unsigned long last_Debounce_Time ;

  public:
    Switch();
    Switch(uint8_t, uint8_t);
    boolean switch_Change(uint8_t &);
    uint8_t get_Switch_State(void);
}; //end of class Switch

Switch::Switch()                        //used to declare switches
{
}
Switch::Switch(uint8_t par_pin, uint8_t par_debounce)    //set up so can initialise swtches
{
  pin = par_pin;                              //store here as used by other members of this class
  pinMode(pin, INPUT_PULLUP);           //enable internal pullups
  debounce_Delay     = par_debounce;
  switch_State       = false;           //the current state of the switch
  last_Switch_State  = false;
  last_Debounce_Time = 0;
} //end of Switch::Switch()

boolean Switch::switch_Change(uint8_t &sw)  //check if change, returns true if a change after the debounce, otherwise returns false
{
  switch_State = !digitalRead(pin);                   //invert so when switch pressed returns true
  if ((switch_State != last_Switch_State) && (millis() - last_Debounce_Time > debounce_Delay)) //change for longer then debounce period?
  { //yes
    sw = last_Switch_State = switch_State;           //pass the new state back to the calling function
#ifdef DEBUGSW
    Serial.print("Switch change, state: ");
    Serial.println(sw);
#endif
    last_Debounce_Time = millis();                   //reset timer, ready for the next change in switch postion
    return true;                                     //tell them there was a change in the switch
  }
  else
  {
    return false;                                    //tell them there was no change in the switch
  }
}  //end of Switch::switch_Change()

uint8_t Switch::get_Switch_State(void)              //note not debounced
{
  return !digitalRead(pin);                         //invert so when switch pressed returns true
}  //end of switch::get_Switch_state()



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


/* define objects */

/* define motors */

JoyStick js;  //define joystick

/** Setup
 *  
 *  
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
 *
 *
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
    boolean flag;                         //flag to indicate joystick position has changed
    flag = js.check_Y_Pos();              //must run both functions to ensure current x & y positions are updated
    flag |= js.check_X_Pos();
    if (flag)                             //check if x or y axis changed,
    { //yes
      //do something

    }
  }
}
//end of loop()


/* end */

