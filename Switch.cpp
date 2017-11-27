#include "Arduino.h"
#include "Switch.h"

/* Diagnostics for switch
   If set to 1 and switch state changes prints out the switch state
   Normally set to zero
*/
#define DEBUGSW 0
/* Switch::Switch()
  Constructor
  sets up I/O pin for switch
  sets up initial state for the switch
*/

Switch::Switch(uint8_t par_pin, unsigned long par_debounce)	//set up so can initialise switches
{
  pin = par_pin;						      //pin switch is connected too
  pinMode(pin, INPUT_PULLUP);			//enable internal pullups
  debounce_Time		= par_debounce;	//Set up debounce period
  switch_State		  = false;		  //the current state of the switch
  last_Switch_State	= false;
  last_Debounce_Time	= 0;
}

/* Switch::switch_Changed()
   check if switch state has changed
   only returns a change if the switch has been debounced
*/
bool Switch::switch_Changed(void)
{
  switch_State = !digitalRead(pin);					//invert so when switch pressed returns true
  if ((switch_State != last_Switch_State) && (millis() - last_Debounce_Time > debounce_Time))	//changed for longer then debounce period?
  {
    last_Switch_State = switch_State;	      //yes, pass the new state back to the calling function
#ifdef DEBUGSW
    Serial.print("Switch change, state: ");
    Serial.println(switch_State);
#endif
    last_Debounce_Time = millis();			//reset timer, ready for the next change in switch postion
    return true;							          //tell them there was a change in the switch
  }
  else
  {
    return false;							          //tell them there was no change in the switch
  }
}

/* Switch::get_Switch_State()
   return current state of switch, true closed, false open
*/

bool Switch::get_Switch_State(void)
{
  return switch_State;
}

