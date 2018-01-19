#include "Yacht.h"
#include "Switch.h"

/* Diagnostics for switch
   If set to 1 and switch state changes prints out the switch state
   Normally set to zero
*/

/* Switch::Switch()
  Constructor
  sets up I/O pin for switch
  sets up initial state for the switch
*/

Switch::Switch(uint8_t par_pin, unsigned long par_debounce)	//set up so can initialise switches
{
  pin = par_pin;						      //pin switch is connected too
  pinMode(pin, INPUT_PULLUP);			//enable internal pullups
  debounce_Time		  = par_debounce;	//Set up debounce period
  switch_State		  = false;		  //the initial state of the switch
  last_Switch_State	= false;
  previous_Debounced_State = false;
  last_Time_Changed	= millis();   //record time switched state changed
  inhibit_Movement = false;       //initially allow movement
}

/* Switch::switch_Changed()
   check if switch state has changed
   only returns a change if the switch has been debounced
   for the debouncing to work this routine has to be called regularly by the main loop
*/
bool Switch::switch_Changed(void)
{
  switch_State = !digitalRead(pin);				  //invert so when switch pressed returns true
  switch_State = digitalRead(pin) ? false : true;
  if (switch_State != last_Switch_State)    //if does not equal last state
  {
    last_Time_Changed = millis();           //record time of change
    last_Switch_State = switch_State;       //yes, update state
  }
  if ((switch_State != previous_Debounced_State) && ((millis() - last_Time_Changed) > debounce_Time))	//changed for longer then debounce period?
  {
    previous_Debounced_State = switch_State;	      //yes, update state

    SWITCH_DEBUG_FILE("Function: ");
    SWITCH_DEBUG_FILE(__FILE__);
    SWITCH_DEBUG_FILE(",");
    SWITCH_DEBUG_PRINT(__FUNCTION__);
    SWITCH_DEBUG_PRINT(" switch_State: ");
    SWITCH_DEBUG_PRINTLN(switch_State);

    last_Time_Changed = millis();			  //reset timer, ready for the next change in switch postion
    return true;							          //tell them there was a change in the switch
  }
  else
  {
    return false;							          //tell them there was no change in the switch
  }
}

/* Switch::get_Switch_State()
   return current debounced state of switch, true closed, false open
*/
bool Switch::get_Switch_State(void)
{
  return switch_State;
}

/* Switch::set_Inhibit_Movement(bool)
   update flag to say if movement is inhibited by the chain the direction this switch
*/
void  Switch::set_Inhibit_Movement_Flag(bool flag)
{
  inhibit_Movement = flag;    //update the flag

  SWITCH_DEBUG_FILE("Function: ");
  SWITCH_DEBUG_FILE(__FILE__);
  SWITCH_DEBUG_FILE(",");
  SWITCH_DEBUG_PRINT(__FUNCTION__);
  SWITCH_DEBUG_PRINT(" inhibit_Movement: ");
  SWITCH_DEBUG_PRINTLN(inhibit_Movement);
}

/* Switch::get_Inhibit_Movement(void)
  get flag to say if movement is inhibited by the chain to the direction of this switch
*/
bool  Switch::get_Inhibit_Movement_Flag(void)
{
  return inhibit_Movement; //get flag status
}
