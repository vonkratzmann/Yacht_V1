/* Class Switch
   If switch is open state is false
   if switch is closed state is true
*/
#ifndef Switch_h
#define Switch_h

#include "Arduino.h"

class Switch
{
  private:
    unsigned	long debounce_Time;	  //the debounce time in milliseconds
    uint8_t		pin;					        //the input on the microprocessor for the switch
    bool		switch_State;			      //the current state of the switch
    bool		last_Switch_State;		  //previous reading of the switch
    unsigned long last_Time_Changed;  //records time switched changed
    bool    previous_debounced_state; //previous debounced state

  public:
    Switch(uint8_t, unsigned long);
    bool	switch_Changed(void);     //check if switch has changed
    bool	get_Switch_State(void);   //get the dedounced state of the switch
};

#endif
