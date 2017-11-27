/** Class Switch 
 *
 * If switch is open state is false
 * if switch is closed state is true
 *
*/
#ifndef Switch_h
#define Switch_h

#include "Arduino.h"

class Switch
{
	private:
		unsigned	long debounce_Delay;	//the debounce time in milliseconds
		uint8_t		pin;					//the input on the microprocessor for the switch
		boolean		switch_State;			//the current state of the switch
		boolean		last_Switch_State;		//previous reading of the switch
		unsigned long last_Debounce_Time;

	public:
		Switch(uint8_t, uint8_t);
		boolean	switch_Changed(void);
		boolean	get_Switch_State(void);
};

#endif