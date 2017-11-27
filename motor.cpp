#include "Arduino.h"
#include "Motor.h"

/** Motor constructor
 *
 * constructor, set speed pin and direction pin 
 *
 */

Motor:Motor
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