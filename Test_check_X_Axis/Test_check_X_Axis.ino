/** Drives a yacht controller for Brad Connelly
   Author:    Kirk Kratzmann
   Version:   1.0
   Date:      01/12/2017
*/

/** Test routine*/

#include "E:\Documents\Google Drive\Kirk\Solve\2014-0059 (BAR 14-6) Brad Connelly - Repair to Joystick Operated Yacht\Yacht_V1\Yacht.h"
#include "E:\Documents\Google Drive\Kirk\Solve\2014-0059 (BAR 14-6) Brad Connelly - Repair to Joystick Operated Yacht\Yacht_V1\JoyStick.h"

/* define objects */

/* define joystick */
JoyStick js; 

int testdata;
bool b;

/** Setup */
void setup(void) 
{
  Serial.begin(9600);                   //set up serial port for any debug prints
  DEBUG_PRINTLN("Started");                       
  return;
}  
//  end of setup()

/* Main Loop */
void loop(void)
{
    b = js.check_X_Axis();                     //check if x axis of joystick has changed
    
    b = js.check_Y_Axis();                     //check if y axis or boom of joystick has changed
    
    while(1)
    {}
   
}
//end of loop()
//-------------------------------------


