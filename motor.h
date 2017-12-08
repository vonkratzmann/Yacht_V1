/** Class motor



*/
#ifndef Motor_h
#define Motor_h

#include "Arduino.h"

/* Motor Parameters
   Response time of the system is controlled by the joystick max rate of change value.
   Motor max rate of change is for when the chain hits the stops,
   this condition not controlled by the joystick
*/
const unsigned long Motor_Scan_Rate  = 100;    //scan every 100 ms or 1/10 of a second
const int  Motor_Max_ROC    = 25;     //limit rate of change allowable by the joystick (see comments above)

class Motor
{
  private:
    int	          currentSpeed;	       //store current speed
    int           requestedSpeed;      //store the new requested speed
    int	          currentDir;          //store current direction
    uint8_t       motor_maxspeed;     //maximum physicl speed for the motor
    int           requestedDir;        //store requested direction
    boolean       stopped;             //flag to say if motor stopped or moving. true is stopped, false indicates moving
    long          timeMovingForwards;  //used to track how long moving forwards
    long          timeMovingBackwards; //used to track how long been moving backwards

    uint8_t*      pwm_Reg;	      //Ouput reg to load duty cycle for pwm pulse 0 to 255
    int           dir_Pin;	      //output pin to set direction to the motor

  public:
    Motor(uint8_t*, int, uint8_t);	            //constructor, set PWM comparision register address, direction pin, and maximum speed for the motor
    void	set_Requested_Speed(int);
    int	  get_Requested_Speed(void);
    void	set_Requested_Dir(int);
    int  	get_Requested_Dir(void);
    int   get_Current_Speed();
    int   get_Current_Dir(void);
    void  update_Speed(void);
    void  update_Dir(void);
    void  clearFwdTimer(void);
    void  clearRevTimer(void);
    long  getFwdTimer(void);
    long  getRevTimer(void);
    void  setFwdTimer(void);
    void  setRevTimer(void);
};

#endif
