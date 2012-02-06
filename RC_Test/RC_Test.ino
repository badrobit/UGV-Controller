#include <Servo.h>

int ch1 = 5;  // Up & Down (forward & back) motion control (Purple)
int ch2 = 3;  // Left & Right motion control  (Yellow)
int ch3 = 6;  // Dead Man Switch (Blue)

unsigned long up_down;  // place to record inpulse
unsigned long left_right;
unsigned long dead_man_switch;

Servo Steering_Servo;                                                                      //Create a Servo object for steering motor
long Steering_Integral = 0;    //Initalization of Integral portion

void STEERING( int steering_set_point );
long PID(int setpoint,int sensor, int Kp, float Ki, int Kd, long* eSum);

void setup()
{
  pinMode(ch1, INPUT);    // Tells Aduino pins are inputs
  pinMode(ch2, INPUT);
  pinMode(ch3, INPUT);
  Serial.begin(9600 );
  pinMode(13, INPUT);                                                                      //Input for Left Bumper
  pinMode(12, INPUT);                                                                      //Input for Right Bumper    
  Steering_Servo.attach(10);                                                               //Servo pulse issued from pin 10

}

void loop()
{
  up_down = pulseIn(ch1, HIGH);    //records inpulse from RC Rx
  left_right = pulseIn(ch2, HIGH);
  dead_man_switch = pulseIn(ch3, HIGH);
  
  if( dead_man_switch >= 900 && dead_man_switch <= 1100 ) // STOP THE ROBOT
  {
    Serial.println( "DEAD MAN SWITCH ACTIVE" );      // DONT RUN
  }
  else if( dead_man_switch >= 1400 && dead_man_switch <= 1600 ) // MANUAL CONTROL
  {
    Serial.println( "PERSON IN CONTROL" ); 
    
    /* The following must be changed to output to the motors. */
    if( up_down >= 900 && up_down <= 1400 )
    {
      Serial.print("\tUP CONTROL:\t");
      Serial.print( up_down );
      Serial.print(",");
    }
    else if( up_down >= 1600 && up_down <= 2100 )
    {
      Serial.print( "\tDOWN CONTROL:\t");
      Serial.print( up_down );
      Serial.print(",");
    }
    else 
    {
      Serial.println( "No valid forward control input" );
    }
    
    //408, 616
    if( left_right >=1000 && left_right <= 1400 )
    {
      // <512
      int steering_set_point = (left_right*0.34);
      Serial.print("\tLEFT CONTROL:\t");
      Serial.print( steering_set_point );
      Serial.print( "\t" ); 
      Serial.print( left_right ); 
      //void STEERING( int steering_set_point )
    }
    else if( left_right >= 1500 && left_right <= 2100 )
    {
      //>512
      int steering_set_point = (left_right*0.34);
      Serial.print("\tRIGHT CONTROL:\t");
      Serial.print( steering_set_point );
      Serial.print( "\t" ); 
      Serial.print( left_right );
    }
    else
    {
      Serial.println( "No valid steering control input" ); 
    }
    
    Serial.println(" ");
  
  }
  else if( dead_man_switch >= 1900 && dead_man_switch <= 2100 ) // AI CONTROL
  {
    Serial.println( "AI IN CONTROL" );  
  }
  else // BAD SIGNAL IGNORE
  {
    Serial.println( "BAD SIGNAL INPUT" ); 
  }
  
  delay( 200 );                      //changes smaple rate 2000 = 2 seconds
}


void STEERING( int steering_set_point )
{
  boolean bumper_left;
  boolean bumper_right;
  int pot_setpoint;
  int pot_steering;
  int steering_controller_initial;
  int steering_controller_corrected;
  int j;
  
  bumper_left = digitalRead(13);
  bumper_right = digitalRead(12);
  
  if(bumper_left == 0)
  {
    j = 1;
  }
  else if(bumper_right == 0)
  {
    j = 2;
  }
  else
  {
    j = 0;
  }
  
   int potentiometer = analogRead(1);
   if(potentiometer < 100)
   {
     potentiometer = 100;
   }
   else if(potentiometer > 923)
   {
     potentiometer = 923;
   }
   else
   {
    potentiometer = potentiometer;
   }
 
   pot_setpoint = map(potentiometer, 100, 923, 408, 616);                                            //potentiometer input converted to minimum and maximum setpoints
   //pot_steering = analogRead(5);                                                                     //position potentiometer;
   pot_steering = steering_set_point; 
   steering_controller_initial = PID(pot_setpoint, pot_steering,1 , 0.005, 0, &Steering_Integral);   //PID function used for as controller for motors
  
   if(steering_controller_initial < -100)
   {
     Steering_Integral = 0;
     steering_controller_initial = -100;
   }
   else if(steering_controller_initial > 100)
   {
     Steering_Integral = 0;
     steering_controller_initial = 100;
   }
   else
   {
     steering_controller_initial = steering_controller_initial;
   }
 
   steering_controller_corrected = map(steering_controller_initial, 100, -100, 1000, 2000);         //Corrected Signal for pulses of microcontroller 
 
  switch (j)
  {
    case 1:
        //bumper left has been engaged
        if(steering_controller_corrected < 1500)
        {
          steering_controller_corrected = 1500;
        }        
        Steering_Servo.write(steering_controller_corrected);
        break;
    
    case 2:
        //bumper right has been engaged
        if(steering_controller_corrected > 1500)
        {
          steering_controller_corrected = 1500;
        }
        Steering_Servo.write(steering_controller_corrected);
        break;
    
    default:
        //no bumpers engaged
        Steering_Servo.write(steering_controller_corrected);
        break;
  }
  
  Serial.println(steering_controller_initial);
}

long PID(int setpoint,int sensor, int Kp, float Ki, int Kd, long* eSum)
{
  long e = setpoint - sensor;
  long eDot;
  *eSum += e;
  float u  = Kp*e + Ki*(*eSum) - Kd*eDot;
  return long(u);
}
