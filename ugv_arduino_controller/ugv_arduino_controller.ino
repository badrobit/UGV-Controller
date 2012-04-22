#include <ros.h> 
#include <std_msgs/String.h>
#include <Servo.h>

ros::NodeHandle nh;

unsigned long up_down;                                                                    // place to record inpulse
unsigned long left_right;                                                                 // place to record inpulse
unsigned long dead_man_switch;                                                            // place to record inpulse

Servo Steering_Servo;  
Servo Drive_Servo;                                                                     
Servo Brake_Servo; 

volatile int state;                                                                       //digital pin 2 - green
int dt;											  //required for encoder

void STEERING(int steeringSetpoint);							  //Prototype for Steering System
void DRIVE(int motorSetpoint);								  //Prototype for Driving System
void BRAKE(int brakeSetpoint);								  //Prototype for Braking System
void ENCODER();										  //Prototype for Interrupt System
int PID(double Kp, double Ki, double Kd, int e, int eSum, int eDot);			  //Prototype for PID Controller
int RPM(int timeDiff);                                                                    //Prototype for Velocity Sensor

/* ROS Based Stuff */


std_msgs::String str_msg;
ros::Publisher chatter( "chatter", &str_msg);
char* data; 

String msg_rpm = ""; 
String msg_steering_setpoint = ""; 
String msg_arduino_mode = ""; 
String msg_pid = ""; 

/* ---- END OF ROS ---- */

void messageCb( const std_msgs::String& incoming_msg )
{
  //str_msg.data = incoming_msg.data; 
  build_msg().toCharArray( str_msg.data, 256 ); 
  chatter.publish( &str_msg );
}

ros::Subscriber<std_msgs::String> sub("gps_navigation", &messageCb );

void setup()
{
  Serial.begin(57600);

  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);

  attachInterrupt(0, ENCODER, RISING);                                                     //Pin 0 is actually digital pin 2 for encoder - green
  pinMode(3,INPUT);                                                                        //Encoder direction pin                       - white
  pinMode(4, INPUT);                                                                       //Left & Right motion control ch2  (Yellow) ----------------------  Port 2
  pinMode(5, INPUT);                                                                       //Up & Down (forward & back) motion control ch1 (Purple) ---------  Port 1
  pinMode(6, INPUT);                                                                       //Dead Man Switch ch3(Blue) --------------------------------------  Port 3
  pinMode(32, INPUT);                                                                      //Input for Right Bumper    
  pinMode(34, INPUT);                                                                      //Input for Left Bumper
  Brake_Servo.attach(9);                                                                   //Servo pulse issued from pin 09 for Braking
  Steering_Servo.attach(10);                                                               //Servo pulse issued from pin 10 for Steering
  Drive_Servo.attach(11);                                                                  //Servo pulse issued from pin 11 for Driving
}

void loop()
{
  noInterrupts();
  up_down = pulseIn(5, HIGH);    //records inpulse from RC Rx
  left_right = pulseIn(4, HIGH);
  dead_man_switch = pulseIn(6, HIGH);
  interrupts();
  
  if(dead_man_switch <= 1200)                                                              // STOP THE ROBOT
  {
    Serial.println("DEAD MAN SWITCH ACTIVE");                    //DONT RUN
    msg_arduino_mode = "Stopped"; 
    Brake_Servo.write(0);                                                                 //Engage Brake
    DRIVE(1500);                                                                           //Stall Drive System
    STEERING(1500);                                                                        //Stall Steering System
  }
  else if(dead_man_switch > 1200 && dead_man_switch <= 1600)                               //MANUAL CONTROL
  {
    msg_arduino_mode = "Manual Control"; 
    for(int i = 0; i < 71; i++)
    {
      Brake_Servo.write(170);
    }
    if( up_down >= 1000 && up_down <= 2000 )
    {
      if(up_down < 1200)
        up_down = 1200; 
      else if(up_down > 1800)
        up_down = 1800; 
      else if(up_down > 1490 && up_down < 1510)
        up_down = 1500; 
      else
        up_down = up_down;
      
      //Serial.print(up_down);
      //Serial.print(" ");  
      DRIVE(up_down);
      }
    else
    {
      Serial.println( "No valid motor control input" ); 
    }
    
    if(left_right >= 900 && left_right <= 2100)
    {
      if(left_right < 1200)
        left_right = 1200; 
      else if(left_right > 1800)
        left_right = 1800; 
      else if(left_right > 1490 && left_right < 1510)
        left_right = 1500; 
      else
        left_right = left_right;
      
      //Serial.println(left_right);  
      STEERING(left_right);   
    }
    else
    {
      Serial.println("No valid steering control input"); 
    }
     
  }
  else if(dead_man_switch > 1600)                                                             // AI CONTROL
  {
    Serial.println("AI IN CONTROL");
    msg_arduino_mode = "AI Control"; 
    Brake_Servo.write(0);
    DRIVE(1500);
    STEERING(1500);
   }
  else                                                                                        // BAD SIGNAL IGNORE
  {
    Serial.println("BAD SIGNAL INPUT"); 
  }  
  
  nh.spinOnce(); 
}

void STEERING(int steeringSetpoint)
{
  boolean bumperLeft;
  boolean bumperRight;
  int potSetpoint;
  int potSteering;
  static int e;
  static int eSum;
  int eDot;
  int u;
  int uCorrected;
  int j;
  int farLeftRight;
    
  if(steeringSetpoint < 1200)
   steeringSetpoint = 1200;
  else if(steeringSetpoint > 1800)
   steeringSetpoint = 1800;
  else
   steeringSetpoint = steeringSetpoint;
   
  farLeftRight = 137;
  potSetpoint = map(steeringSetpoint, 1200, 1800, (512-farLeftRight), (512+farLeftRight));
  potSteering = analogRead(13);                                                                      //position potentiometer;
  eDot = e - (potSetpoint - potSteering);
  e = potSetpoint - potSteering;
  eSum = e + eSum;
  
    
  if(eSum < -2000)
    eSum = -2000;
  else if(eSum > 2000)
    eSum = 2000;
  else
    eSum = eSum;
    
  u = PID(2.5, .05, 0, e, eSum, eDot);                                                               //PID function used for as controller for motors
     
  if(u < -farLeftRight)
   u = -farLeftRight;
  else if(u > farLeftRight)
   u = farLeftRight;
  else
   u = u;
    
  uCorrected = map(u, farLeftRight, -farLeftRight, 1400, 1600);                                       //Corrected Signal for pulses of microcontroller 
  bumperLeft = digitalRead(32);                                                                       //attach left bumper to pin 32
  bumperRight = digitalRead(34);                                                                      //attach left bumper to pin 34
   
  if(bumperLeft == 0)                                                                                 //Case 1 
    j = 1;
  else if(bumperRight == 0)                                                                           //Case 2
    j = 2;
  else                                                                                                //default
    j = 0; 
    
  // OUTPUT to the PC104
  msg_steering_setpoint += uCorrected;
 
  switch (j)
  {
    case 1:                                                                                            //bumper left has been engaged
        if(uCorrected < 1500)
          uCorrected = 1500;
        Steering_Servo.write(uCorrected);
        break;
    case 2:                                                                                             //bumper right has been engaged
        if(uCorrected > 1500)
          uCorrected = 1500;
        Steering_Servo.write(uCorrected);
        break;
    default:                                                                                             //no bumpers engaged
        Steering_Servo.write(uCorrected);
        break;
  }
}

void DRIVE(int motorSetpoint)
{
  int timeDiff;
  int rpm;
  int setpoint;
  static int e;
  static int eSum;
  int eDot;
  double u;
  int uCorrected;
  int vel;                                                                                                 //Setting max and min velocity
  
  
  vel = 500;
  setpoint =map(motorSetpoint, 1200, 1800, -vel, vel);                		                           //This is the setpoint mapping velocity never change the one below
  timeDiff = dt;                                                                                           // 
  rpm = RPM(timeDiff);                                                                                     //RPM(int TimeDiff) Function used along with interupts to obatin rpm
  eDot = e - (setpoint - rpm);                                                                             //
  e = setpoint - rpm;                                                                                      //
  eSum = eSum + e;                                                                                         // 
  
  if(eSum < -15000)
    eSum = -15000;
  else if(eSum > 15000)
    eSum = 15000;
  else
    eSum = eSum;
  
  u = PID(0.5, 0.01, 0, e, eSum, eDot);                                                                   //Control Signal calculated using PID
    
  if(u < -vel)
    u = -vel;
  else if(u > vel)
    u = vel;
  else
    u = u;
   
  uCorrected = map(u, -vel, vel, 1300, 1700);
  Drive_Servo.write(uCorrected);
  Serial.println(rpm);
}

int PID(double Kp, double Ki, double Kd, int e, int eSum, int eDot)
{
  double u  = Kp*e + Ki*eSum - Kd*eDot;
  msg_pid += (int)u; 
  return (int)u;
}

int RPM(int timeDiff)
{ 
  static int rpm;
  int rpmOld = rpm;											  //Allows for average to be used
  
  rpm = 60e3/timeDiff;                                                                                    //Simplified math to convert to RPM configured for input of microseconds    
  
  if(rpm < 2 && rpm > -2)                                                                                 //Used to allow Velocity to reach zero
    rpm = 0;
  else if(rpmOld > 300)
    rpm = abs(rpm);
  else if(rpmOld < -300)
    rpm = -(abs(rpm));
  else
    rpm = rpm;
  
  rpm = (9*rpmOld + rpm)/10;    //Averaging calcluation without needing an array
  msg_rpm += rpm; 
  return rpm;
}

void ENCODER()
{
  static int t; 
  state = !state;
  dt = micros() - t;
  
  if(digitalRead(3) == 0)
   dt = -dt;
  else
   dt = dt;
   
  t = micros();
}

String build_msg()
{
  // < control_mode, rpm, steering_setpoint. pid >
  String data;

  data.concat( msg_rpm ); 
  data.concat( "," ); 
  data.concat( msg_arduino_mode ); 
  data.concat( "," ); 
  data.concat( msg_steering_setpoint ); 
  data.concat( "," ); 
  data.concat( msg_pid ); 
 
  Serial.print( "Message Data:\t" ); 
  Serial.println( data );  
  
  return data; 
}  
  


