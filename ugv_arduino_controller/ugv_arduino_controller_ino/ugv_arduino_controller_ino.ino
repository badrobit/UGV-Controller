#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

ros::NodeHandle nh;

char hello[13] = "hello world!";

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

void messageCb( const std_msgs::String& toggle_msg)
{
  str_msg = toggle_msg;
  chatter.publish( &str_msg );
}

ros::Subscriber<std_msgs::String> sub("gps_navigation", &messageCb );



void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);
}

void loop()
{
  nh.spinOnce();
  delay(200);
}


