#include <math.h>
#include <iostream>
#include <ros/ros.h>
#include <gps_common/GPSFix.h>
#include <sensor_msgs/NavSatFix.h>
#include "std_msgs/String.h"
#include <sstream>

static const double PI = 3.13159265; 
static const double DEGREE_TO_RADIAN = 0.017453292519943295769236907684886;
static const double EARTH_RADIUS_IN_METERS = 6372797.560856;

//// 
//
//  THIS WILL NOT WORK IF YOU ATTEMPT TO CROSS THE INTERNATIONAL DATE LINE OR THE EQUATOR .. SORRY
//
////
class gps_navigation
{
public:
    double previous_lat;
    double previous_long;

    double current_lat;
    double current_long;

    double goal_lat; 
    double goal_long; 

    double system_heading;

    double distance_to_goal;
    double angle_to_goal;

    

    // Type for GPS messages
    //gps_common::GPSFix gpsMsg;
    sensor_msgs::NavSatFix gpsMsg; 

  // Constructor
  gps_navigation(ros::NodeHandle nh_) : n(nh_)
  {
    // Subscribing to the topic /fix
    //gps_sub = n.subscribe("/fix", 100, &gps_navigation::gpsCallback, this);
    gps_sub = n.subscribe("/fix", 100, &gps_navigation::gpsCallback, this);

    gps_nav_pub = n.advertise<std_msgs::String>("gps_navigation", 1000);
  }

  std::string get_msg_to_arduino()
  {
    return msg_to_arduino; 
  }


    // Callback Function for the GPS
    //void gpsCallback(const gps_common::GPSFixConstPtr &msg)
    void gpsCallback( const sensor_msgs::NavSatFixConstPtr &msg )
    {
        /*
        // TESTING ONLY
        previous_lat = 45.950023; 
        previous_long = -66.641768; 

        current_lat = 45.90001;
        current_long = -66.641822;
        */

        // a location just outside UNB's Student Union Building.
        goal_lat = 45.944801; 
        goal_long = -66.642275; 

        gpsMsg = *msg;

        previous_lat = current_lat; 
        previous_long = current_long; 
        // Getting the data from the GPS
        current_lat = gpsMsg.latitude; 
        current_long = gpsMsg.longitude;
        
        std::cout << "Current Latitude:\t" << current_lat << std::endl;
        std::cout << "Current Longitude:\t" << current_long << std::endl;

        compute_system_heading( previous_lat, previous_long, current_lat, current_long, &system_heading ); 
        std::cout << "Current System Heading:\t" << system_heading << std::endl; 
        
        compute_distance_to_goal( previous_lat, previous_long, current_lat, current_long, &distance_to_goal );
        std::cout << "Distance to Goal:\t" << distance_to_goal << std::endl; 
        std::cout << "Distance in KM:\t" << distance_to_goal / 1000 << std::endl; 

        compute_angle_to_goal( current_lat, current_long, system_heading, goal_lat, goal_long, &angle_to_goal );
        std::cout << "Angle to Goal:\t" << angle_to_goal << std::endl; 
        std::cout << "Angle in Deg:\t" << angle_to_goal * 180 / PI << std::endl;
        std::cout << "\n\n"; 

        std::stringstream ss;
        // ANGLE_TO_GOAL
        //ss << angle_to_goal;
        //ss << ","; 
        // SPEED_TO_GOAL
        //ss << 1800; 
        ss << "MAT"; 
        msg_to_arduino = ss.str(); 

        msg_to_publish.data = msg_to_arduino; 

        gps_nav_pub.publish(msg_to_publish);

        std::cout << "Message to Arduino:\t:" << msg_to_arduino << std::endl; 
    }

    void compute_distance_to_goal( double gps_lat, double gps_long, double goal_lat, double goal_long, double* dist_to_goal )
    {
        double lat_arc = ( gps_lat - goal_lat ) * DEGREE_TO_RADIAN;
        double long_arc = ( gps_long - goal_long ) * DEGREE_TO_RADIAN; 

        double lat_h = sin( lat_arc * 0.50 ); 
        lat_h *= lat_h; 

        double long_h = sin( long_arc * 0.50 ); 
        long_h *= long_h; 

        double temp = cos( gps_lat * DEGREE_TO_RADIAN ) * cos( goal_lat * DEGREE_TO_RADIAN ); 
        *dist_to_goal = ( EARTH_RADIUS_IN_METERS * asin( sqrt( lat_h + temp * long_h ) ) ); 
    } 

    void compute_angle_to_goal( double current_lat, double current_long, double system_heading, double goal_lat, double goal_long, double* angle_to_goal )
    {
        double line_to_goal = atan2( ( goal_long - current_long ), ( goal_lat - current_lat ) ); 

        *angle_to_goal = line_to_goal - system_heading; 
    }

    void compute_system_heading( double previous_lat, double previous_long, double current_lat, double current_long, double* heading )
    {
        *heading = atan2( ( current_long - previous_long ), ( current_lat - previous_lat ) );
    }

private:

  std_msgs::String msg_to_publish;

  // < angle_to_dist, speed > 
  std::string msg_to_arduino; 

  // Nodehandle
  ros::NodeHandle n;

  // Subscriber
  ros::Subscriber gps_sub;

  // Publisher
  ros::Publisher gps_nav_pub;
};


int main(int argc, char** argv)
{
    // Initializing the node for the GPS
    ros::init(argc, argv, "gps_navigation");
    ros::NodeHandle nh_;
    ros::NodeHandle n;

    gps_navigation *p = new gps_navigation( nh_ );

    ros::spin();
    return 0;
}