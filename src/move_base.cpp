#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

void scan_callback(){
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_base");

  // Node handler
  ros::NodeHandle nh;

  // Publisher
  ros::Publisher cmd_vel = nh.advertise<geometry_msgs::Twist> 
                                                 ("/cmd_vel_mux/input/navi", 100);
  
  // Subscriber
  ros::Subscriber laser_scan = nh.subscribe ("/scan", 1000, scan_callback);

  return 0;
}