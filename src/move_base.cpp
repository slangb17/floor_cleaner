#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

sensor_msgs::LaserScan laser_msg;
std::vector<float> laser_ranges;
geometry_msgs::Twist velocity_msg;

void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg){
  
  laser_msg = *scan_msg;

  laser_ranges = laser_msg.ranges;

  ROS_INFO("%d", laser_ranges.size());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_base");

  // Node handler
  ros::NodeHandle n;

  // Publisher
  ros::Publisher cmd_vel = n.advertise<geometry_msgs::Twist> 
                                                 ("/cmd_vel_mux/input/navi", 100);
  
  // Subscriber
  ros::Subscriber laser_scan = n.subscribe("/scan", 1000, scan_callback);

  while(ros::ok()){



    ros::spinOnce();

  }
  return 0;
}