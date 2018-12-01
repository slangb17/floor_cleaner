#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

//
ros::Publisher cmd_vel;

//
sensor_msgs::LaserScan laser_msg;
std::vector<float> laser_ranges;
geometry_msgs::Twist velocity_msg;
unsigned int laser_ranges_size = 0;
bool wallDetected = false;
bool wallToTheLeft = false;
bool cornerFound = false;

void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{

  laser_msg = *scan_msg;

  laser_ranges = laser_msg.ranges;
  laser_ranges_size = laser_ranges.size();

  float laser_ranges_min = 5.0f;
  float laser_ranges_max = 0.0f;

  for (int i = 0; i < laser_ranges_size; i++)
  {
    if (laser_ranges[i] < laser_ranges_min)
    {
      laser_ranges_min = laser_ranges[i];
    }
    if (laser_ranges[i] > laser_ranges_max)
    {
      laser_ranges_max = laser_ranges[i];
    }
  }

  float laser_ranges_left = 0.0f;
  float laser_ranges_center = 0.0f;
  float laser_ranges_right = 0.0f;

  for (int i = 0; i < laser_ranges_size; i++)
  {
    if (i < laser_ranges_size * 1 / 3)
    {
      laser_ranges_right += laser_ranges[i];
    }
    else if (i < laser_ranges_size * 2 / 3)
    {
      laser_ranges_center += laser_ranges[i];
    }
    else
    {
      laser_ranges_left += laser_ranges[i];
    }
  }

  laser_ranges_right /= (laser_ranges_size / 3);
  laser_ranges_center /= (laser_ranges_size / 3);
  laser_ranges_left /= (laser_ranges_size / 3);

  ROS_INFO("Left: %f, center: %f, right: %f", laser_ranges_left,
           laser_ranges_center, laser_ranges_right);

  if (cornerFound != true)
  {
    if (laser_ranges_min < 0.4){
      velocity_msg.linear.x = 0.01;
      wallDetected = true;
    }
    else if (laser_ranges_min < 0.6)
    {
      velocity_msg.linear.x = 0.05;
      if (wallDetected != true)
      {
        wallDetected = true;
        if (laser_ranges_left < laser_ranges_right)
        {
          wallToTheLeft = true;
        }
        else
        {
          wallToTheLeft = false;
        }
      }
    }

    else if (laser_ranges_min < 0.8)
    {
      velocity_msg.linear.x = 0.15;
    }

    else 
    {
      velocity_msg.linear.x = 0.25;  
    }
  }



  if (wallDetected == true)
  {
    if (wallToTheLeft == true)
    {
      if (laser_ranges[639] < 0.9)
      {
        velocity_msg.angular.z = -0.3;
      }
      else
      {
        velocity_msg.angular.z = 0.3;
      }
    }

    else
    {
      if (laser_ranges[0] < 0.9)
      {
        velocity_msg.angular.z = 0.3;
      }
      else
      {
        velocity_msg.angular.z = -0.3;
      }
    }
  }

  if (wallDetected == true && (laser_ranges_left > laser_ranges_center ||
                               laser_ranges_right > laser_ranges_center))
  {
    //cornerFound = true;
  }

  if (cornerFound == true)
  {
    ROS_WARN("I FOUND A CORNER!");
    velocity_msg.linear.x = 0;
    velocity_msg.angular.z = 0;
  }

  cmd_vel.publish(velocity_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_base");

  // Node handler
  ros::NodeHandle n;

  // Publisher
  cmd_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 100);

  // Subscriber
  ros::Subscriber laser_scan = n.subscribe("/scan", 1000, scan_callback);

  while (ros::ok())
  {

    ros::spinOnce();
  }
  return 0;
}