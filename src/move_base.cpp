#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <geometry_msgs/PointStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <queue>
#include <iostream>
#include <interactive_markers/interactive_marker_server.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <geometry_msgs/TransformStamped.h>


using namespace std;
using namespace ros;
using namespace geometry_msgs;

// The published is defined here as it's used in the scan_callback function:
ros::Publisher cmd_vel;

//struct for savning coordinates
struct Vector2D
{
  float x;
  float y;
};



// Variables used in scan_calback:
sensor_msgs::LaserScan laser_msg;
std::vector<float> laser_ranges;
geometry_msgs::Twist velocity_msg;
unsigned int laser_ranges_size = 0;
bool wallDetected = false;
bool wallToTheLeft = false;
bool cornerFound = false;
int counter =0;
Vector2D corners [4];

// Initialize the transform listener
tf2_ros::Buffer tfBuffer;



void corner_saver();



void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
  // The msg is stored in laser_msg:
  laser_msg = *scan_msg;

  // The ranges from the sensor readings in laser_ranges:
  laser_ranges = laser_msg.ranges;
  laser_ranges_size = laser_ranges.size();

  // The min and max sensor readings are found:
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

  // The sensors ranges is divided into three sections: left, center, and right
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

  // The mean of the sections is found:
  laser_ranges_right /= (laser_ranges_size / 3);
  laser_ranges_center /= (laser_ranges_size / 3);
  laser_ranges_left /= (laser_ranges_size / 3);

  // The sections are printed in the terminal:
  ROS_INFO("Left: %f, center: %f, right: %f", laser_ranges_left,
           laser_ranges_center, laser_ranges_right);

  // The code inside the if-statement will only be run if a corner has NOT been found:
  if (cornerFound != true)
  {
    // If any of the sensor readings are closer than 0.4 to an obstacle the robot is slowed down
    // It should never fully stop, as this makes it run into an infinite turning loop:
    if (laser_ranges_min < 0.5)
    {
      velocity_msg.linear.x = 0.01;
      wallDetected = true;
    }
    else if (laser_ranges_min < 0.7)
    {
      velocity_msg.linear.x = 0.10;
      if (wallDetected != true)
      {
        wallDetected = true;
        // When a wall has been detected the first time, it is determined on what side of the robot the wall is positioned:
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

    else if (laser_ranges_min < 0.9)
    {
      velocity_msg.linear.x = 0.15;
    }

    else
    {
      velocity_msg.linear.x = 0.30;
    }

    // When the position of the wall has been found, the robot will follow the wall:
    if (wallDetected == true)
    {
      if (wallToTheLeft == true)
      {
        if (laser_ranges[laser_ranges_size - 1] < 0.7)
        {
          velocity_msg.angular.z = -0.2;
        }
        else
        {
          velocity_msg.angular.z = 0.2;
        }
        if (laser_ranges[laser_ranges_size-1] > 1.0)
        {
          velocity_msg.linear.x = 0.05;
        }
      }

      else
      {
        if (laser_ranges[0] < 0.7)
        {
          velocity_msg.angular.z = 0.2;
        }
        else
        {
          velocity_msg.angular.z = -0.2;
        }
        if (laser_ranges[0] > 1.0)
        {
          velocity_msg.linear.x = 0.05;
        }
      }
    }

    // This is used to determine when a corner has been found:
    // The idea is to use this to initialize the goal marking process:
    if (wallDetected == true && (laser_ranges_left > laser_ranges_center ||
      laser_ranges_right > laser_ranges_center))
      {
        if(laser_ranges_center<0.5)
        {
          cornerFound = true;
        }
      }
    }

  if (cornerFound == true)
  {
corner_saver();

    ROS_WARN("I FOUND A CORNER: %d",counter);
    cornerFound = false;
//    velocity_msg.linear.x = 0;
//    velocity_msg.angular.z = 0;
    //add corner to an array
    //when four corners are found, publish coordinates
  }

  cmd_vel.publish(velocity_msg);
}


void corner_saver(){

 geometry_msgs::TransformStamped transformStamped;

 // trying to obtain coordinates, if it fails it will just send a warning and wait a second.
  try
  {
    transformStamped = tfBuffer.lookupTransform( "map","base_footprint",
                                                ros::Time(0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();

  }

  //adding coordinates in the struct
  Vector2D tmp;
  tmp.x = fabs(transformStamped.transform.translation.x);
  tmp.y = fabs(transformStamped.transform.translation.y);

  //assigning place in the coordinate
  corners[counter]= tmp;
  counter++;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_base");

  // Node handler
  ros::NodeHandle n;
  tf2_ros::TransformListener tfListener(tfBuffer);

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
