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
#include <time.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float64MultiArray.h>


using namespace std;
using namespace ros;
using namespace geometry_msgs;


//array for x and y coordinates
double x_cord [4];
double y_cord [4];

void points(const std_msgs::Float64MultiArray::ConstPtr& tmp_array)
{
  int i = 0;
  int x = 0;
  int y = 0;
	// print all the remaining numbers
	for(std::vector<double>::const_iterator it = tmp_array->data.begin(); it != tmp_array->data.end(); ++it)
	{
    if ( i % 2 == 1 )
    {
      y_cord[y] = *it;
      y++;
    }
    else
    {
      x_cord[x] = *it;
      x++;
    }
		
		i++;
	}

  for ( int i = 0; i < 4; i++)
  {
    ROS_INFO("X: %f Y: %f", x_cord[i], y_cord[i]);
  }

	return;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "path_maker");

  // Node handler
  ros::NodeHandle n;

  // Subscriber
  ros::Subscriber cleaning = n.subscribe("/cleaning_points", 1, points);


  while (ros::ok())
  {
    ros::spinOnce();
  }
  return 0;
}
