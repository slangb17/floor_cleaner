// Pure Pursuit Boilerplate Exercise
//
// Copyright (c) 2018 Karl D. Hansen
// MIT License - see LICENSE file.

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <algorithm>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64MultiArray.h>

using namespace std;
using namespace ros;
using namespace geometry_msgs;

ros::Publisher marker_pub;
ros::Publisher cleaning;
ros::Subscriber click_sub;

vector< vector< float > > points;

#pragma region Markers
void send_markers(vector<vector<float> > points) //Sets the markers on the map.
{
  ROS_WARN("Sending Markers");
  int markers = points.size();
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.ns = "bus_stops";
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 1.0;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0.7071;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 0.7071;
  marker.lifetime = ros::Duration();

  visualization_msgs::MarkerArray marker_array;
  for (int i = 0; i < markers; i++)
  {
    marker.header.frame_id = "map";
    marker.id = i;
    marker.pose.position.x = points[i][0];
    marker.pose.position.y = points[i][1];
    marker.pose.position.z = marker.scale.x;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker_array.markers.push_back(marker);
  }
  marker_pub.publish(marker_array);
}
#pragma endregion

// Callback function to handle clicked points in RViz
// When more than two points have been clicked, the "to" point changes to the "from" point.
void clickCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
  ROS_INFO("Clicked: %f, %f, %f", msg->point.x, msg->point.y, msg->point.z);
  vector< float > tmp;
  tmp.push_back(msg->point.x);
  tmp.push_back(msg->point.y);

  points.push_back(tmp);

  if (points.size() > 3)
  {
    send_markers(points);

    std_msgs::Float64MultiArray tmp_array;
    //everything in the array is set to 0
    tmp_array.data.clear();

    //counter to go throught the arrays
    for (int i = 0; i < points.size(); i++)
    {
      //using the push_back function to push first x than y to the array
      tmp_array.data.push_back(points[i][0]);
      tmp_array.data.push_back(points[i][0]);
    }
    //publish the array
    cleaning.publish(tmp_array);

    shutdown();
  }
  
}

int main(int argc, char **argv)
{

  // Initialize the ROS node
  ros::init(argc, argv, "node_maker");
  ros::NodeHandle node;


  cleaning = node.advertise<std_msgs::Float64MultiArray>("/cleaning_points", 100);
  marker_pub = node.advertise<visualization_msgs::MarkerArray>("busroute_markers", 1);

  // Subscribe to the clicked point from RViz
  click_sub = node.subscribe("clicked_point", 1000, clickCallback);

  // The business code
  // - Lookup the position of the robot in the map frame.
  // - Compute the lookahead point.
  // - Find the coresponding linear and angular speeds.
  // - Publish them.
  ros::Rate rate(10.0);
  while (node.ok())
  {
    // Execute any callbacks
    ros::spinOnce();

    rate.sleep();
  }
  return 0;
};
