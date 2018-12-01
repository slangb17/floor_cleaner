#include "ros/ros.h"
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float64MultiArray.h>
#include <math.h>
#include <iostream>
#include <string>

// Action

#include <algorithm>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <move_base_msgs/MoveBaseAction.h>

//Markers

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Interactive Markers

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

// Transform System

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#define expectedArraySize 4

using namespace std;
using namespace visualization_msgs;

//array for x and y coordinates
double x_cord[expectedArraySize];
double y_cord[expectedArraySize];

//Interactive marker shit
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;
void makeButtonMarker(std::string frameId);
void frameCallback(const ros::TimerEvent &);
void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

//Functions
void send_markers(); //Makes markers wherever the x and y cords are.

void points(const std_msgs::Float64MultiArray::ConstPtr &tmp_array)
{
  int i = 0;
  int x = 0;
  int y = 0;
  // print all the remaining numbers
  for (std::vector<double>::const_iterator it = tmp_array->data.begin(); it != tmp_array->data.end(); ++it)
  {
    if (i % 2 == 1)
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

  for (int i = 0; i < expectedArraySize; i++)
  {
    ROS_INFO("X: %f Y: %f", x_cord[i], y_cord[i]);
  }

  makeButtonMarker("map");

  return;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "path_maker");

  // Node handler
  ros::NodeHandle n;

  // Subscriber
  ros::Subscriber cleaning = n.subscribe("/cleaning_points", 1, points);

  // create a timer to update the published transforms
  ros::Timer frame_timer = n.createTimer(ros::Duration(0.01), frameCallback);

  server.reset(new interactive_markers::InteractiveMarkerServer("basic_controls", "", false));

  while (ros::ok())
  {
    ros::spinOnce();
  }
  return 0;
}

//http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers%3A%20Basic%20Controls Incase you feel frisky.

void makeButtonMarker(std::string frameId)
{

  for (int i = 0; i < expectedArraySize; i++)
  {
    InteractiveMarker int_marker;
    InteractiveMarkerControl control;
    tf::Vector3 position = tf::Vector3(x_cord[i], y_cord[i], 2);
    int_marker.header.frame_id = frameId;
    tf::pointTFToMsg(position, int_marker.pose.position);
    int_marker.scale = 1;

    int_marker.name = (char)i;
    int_marker.description = "Button\n(Left Click)";

    control.interaction_mode = InteractiveMarkerControl::BUTTON;
    control.name = "button_control";
    Marker marker;

    marker.type = Marker::ARROW;
    marker.scale.x = int_marker.scale * 2;
    marker.scale.y = int_marker.scale * 0.45;
    marker.scale.z = int_marker.scale * 0.45;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0.7071;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 0.7071;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 1.0;
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = i;
    control.markers.push_back(marker);
    control.always_visible = true;
    int_marker.controls.push_back(control);

    server->insert(int_marker);
    server->setCallback(int_marker.name, &processFeedback);
  }

  server->applyChanges();
}

void frameCallback(const ros::TimerEvent &)
{
  static uint32_t counter = 0;

  static tf::TransformBroadcaster br;

  tf::Transform t;

  ros::Time time = ros::Time::now();

  t.setOrigin(tf::Vector3(0.0, 0.0, sin(float(counter) / 140.0) * 2.0));
  t.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
  br.sendTransform(tf::StampedTransform(t, time, "base_link", "moving_frame"));

  t.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  t.setRotation(tf::createQuaternionFromRPY(0.0, float(counter) / 140.0, 0.0));
  br.sendTransform(tf::StampedTransform(t, time, "base_link", "rotating_frame"));

  counter++;
}

void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
    << " / control '" << feedback->control_name << "'";

  switch (feedback->event_type)
  {
  case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
    ROS_INFO_STREAM(s.str() << ": button click"
                            << ".");

    int idNumber = (int)( feedback->marker_name )[0];
    ROS_WARN("Id: %d", idNumber);

    break;
  }
  server->applyChanges();
}
