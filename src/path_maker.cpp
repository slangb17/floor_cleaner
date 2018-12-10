#include "ros/ros.h"
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float64MultiArray.h>
#include <math.h>
#include <iostream>
#include <string>
#include <iterator>

// Action

#include <algorithm>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <move_base_msgs/MoveBaseAction.h>

//Markers

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Transform System

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

using namespace std;
using namespace visualization_msgs;
using namespace ros;

class Route
{
private:
  Publisher marker_pub;
  Subscriber cleaning;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client;

  //array for x and y coordinates
  vector<vector<double> > point_cords;

  vector<vector<double> > side1;
  vector<vector<double> > side2;

  int side = 2;
  int A = 0;
  int B = 0;

  void _send_goal(vector< double > side)
  {
      move_base_msgs::MoveBaseGoal goal;
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.pose.position.x = side[0];
      goal.target_pose.pose.position.y = side[1];
      goal.target_pose.pose.position.z = 0;
      goal.target_pose.pose.orientation.w = 1.0;
      client.sendGoal(goal, boost::bind(&Route::_target_reached_cb, this, _1, _2)  );
  }

  void _target_reached_cb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result)
  {
    if (side == 1)
    {
      if (A == B)
      {
        ROS_INFO("Going to B: %d", B);
        _send_goal(side2[B]);
        B++;
      }
      else
      {
        _send_goal(side2[B]);
        A++;
        side = 2;
      }
    }
    else
    {
      if (A == B)
      {
         ROS_INFO("Going to A: %d", A);
        _send_goal(side1[A]);
        A++;
      }
      else
      {
        _send_goal(side1[A]);
        B++;
        side = 1;
      }
    }

    //Checking to make sure, that we dont go over the array sizes.
    if (A >= side1.size())
    {
      A = side1.size() - 1;
    }
    else if (B >= side2.size())
    {
      B = side2.size() - 1;
    }
  }

#pragma region Calleback function for getting points

  void points(const std_msgs::Float64MultiArray::ConstPtr &tmp_array)
  {
    int i = 0;
    // print all the remaining numbers
    vector<double> tmp_vector;
    for (std::vector<double>::const_iterator it = tmp_array->data.begin(); it != tmp_array->data.end(); ++it)
    {
      if (i > 1) //0 nothing, 1 nothing, 2 we push it into the vector, and clear the temp vector, last vector never gets pushed.
      {
        point_cords.push_back(tmp_vector);
        tmp_vector.clear();
        i = 0;
      }
      tmp_vector.push_back(*it);
      i++;
    }
    point_cords.push_back(tmp_vector); //The last cordinate is never pushed, if this isnt here.

    for (int i = 0; i < point_cords.size(); i++)
    {
      vector<double> temp_vector = point_cords[i];
      ROS_INFO("X: %f Y: %f", temp_vector[0], temp_vector[1]);
    }

    side1 = vector_point_maker(1, 0);
    side2 = vector_point_maker(2, 3);

    vector< vector< double > > tmpArray = side1;
    tmpArray.insert(tmpArray.end(), side2.begin(), side2.end());

    send_markers(tmpArray);
    _send_goal(side2[0]);

    return;
  }

#pragma endregion
#pragma region Vector Magic

  double vector_length(double x, double y) //finds the length of the vector between two points.
  {
    return sqrt( ( pow(x, 2) + pow(y, 2) ) ) ;
  }

  vector<vector<double> > vector_point_maker(int vector_point_to, int vector_point_from)
  {
    //Point 1 to 2, and Point 4 to 3.
    float TBDia = 0.6;                    //Diameter of turtlebot is used to calculate the distance between points. Meters.
    vector<vector<double> > return_vector; //The return vector, with vectors. (Its a variable array with vector cordinats inside)

    vector<double> point_to = point_cords[vector_point_to];
    vector<double> point_from = point_cords[vector_point_from];

    double x = point_to[0] - point_from[0];
    double y = point_to[1] - point_from[1];

    double vector_len = vector_length(x, y);
    int amount_of_spaces = (vector_len / TBDia) - 1;

    double x_space_length = x / amount_of_spaces;
    double y_space_length = y / amount_of_spaces;

    ROS_WARN("Doing %f: %f, %f, %d ", x, y, vector_len, amount_of_spaces);

    for (int i = 0; i <= amount_of_spaces; i++)
    {
      vector<double> add_distance;
      add_distance.push_back(point_from[0] + x_space_length * i);
      add_distance.push_back(point_from[1] + y_space_length * i);
      return_vector.push_back(add_distance);
    }

    return return_vector;
  }

#pragma endregion
#pragma region Markers

  void send_markers(vector<vector<double> > points) //Sets the markers on the map.
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

public:
  Route() : client("move_base")
  {
    ros::NodeHandle n;
    marker_pub = n.advertise<visualization_msgs::MarkerArray>("busroute_markers", 1);
    cleaning = n.subscribe("/cleaning_points", 1, &Route::points, this);
  };
  ~Route(){};
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "path_maker");

  Route newObject;

  while (ros::ok())
  {
    ros::spinOnce();
  }
  return 0;
}