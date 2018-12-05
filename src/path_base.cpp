#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
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
#include <math.h>
#include <algorithm>
#include <tf2/LinearMath/Vector3.h>

using namespace std;
using namespace ros;
using namespace geometry_msgs;

// The published is defined here as it's used in the scan_callback function:
ros::Publisher cmd_vel;
ros::Publisher cleaning;
ros::Publisher marker_pub;

double x_cord[4];
double y_cord[4];

// Variables used in scan_callback:
sensor_msgs::LaserScan laser_msg;
geometry_msgs::Twist velocity_msg;
bool wallDetected = false;
bool wallToTheLeft = false;
bool cornerFound = false;
int counter = 0;

clock_t old_time = 0;

//Initialize the transform listener
tf2_ros::Buffer tfBuffer;

//declaring the function
void corner_saver();
vector<vector<float> > polarToSquare(vector<vector<float> >);
vector<vector<float> > dirVectors(vector<vector<float> >);
vector<vector<float> > mediaVectors(vector<vector<float> >);
vector<float> averageVectorFun(vector<vector<float> >);
void send_markers(vector<vector<float> >);
void send_marker(vector<vector<float> >);

//Prints the entries of an array as a warning
void printFun(vector<vector<float> > input)
{
  for (int i = 0; i < input.size(); i++)
  {
    ROS_WARN("Index: %d, Data_X: %f, Data_y: %f", i, input[i][0], input[i][1]);
  }
}

//Takes all the data from the infrared distance sensor and places them in three arrays
void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
  //The msg is stored in laser_msg:
  laser_msg = *scan_msg;

  //The ranges from the sensor readings in laser_ranges:
  vector<float> laser_ranges = laser_msg.ranges;
  float laser_angle_min = laser_msg.angle_min;
  float laser_angle_inc = laser_msg.angle_increment;
  u_int laser_ranges_size = laser_ranges.size();

  //The sensors ranges is divided into three sections: left, center, and right
  //These vectors containes a vector with two variables: Length, 0; Angle, 1
  vector<vector<float> > lr_pol_left;
  vector<vector<float> > lr_pol_center;
  vector<vector<float> > lr_pol_right;
  
  //A for loop that loops as many times as there are entries in the laser_ranges vector
  for (int i = 0; i < laser_ranges_size; i++)
  {
    vector<float> tmp_vec;
    // if the entry is not a number or infinit it jumps to the next entry in the foor loop ie i++
    if (isnan(laser_ranges[i]) || laser_ranges[i] == NAN || isinf(laser_ranges[i]))
    {
      continue;
    } 
    tmp_vec.push_back(laser_ranges[i]); // takes the entry from laser_range and puts it at the buttom of tmp_vec vector
    tmp_vec.push_back(laser_angle_min + (laser_angle_inc * i)); // takes the laser_angle increment times i, adds it to the minimum laser angle to find the angle of the current entry and puts it at the bottom of tmp_vec vector
    // taking all the lasers angles and placing them in three arrays which is left, right, or center.
    if (i < laser_ranges_size * 1 / 3)
    {
      lr_pol_right.push_back(tmp_vec);
    }
    else if (i < laser_ranges_size * 2 / 3)
    {
      lr_pol_center.push_back(tmp_vec);
    }
    else
    {
      lr_pol_left.push_back(tmp_vec);
    }
  }

  //Going from polar cordinats to square cordinats.
  //Makes directions and a lenghts into a 2D coordinate system   
  vector<vector<float> > lr_sqr_right = polarToSquare(lr_pol_right);
  vector<vector<float> > dir_vectors = dirVectors(lr_sqr_right);
  ROS_WARN("NEW");
  vector<vector< float > > dir_vectors2 = mediaVectors(dir_vectors);

  ROS_INFO("Size: %d", dir_vectors2.size());
  for (int i = 0; i < dir_vectors2.size(); i++)
  {
    //ROS_WARN("Indexs: %d, Data_X: %f, Data_y: %f", i, dir_vectors2[i][1],  dir_vectors[i][1]);
  }

  vector<float> averageVector = averageVectorFun(dir_vectors);
  // TODO: Marker of vector.

  double TBAngle = atan2(averageVector[1], averageVector[0]);

  velocity_msg.angular.z = TBAngle;
  velocity_msg.linear.x = 0.2;

  //send_markers(lr_sqr_right);
  cmd_vel.publish(velocity_msg);
}

//Takes the x and y from all vectors in an array and calculates the slope (a) of the vector and finds the median ...
// ...from that it then picks all the vectors within the a...
vector<vector<float> > mediaVectors(vector<vector<float> > input)
{
  int size = input.size();
  float a[size];  
  //for every vector it finds the slope y/x=a
  for (int i = 0; i < size; i++)
  {
    float x = input[i][0];
    float y = input[i][1];
    if (x < 0.01)
    {
      a[i] = y;
    }
    else
    {
      a[i] = y/x;    
    }
  }
  sort(a, a + size);
  // finds the median and uses it to define an area where the values are acceptable
  int median = size*(1.0/2.0)-1;
  float sortingNumber = a[ median ];
  float sortingMin = sortingNumber * 0.95;
  float sortingMax = sortingNumber * 1.05;
  ROS_INFO("Min: %f, Max: %f, SortingNumber: %f", sortingMin, sortingMax, sortingNumber);
  vector<vector<float> > returnVector;
  // creates a new vectorvector that sorts the acceptable from the outliers
  for(int i=0; i < size; i++)
  {
    if (sortingMax > a[i] > sortingMin)
    {
      returnVector.push_back(input[i]);
    }
  }
  return returnVector;
}

//Takes all entries in the vector vector and makes a array with the average x and y
vector<float> averageVectorFun(vector< vector< float > > input)
{
  float x = 0, y = 0;
  float size = input.size();
  for ( int i = 0; i < size; i++)
  {
    x = x + input[i][0];
    y = y + input[i][1];
  }
  x = x / size;
  y = y / size;

  vector< float > result;
  result.push_back(x);
  result.push_back(y);
  return result;
}

// Takes a vector vector with distance and angels as entries then makes it into x,y coordinates
vector<vector<float> > polarToSquare(vector<vector<float> > input)
{
  vector<vector<float> > return_array;
  for (int i = 0; i < input.size(); i++)
  {
    vector<float> return_vector;
    float distance = input[i][0];
    float angle = input[i][1];
    return_vector.push_back(distance * cos(angle));
    return_vector.push_back(distance * sin(angle));

    return_array.push_back(return_vector);
  }
  return return_array;
}

// directional vector between the points the current and next point is calculated:
vector<vector<float> > dirVectors(vector<vector<float> > input)
{
  vector<vector<float> > return_array;
  for (int i = 0; i < input.size() - 1; i++)
  {
    vector<float> return_vector;
    float x_0 = input[i][0];
    float y_0 = input[i][1];

    float x_1 = input[i + 1][0];
    float y_1 = input[i + 1][1];

    return_vector.push_back(x_1 - x_0);
    return_vector.push_back(y_1 - y_0);

    return_array.push_back(return_vector);
  }
  return return_array;
}

//
void corner_saver()
{
  // checking time since last run, this is to secure the robot does not run this function multiple times in same corner
  if (clock() - old_time < 20000000)
  {
    return;
  }
  old_time = clock();

  geometry_msgs::TransformStamped transformStamped;
  ROS_WARN("Corner Saved: %d", counter);
  ROS_INFO("Time: %ld", old_time);
  // trying to obtain coordinates, if it fails it will just send a warning and wait a second.
  try
  {
    transformStamped = tfBuffer.lookupTransform("map", "base_footprint",
                                                ros::Time(0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

  //adding coordinates in the x and y array
  x_cord[counter] = fabs(transformStamped.transform.translation.x);
  y_cord[counter] = fabs(transformStamped.transform.translation.y);

  //incrementing to assign a place in the array
  counter++;
}
#pragma region Marker
//Sets the markers on the map.
void send_marker(vector< vector< float> > input) 
{
  //ROS_WARN("Sending Markers");
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.ns = "bus_stops";
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.5;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 5;
  marker.pose.orientation.w = 0;
  marker.lifetime = ros::Duration();

  visualization_msgs::MarkerArray marker_array;
  marker.header.frame_id = "base_laser_link";
  marker.id = 1;
  marker.pose.position.x = input[0][0];
  marker.pose.position.y = input[0][1];
  marker.pose.position.z = 0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker_array.markers.push_back(marker);
  marker_pub.publish(marker_array);
}
#pragma endregion
#pragma region Markers
//Sets the markers on the map.
void send_markers(vector<vector<float> > points) 
{
  //ROS_WARN("Sending Markers");
  int markers = points.size();
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.ns = "bus_stops";
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.5;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
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
    marker.header.frame_id = "camera_depth_frame";
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "path_base");

  // Node handler
  ros::NodeHandle n;
  tf2_ros::TransformListener tfListener(tfBuffer);
  //old_time= clock();

  // Publisher
  cmd_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 100);
  cleaning = n.advertise<std_msgs::Float64MultiArray>("/cleaning_points", 100);
  marker_pub = n.advertise<visualization_msgs::MarkerArray>("busroute_markers", 1);

  // Subscriber
  ros::Subscriber laser_scan = n.subscribe("/scan", 1000, scan_callback);

  while (ros::ok())
  {
    ros::spinOnce();
    //checks to see if conter if more than 2. If counter is 3, the robot has obtained 4 coordinatesets for 4 different corners.
    if (counter > 3)
    {
      //initializing an array of the type float64
      std_msgs::Float64MultiArray tmp_array;
      //everything in the array is set to 0
      tmp_array.data.clear();

      //counter to go throught the arrays
      for (int i = 0; i < counter; i++)
      {
        //using the push_back function to push first x than y to the array
        tmp_array.data.push_back(x_cord[i]);
        tmp_array.data.push_back(y_cord[i]);
      }
      //publish the array
      cleaning.publish(tmp_array);
      //shutsdown node
      ros::shutdown();
    }
  }
  return 0;
}
