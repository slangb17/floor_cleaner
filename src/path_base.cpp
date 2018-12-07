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

#define Percent_Deviation 0.05
#define Distance_Wall 0.40
#define Per_slop 0.2
#define End_Distance 0.5
#define Time_Points 2

using namespace std;
using namespace ros;
using namespace geometry_msgs;

// The published is defined here as it's used in the scan_callback function:
ros::Publisher cmd_vel;
ros::Publisher cleaning;
ros::Publisher marker_pub;

// Variables used in scan_callback:
sensor_msgs::LaserScan laser_msg;
geometry_msgs::Twist velocity_msg;

int index_right_unix = 0;
vector< vector< float > > x_y_cords;

clock_t old_time = 0;
clock_t old_time2 = 0;

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
float angleFromScanArray(vector<vector<float> > );
vector< float > currentPos();
vector< vector< float > > dirVectorsPoints(vector< vector< float > >);
vector<vector< float > > intersections (vector< vector< float > >);

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
  if (scan_msg == NULL || laser_ranges_size < 0.01) return;

  //The sensors ranges is divided into three sections: left, center, and right
  //These vectors containes a vector with two variables: Length, 0; Angle, 1
  vector<vector<float> > lr_pol_left;
  vector<vector<float> > lr_pol_center;
  vector<vector<float> > lr_pol_right;
  //ROS_INFO("Scan_Szie %d", laser_ranges_size);
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
    if (i < laser_ranges_size * 1.0 / 3.0) //Lower 1/3 is right side.
    {
      lr_pol_right.push_back(tmp_vec);
    }
    else if (i < laser_ranges_size * 2.0 / 3.0)
    {
      lr_pol_center.push_back(tmp_vec);
    }
    else //Upper 1/3 of array is left side
    {
      lr_pol_left.push_back(tmp_vec);
    }
  }
  //ROS_INFO("lr_pol_right %d", lr_pol_right.size());
  vector<vector<float> > lr_sqr_right = polarToSquare(lr_pol_right);
  //send_markers(lr_sqr_right);
  if (lr_pol_right.size() != 0) 
  {
    vector<vector<float> > dir_vectors = dirVectors(lr_sqr_right);
    //ROS_INFO("dir_vectors_size %d", dir_vectors.size());
    vector<vector< float > > dir_vectors2 = mediaVectors(dir_vectors);
    //ROS_INFO("dir_vectors_size2 %d", dir_vectors2.size());
    vector<float> averageVector = averageVectorFun(dir_vectors2);
    //ROS_INFO("averageVector %d", averageVector.size());
    //ROS_INFO("debug2");

    vector<float> temps = lr_sqr_right[index_right_unix];
    //ROS_INFO("debug3");

    float TBAngle = 0.0;
    float TBSpeed = 0.2;
    float dw = fabs(temps[1]);
    //ROS_INFO("Distance: %f", dw);

    TBAngle = atan2(averageVector[1], averageVector[0]);
    //ROS_INFO("%f < dw: %f < Distance: %f", Distance_Wall * (1.0 - Percent_Deviation), dw, Distance_Wall * (1.0 + Percent_Deviation) );
    if (dw > Distance_Wall * (1.0 + Percent_Deviation)) // ser om tb er for tæt på væggen og retter kursen 
    {
      TBAngle = TBAngle - 0.2;
      //ROS_WARN("wall to far");
    }
    else if (dw < Distance_Wall * (1.0 - Percent_Deviation)) // ser om tb er for langt væk fra væggen og retter kursen 
    {
      TBAngle = TBAngle + 0.2;
      //ROS_WARN("wall to close");
    }
    else if (Distance_Wall * (1.0 + Percent_Deviation) > dw > Distance_Wall * (1.0 - Percent_Deviation))
    {
      if (fabs(TBAngle) < 0.3)
      {
        corner_saver();
      }
    }

    velocity_msg.angular.z = TBAngle;
    velocity_msg.linear.x = TBSpeed;
    cmd_vel.publish(velocity_msg);
  }
  //Going from polar cordinats to square cordinats.
  //Makes directions and a lenghts into a 2D coordinate system
  //ROS_INFO("Debug: 1");

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
  //Sizes is a 2D array, which has the value first and the number of hits second.
  //A array that sorts out based on the standard diviation.
  list<vector<float> > sizes;
  vector<float> tempFloat;
  tempFloat.push_back(a[0]);
  tempFloat.push_back(0);
  sizes.push_back(tempFloat);
  for ( int i = 0; i < size; i++)
  {
    vector<float> tempVector;
    bool add = true;
    for (list<vector<float> >::iterator it=sizes.begin(); it != sizes.end(); ++it)
    {
      vector<float> index_data = (*it);
      if ( index_data[0]*(1.0 + Percent_Deviation) > a[i] > index_data[0]*(1.0 - Percent_Deviation ) )
      {
        index_data[1] += 1.0;
        add = false;
      }
    }
    if (add)
    {
      tempVector.push_back(a[i]);
      tempVector.push_back(1);
      sizes.push_back(tempVector);
    }
  }
  //Finds the number with the biggest amount of hits and assigns it to sorting number.
  float max = 0.0;
  float sortingNumber = 0;
  for (list<vector<float> >::iterator it=sizes.begin(); it != sizes.end(); ++it)
  {
    vector<float> index_data = (*it);
    //ROS_INFO("inside: %f", index_data[1]);
    if (index_data[1] > max)
    {
      sortingNumber = index_data[0];
      max = index_data[1];
    }
  }
  
  // finds the median and uses it to define an area where the values are acceptable
  //int median = size*(1.0/2.0)-1;
  //float sortingNumber = a[ median ];
  float sortingMin = sortingNumber * ( 1.0 - Percent_Deviation);
  float sortingMax = sortingNumber * ( 1.0 + Percent_Deviation);
  //ROS_INFO("Min: %f, Max: %f, SortingNumber: %f", sortingMin, sortingMax, sortingNumber);
  vector<vector<float> > returnVector;
  // creates a new vectorvector that sorts the acceptable from the outliers
  for(int i=0; i < size; i++)
  {
    if (sortingMax > a[i] > sortingMin)
    {
      //This function makes the first vector in the vector a index of where the first valid point is.
      if (returnVector.size() == 0)
      {
        index_right_unix = i;
      }
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
  if (clock() - old_time < Time_Points * 1000000)
  {
    return;
  }
  old_time = clock();

  
  //ROS_INFO("Time: %ld", old_time);
  // trying to obtain coordinates, if it fails it will just send a warning and wait a second.
  ROS_WARN("Point Saved");
  x_y_cords.push_back( currentPos() );
}

void checkLap()
{
  if (x_y_cords.size() == 0) return;
  vector< float > startPos = x_y_cords[0];
  float startX = startPos[0];
  float startY = startPos[1];
  vector<float> tempPos = currentPos();
  float curX = tempPos[0];
  float curY = tempPos[1];
  float distBetween = sqrt(pow(startX-curX,2) + pow(startY-curY,2));
  if (distBetween < End_Distance && clock() - old_time2 > 10000000)
  {
    ROS_WARN("Distance: %f", distBetween);
    dirVectorsPoints (x_y_cords);  
    ros::shutdown();
  }
}

vector< float > currentPos()
{
  geometry_msgs::TransformStamped transformStamped;
   while (true)
  {
    try
    {
      transformStamped = tfBuffer.lookupTransform("map", "base_footprint", ros::Time(0));
      break;
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
  }

  //adding coordinates in the x and y array
  vector< float > TBMarkers;
  TBMarkers.push_back(fabs(transformStamped.transform.translation.x));
  TBMarkers.push_back(fabs(transformStamped.transform.translation.y));
  return TBMarkers;
}

//Takes a point matrix.
vector< vector< float > > dirVectorsPoints(vector< vector< float > > input)
{
  if (input.size() == 0) return vector< vector< float > >();
  vector< vector< float > > dir_vectors;
  dir_vectors = dirVectors(input);

  int size = dir_vectors.size();
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
  
  list<vector<float> > sizes;
  vector<float> tempFloat;
  tempFloat.push_back(a[0]);
  tempFloat.push_back(0);
  sizes.push_back(tempFloat);
  // catagorises vectors into groups with similar slopes
  for ( int i = 0; i < size; i++)
  {
    vector<float> tempVector;
    bool add = true;
    for (list<vector<float> >::iterator it=sizes.begin(); it != sizes.end(); ++it)
    {
      vector<float> index_data = (*it);
      if ( index_data[0]*(1.0 + Percent_Deviation) > a[i] > index_data[0]*(1.0 - Percent_Deviation ) )
      {
        index_data[1] += 1.0;
        add = false;
      }
    }
    if (add)
    {
      tempVector.push_back(a[i]);
      tempVector.push_back(1);
      sizes.push_back(tempVector);
    }
  }
  //Finds the number with the biggest amount of hits and assigns it to sorting number.
  //float max[4] = {0.0};
  float sortingNumber[4] = {0.0};
  /*for (list<vector<float> >::iterator it=sizes.begin(); it != sizes.end(); ++it)
  {
    vector<float> index_data = (*it);
    for(int i = 0; i < 4; i++)
    {
      if (max[i] < index_data[1])
      {
        max[i] = index_data[1];
        sortingNumber[i] = index_data[0];
        break;
      }
    }
  }*/

  sort(sizes.begin(), sizes.end(), [](const std::vector<int>& a, const std::vector<int>& b) { return a[1] < b[1]; });
  for (int i = 0; i < 4; i++)
  {
    vector< float > index_data = sizes.back();
    sizes.pop_back();
    sortingNumber[i] = index_data[0];
  }

  //Return array, only the vectors with the right slop is being keeped. 
  vector< vector< float > > vector_newest;
  vector< vector< float > > point_newest;
  for (int i = 0; i < 4; i++)
  {
    vector<float > tmp;
    tmp.push_back(0);
    tmp.push_back(0);
    vector_newest.push_back(tmp);

    vector<float > tmp2;
    tmp2.push_back(0);
    tmp2.push_back(0);
    point_newest.push_back(tmp2);
  }

  for (int i = 0; i < size; i++)
  {
    for( int j = 0; j < 4; j++)
    {
      float sorting_Max = sortingNumber[j] * (1.0 + Percent_Deviation);
      float sorting_Min = sortingNumber[j] * (1.0 - Percent_Deviation);
      if ( sorting_Max > a[i] > sorting_Min )
      {
        vector_newest[j][0] = vector_newest[j][0] + dir_vectors[i][0];
        vector_newest[j][1] = vector_newest[j][1] + dir_vectors[i][1];
        point_newest[j] = input[i];
      }
    }
  }

  vector<vector< float > > a_b;
  for(int i = 0; i < vector_newest.size(); i++)
  {
    float a = 0.0;
    float b = 0.0;
    if (0.001 > vector_newest[i][0] > -0.001)
    {
      a = vector_newest[i][1];
    }
    else
    {
      a = vector_newest[i][1] / vector_newest[i][0];
    }
    b = point_newest[i][1] - a * point_newest[i][0];

    vector< float > tmp;
    tmp.push_back(a);
    tmp.push_back(b);
    a_b.push_back(tmp);
  }

  vector<vector< float > > intersectionPoints = intersections(a_b);

  send_markers(intersectionPoints);
  return vector< vector < float > >();
}
//Finds the intersections between four given points.
vector<vector< float > > intersections (vector< vector< float > > input)
{
  vector<vector< float > > returnvector;
  for (int i=0; i< input.size(); i++)
  {
    float a = input[i][0];
    float b = input[i][1];
    for (int j = i + 1; j < input.size(); j++)
    {
      float c = input[j][0];
      float d = input[j][1];

      if ((1.0 - Per_slop) * a < c < a * (1.0 + Per_slop))
      {
        continue;
      }
      else
      {
        float x = (d-b)/(a-c);
        float y = a*( (d-b)/(a-c) ) + b;
        
        vector< float > tempReturn;
        tempReturn.push_back(x);
        tempReturn.push_back(y);

        returnvector.push_back(tempReturn);
      }
    }
  }
  return returnvector;
}
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
    if (i == 0)
    {
      marker.color.r = 1.0;
    }
    else
    {
      marker.color.r = 0.0;
    }
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

  // Publisher
  cmd_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 100);
  cleaning = n.advertise<std_msgs::Float64MultiArray>("/cleaning_points", 100);
  marker_pub = n.advertise<visualization_msgs::MarkerArray>("busroute_markers", 1);

  // Subscriber
  ros::Subscriber laser_scan = n.subscribe("/scan", 1000, scan_callback);

  old_time2 = clock();

  while (ros::ok())
  {
    checkLap();
    ros::spinOnce();
  }
  return 0;
}
