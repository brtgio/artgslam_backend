#ifndef ARGTSLAM_HPP
#define ARGTSLAM_HPP
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <string>
#include <vector>
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "Myconstants.hpp"
#include <iostream>
#include <cmath>
#include <nav_msgs/OccupancyGrid.h>
#include<array>
#include <std_msgs/Float32.h>


// Clase principal
class argtSLAM
{
public:
  argtSLAM();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);
  void sonarChatterCallback(const sensor_msgs::PointCloud::ConstPtr &msg);
  static void odomchatterCallback(const nav_msgs::Odometry::ConstPtr &msg);

  ros::NodeHandle nh_;
  int linear_, angular_;
  double l_scale_, a_scale_;

  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber sonar_sub_;
  ros::Subscriber sub_odom;
  ros::Publisher sonarPoincloud_filter;
  ros::Publisher sonarPoincloudraw;
  ros::Publisher sonarFilterdata_bag;
  ros::Publisher sonarRawdata_bag;
  ros::Publisher map_pub_;  // Publisher para el mapa


  sensor_msgs::PointCloud pointCLoudVector_filter;
  sensor_msgs::PointCloud pointCLoudVector_raw;
  geometry_msgs::Point32 sonarRaw;
  geometry_msgs::Point32 sonarFilter;
  nav_msgs::OccupancyGrid occupancy_grid_;


  // Data Processing functions
  void firFilter(int i,float x, float y, float *filter_xk, float *filter_yk);

  // Variables for correcting angular measurements
  static double theta_ant;
  static double theta_act;
  static int numRotation;
  static const double corFactor; // Ramp correction factor (can stay initialized here)
  
  //Variables for making a grid map
   std::vector<std::vector<int>> gridMap;
  void xy2Grid(const double& x, const double& y, int& xGrid, int& yGrid);
  void fillGrid(int xGrid, int yGrid);
  void publishMap();
  void initSampleSonar();

};

#endif // ARGTSLAM_HPP



