#pragma once
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
#include "AuxPublishers.hpp"


// Clase principal
class argtSLAM
{
public:
    argtSLAM();

private:
    AuxPublishers auxpub;

    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);
    void sonarChatterCallback(const sensor_msgs::PointCloud::ConstPtr &msg);
    void odomchatterCallback(const nav_msgs::Odometry::ConstPtr &msg);

    // Node stuff
    ros::NodeHandle nh;
    ros::Publisher vel_pub_, sonarPoincloud_filter, sonarPoincloudraw;
    ros::Publisher sonarFilterdata_bag, sonarRawdata_bag, map_pub_, yawPub;
    ros::Subscriber joy_sub_, sonar_sub_, sub_odom;

    // Robot state
    double yaw = 0.0;
    float x = 0.0f, y = 0.0f, theta = 0.0f;

    // FIR filter buffers
    float samplesSonar_x[8][41];
    float samplesSonar_y[8][41];

    // For odometry correction

        //auxiliary variables
    float theta_previus = 0;
    float theta_now = 0;
    float completed_rotations = 0;
    const double corFactor_ = 1.025;

    //joystick parameters  int linear_, angular_;
    double l_scale_, a_scale_,linear_,angular_;

    // Messages
    sensor_msgs::PointCloud pointCLoudVector_filter, pointCLoudVector_raw;
    geometry_msgs::Point32 sonarRaw, sonarFilter;
    nav_msgs::OccupancyGrid occupancy_grid_;
    std_msgs::Float32 yawAmigobot;

    // Grid map
    std::vector<std::vector<int>> gridMap;

    // Helpers
    void firFilter(int numSonar, float x, float y, float *filter_xk, float *filter_yk);
    void xy2Grid(const double& x, const double& y, int& xGrid, int& yGrid);
    void fillGrid(int xGrid, int yGrid);
    void publishMap();
    void initSampleSonar();
};


