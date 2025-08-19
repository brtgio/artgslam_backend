#pragma once
#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/Float32.h>

/*This class will work only to publish data in auxiliary form only for debuging backend , ist not allow to add subcribers or data processing fuctions, only setters
fuctions are allow, this should be keept as simple as posible.
*/

class AuxPublishers{

    public:
    AuxPublishers(ros::NodeHandle& nh);


    void setThetaAux_pub(float theta);
    void setThetaRamp(float thetaRamp);
    void thetaCorrect(float thetaCorrect);

    private:

    //node handler 
    ros::NodeHandle& nh;
    //publishers
   
    ros::Publisher theta_pub;
    ros::Publisher thetaRamp_pub;
    ros::Publisher thetaCorrect_pub;


};
