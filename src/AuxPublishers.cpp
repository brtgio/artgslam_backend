#include "AuxPublishers.hpp"

AuxPublishers::AuxPublishers(ros::NodeHandle &nh)
: nh(nh)
{
    theta_pub = nh.advertise<std_msgs::Float32>("thetaRewiew/theta_topic", 10);
        thetaRamp_pub = nh.advertise<std_msgs::Float32>("thetaRewiew/thetaRamp_topic", 10);
        thetaCorrect_pub = nh.advertise<std_msgs::Float32>("thetaRewiew/thetaCorrect_topic", 10);
}

void AuxPublishers::setThetaAux_pub(float theta)
{
    std_msgs::Float32 msg;
        msg.data = theta;
        theta_pub.publish(msg);
}

void AuxPublishers::setThetaRamp(float thetaRamp)
{
            std_msgs::Float32 msg;
        msg.data = thetaRamp;
        thetaRamp_pub.publish(msg);
}

void AuxPublishers::thetaCorrect(float thetaCorrect)
{
            std_msgs::Float32 msg;
        msg.data = thetaCorrect;
        thetaCorrect_pub.publish(msg);
}
