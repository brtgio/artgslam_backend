#include "argtSlam.hpp"
int main(int argc, char **argv)
{
  ros::init(argc, argv, "argt_slam_node");  // Nombre más descriptivo y coherente

  argtSLAM slam_node;

  
  ros::spin();
  return 0;
}

