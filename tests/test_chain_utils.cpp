#include <ros/ros.h>
#include <rtt_ros_kdl_tools/chain_utils.hpp>

int main(int argc, char** argv){
  ros::init(argc, argv, "test_chain_utils");
  ros::NodeHandle nh, nh_priv("~");
  
  rtt_ros_kdl_tools::ChainUtils chain_utils;
  
  return 0;
}
