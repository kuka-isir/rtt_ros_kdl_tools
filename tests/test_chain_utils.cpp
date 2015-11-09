#include <ros/ros.h>
#include <rtt_ros_kdl_tools/chain_utils.hpp>

int main(int argc, char** argv){
  ros::init(argc, argv, "test_chain_utils");
  ros::NodeHandle nh, nh_priv("~");
  
  rtt_ros_kdl_tools::ChainUtils chain_utils;
  
  std::cout<< "Chain has "<<chain_utils.kdl_chain_.getNrOfJoints()<<" joints"<<std::endl;
  
  std::vector<double> joint_poses;
  joint_poses.push_back(0.0);
  joint_poses.push_back(0.0);
  joint_poses.push_back(0.0);
  joint_poses.push_back(0.0);
  joint_poses.push_back(0.0);
  joint_poses.push_back(0.0);
  joint_poses.push_back(0.0);
  chain_utils.setJointPosition(joint_poses);
  
  std::vector<double> joint_vels;
  joint_vels.push_back(0.0);
  joint_vels.push_back(0.1);
  joint_vels.push_back(0.2);
  joint_vels.push_back(0.3);
  joint_vels.push_back(0.4);
  joint_vels.push_back(0.5);
  joint_vels.push_back(0.6);
  chain_utils.setJointVelocity(joint_vels);
  
  KDL::JntArray jnt_array_pose, jnt_array_vel;
  chain_utils.getJointPositions(jnt_array_pose);
  chain_utils.getJointVelocities(jnt_array_vel);
  for (int i=0; i<chain_utils.kdl_chain_.getNrOfJoints(); i++){
    std::cout<< "Jnt nb "<< i<< " has pose "<<jnt_array_pose(i)<<std::endl;
    std::cout<< "Jnt nb "<< i<< " has vel "<<jnt_array_vel(i)<<std::endl;
  }
  
  KDL::Frame kdl_frame;
  chain_utils.getSegmentPosition(9, kdl_frame);
  std::cout<<"Segment 2 pose ["<<kdl_frame.p.x()<<","<<kdl_frame.p.y()<<","<<kdl_frame.p.z()<<"]"<<std::endl;
  
  return 0;
}
