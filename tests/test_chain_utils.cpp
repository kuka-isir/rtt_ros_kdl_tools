#include <ros/ros.h>
#include <rtt_ros_kdl_tools/chain_utils.hpp>
#include <gtest/gtest.h>

TEST(TestChainUtils, testGetNbSegments){
  rtt_ros_kdl_tools::ChainUtils chain_utils;
  
  ROS_INFO_STREAM("Chain :"<<chain_utils.kdl_chain_.getNrOfSegments()<<" , ChainUtils :"<< chain_utils.nbSegments() );
  
  EXPECT_TRUE(chain_utils.kdl_chain_.getNrOfSegments() == chain_utils.nbSegments());
}

TEST(TestChainUtils, testSetGetPosition){
  rtt_ros_kdl_tools::ChainUtils chain_utils;
  std::vector<double> joint_poses, q;
  joint_poses.push_back(1.603450059890747);
  joint_poses.push_back(0.3992106020450592);
  joint_poses.push_back(-0.10936897993087769);
  joint_poses.push_back(-1.783982515335083);
  joint_poses.push_back(0.11895296722650528);
  joint_poses.push_back(0.9718001484870911);
  joint_poses.push_back(0.1688366532325744);
  chain_utils.setJointPosition(joint_poses);
  
  KDL::JntArray jnt_array_pose;
  chain_utils.getJointPositions(jnt_array_pose);
 
  for (int i=0; i<chain_utils.kdl_chain_.getNrOfJoints(); i++)
    q.push_back(jnt_array_pose(i));
  
  EXPECT_TRUE(q == joint_poses);
  
  KDL::JntSpaceInertiaMatrix mass_mat;
  chain_utils.getInertiaMatrix(mass_mat);
  
  ROS_ERROR_STREAM("Mass matrix:\n");
  std::cout << mass_mat.data << std::endl;
}

TEST(TestChainUtils, testSetGetVelocity){
  rtt_ros_kdl_tools::ChainUtils chain_utils;
  std::vector<double> joint_vels, qd;
  joint_vels.push_back(1.603450059890747);
  joint_vels.push_back(0.3992106020450592);
  joint_vels.push_back(-0.10936897993087769);
  joint_vels.push_back(-1.783982515335083);
  joint_vels.push_back(0.11895296722650528);
  joint_vels.push_back(0.9718001484870911);
  joint_vels.push_back(0.1688366532325744);
  chain_utils.setJointVelocity(joint_vels);
  
  KDL::JntArray jnt_array_vel;
  chain_utils.getJointVelocities(jnt_array_vel);
 
  for (int i=0; i<chain_utils.kdl_chain_.getNrOfJoints(); i++)
    qd.push_back(jnt_array_vel(i));
  
  EXPECT_TRUE(qd == joint_vels);
  
}


int main(int argc, char** argv){
  ros::init(argc, argv, "test_chain_utils");
  ros::NodeHandle nh, nh_priv("~");
  
  testing::InitGoogleTest(&argc, argv);
  
  
  rtt_ros_kdl_tools::ChainUtils chain_utils;

  
  
//   std::cout<< "Chain has "<<chain_utils.kdl_chain_.getNrOfJoints()<<" joints"<<std::endl;
//   
//   std::vector<double> joint_poses;
//   joint_poses.push_back(1.603450059890747);
//   joint_poses.push_back(0.3992106020450592);
//   joint_poses.push_back(-0.10936897993087769);
//   joint_poses.push_back(-1.783982515335083);
//   joint_poses.push_back(0.11895296722650528);
//   joint_poses.push_back(0.9718001484870911);
//   joint_poses.push_back(0.1688366532325744);
//   chain_utils.setJointPosition(joint_poses);
//   
//   std::vector<double> joint_vels;
//   joint_vels.push_back(0.0);
//   joint_vels.push_back(0.1);
//   joint_vels.push_back(0.2);
//   joint_vels.push_back(0.3);
//   joint_vels.push_back(0.4);
//   joint_vels.push_back(0.5);
//   joint_vels.push_back(0.6);
//   chain_utils.setJointVelocity(joint_vels);
//   
//   KDL::JntArray jnt_array_pose, jnt_array_vel;
//   chain_utils.getJointPositions(jnt_array_pose);
//   chain_utils.getJointVelocities(jnt_array_vel);
//   for (int i=0; i<chain_utils.kdl_chain_.getNrOfJoints(); i++){
//     std::cout<< "Jnt nb "<< i<< " has pose "<<jnt_array_pose(i)<<std::endl;
//     std::cout<< "Jnt nb "<< i<< " has vel "<<jnt_array_vel(i)<<std::endl;
//   }
//   
//   KDL::Frame kdl_frame;
//   chain_utils.getSegmentPosition(8, kdl_frame);
//   std::cout<<"Segment "<<chain_utils.getSegmentName(8) <<"pose ["<<kdl_frame.p.x()<<","<<kdl_frame.p.y()<<","<<kdl_frame.p.z()<<"]"<<std::endl;
//   
  return RUN_ALL_TESTS();
}
