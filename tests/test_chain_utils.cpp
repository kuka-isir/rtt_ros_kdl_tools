#include <ros/ros.h>
#include <rtt_ros_kdl_tools/chain_utils.hpp>
#include <gtest/gtest.h>

TEST(TestChainUtils, testGetNbSegments){
  rtt_ros_kdl_tools::ChainUtils chain_utils;
  chain_utils.printChain();
  
  EXPECT_TRUE(chain_utils.kdl_chain_.getNrOfSegments() == chain_utils.nbSegments());
}

TEST(TestChainUtils, testSetGetJointPosition){
  rtt_ros_kdl_tools::ChainUtils chain_utils;
  std::vector<double> joint_poses, q;
  joint_poses.push_back(1.6036);
  joint_poses.push_back(0.399208);
  joint_poses.push_back(-0.109438);
  joint_poses.push_back(-1.78409);
  joint_poses.push_back(0.119221);
  joint_poses.push_back(0.972755);
  joint_poses.push_back(0.16883);
  chain_utils.setJointPosition(joint_poses);
  
  KDL::JntArray jnt_array_pose;
  chain_utils.getJointPositions(jnt_array_pose);
 
  for (int i=0; i<chain_utils.kdl_chain_.getNrOfJoints(); i++)
    q.push_back(jnt_array_pose(i));
  
  EXPECT_TRUE(q == joint_poses);
  
}

TEST(TestChainUtils, testGetFrame){
  rtt_ros_kdl_tools::ChainUtils chain_utils;
  std::vector<double> joint_poses, joint_vels;
  joint_poses.push_back(1.6036);
  joint_poses.push_back(0.399208);
  joint_poses.push_back(-0.109438);
  joint_poses.push_back(-1.78409);
  joint_poses.push_back(0.119221);
  joint_poses.push_back(0.972755);
  joint_poses.push_back(0.16883);
  chain_utils.setJointPosition(joint_poses);
  
  joint_vels.push_back(0);
  joint_vels.push_back(0);
  joint_vels.push_back(0);
  joint_vels.push_back(0);
  joint_vels.push_back(0);
  joint_vels.push_back(0);
  joint_vels.push_back(0);
  chain_utils.setJointVelocity(joint_vels);
  
  KDL::Frame frame, real_frame;
  std::string seg_name = "ati_link";
  chain_utils.getSegmentPosition(seg_name, frame);
  
  real_frame.p.data[0] = -0.019;
  real_frame.p.data[1] = -0.473;
  real_frame.p.data[2] = 0.329;
  
  EXPECT_TRUE(KDL::Equal(real_frame.p, frame.p, 0.01));

}

TEST(TestChainUtils, testGetInertiaMatrix){
  rtt_ros_kdl_tools::ChainUtils chain_utils;
  std::vector<double> joint_poses, joint_vels;
  joint_poses.push_back(1.6036);
  joint_poses.push_back(0.399208);
  joint_poses.push_back(-0.109438);
  joint_poses.push_back(-1.78409);
  joint_poses.push_back(0.119221);
  joint_poses.push_back(0.972755);
  joint_poses.push_back(0.16883);
  chain_utils.setJointPosition(joint_poses);
  
  joint_vels.push_back(0);
  joint_vels.push_back(0);
  joint_vels.push_back(0);
  joint_vels.push_back(0);
  joint_vels.push_back(0);
  joint_vels.push_back(0);
  joint_vels.push_back(0);
  chain_utils.setJointVelocity(joint_vels);
  
  KDL::JntSpaceInertiaMatrix mass_mat, real_mass_mat;
  chain_utils.getInertiaMatrix(mass_mat);
  
  real_mass_mat = mass_mat;
  real_mass_mat.data <<  0.982248 ,    0.0285327,    0.686608,     0.0202515,    0.0314535,    0.00392322,   -0.000120107,
		    0.0285327,    1.60483,      0.0576687,    -0.407289,    0.00501679,   -0.0026864,   6.80645e-06, 
		    0.686608,     0.0576687,    0.541618,     0.000733275,  0.0295111,    0.00345672,   -0.000110815,
		    0.0202515,    -0.407289,    0.000733275,  0.577676,     -0.00507489,  -0.029607,    -1.1825e-05, 
		    0.0314535,    0.00501679,   0.0295111,    -0.00507489,  0.0118103,    5.56162e-06,  6.77319e-05, 
		    0.00392322,   -0.0026864,   0.00345672,   -0.029607,    5.56162e-06,  0.010561,     0,           
		    -0.000120107, 6.80645e-06,  -0.000110815, -1.1825e-05,  6.77319e-05,  0,            0.0001203; 

  EXPECT_TRUE(KDL::Equal(real_mass_mat, mass_mat, 0.15));

}

TEST(TestChainUtils, testGetGravityTorque){
  rtt_ros_kdl_tools::ChainUtils chain_utils;
  std::vector<double> joint_poses, joint_vels;
  joint_poses.push_back(1.6036);
  joint_poses.push_back(0.399208);
  joint_poses.push_back(-0.109438);
  joint_poses.push_back(-1.78409);
  joint_poses.push_back(0.119221);
  joint_poses.push_back(0.972755);
  joint_poses.push_back(0.16883);
  chain_utils.setJointPosition(joint_poses);
  
  joint_vels.push_back(0);
  joint_vels.push_back(0);
  joint_vels.push_back(0);
  joint_vels.push_back(0);
  joint_vels.push_back(0);
  joint_vels.push_back(0);
  joint_vels.push_back(0);
  chain_utils.setJointVelocity(joint_vels);
  
  KDL::JntArray gravity_torque, real_gravity_torque;
  chain_utils.getGravityTorque(gravity_torque);
  
  Eigen::VectorXd values(7);
  values(0) = -1.79e-17; values(1) = -27.09; values(2) = -0.7907; values(3) = 12.9446; values(4) = -0.208539; values(5) = 0.00878; values(6) = 0.00035;
  real_gravity_torque = gravity_torque;
  real_gravity_torque.data = values;
 
  EXPECT_TRUE(KDL::Equal(real_gravity_torque, gravity_torque, 0.15));
  
}

TEST(TestChainUtils, testGetCoriolisTorque){
  rtt_ros_kdl_tools::ChainUtils chain_utils;
  std::vector<double> joint_poses, joint_vels;
  joint_poses.push_back(1.6036);
  joint_poses.push_back(0.399208);
  joint_poses.push_back(-0.109438);
  joint_poses.push_back(-1.78409);
  joint_poses.push_back(0.119221);
  joint_poses.push_back(0.972755);
  joint_poses.push_back(0.16883);
  chain_utils.setJointPosition(joint_poses);
  
  joint_vels.push_back(0);
  joint_vels.push_back(0);
  joint_vels.push_back(0);
  joint_vels.push_back(0);
  joint_vels.push_back(0);
  joint_vels.push_back(0);
  joint_vels.push_back(0);
  chain_utils.setJointVelocity(joint_vels);
  
  KDL::JntArray coriolis_torque, real_coriolis_torque;
  chain_utils.getCoriolisTorque(coriolis_torque);
  
  real_coriolis_torque = coriolis_torque;
  real_coriolis_torque.data.setZero();
 
  EXPECT_TRUE(KDL::Equal(real_coriolis_torque, coriolis_torque, 0.15));
  
}

TEST(TestChainUtils, testSetGetJointVelocity){
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
  
  return RUN_ALL_TESTS();
}
