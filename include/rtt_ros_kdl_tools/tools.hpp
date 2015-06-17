#ifndef __RTT_ROS_KDL_TOOLS_HPP
#define __RTT_ROS_KDL_TOOLS_HPP

#include <rtt/RTT.hpp>
#include <rtt/TaskContext.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/graph/graph_concepts.hpp>

#include <kdl/chainiksolvervel_pinv_nso.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainidsolver_vereshchagin.hpp>
#include <kdl/chaindynparam.hpp>

#include <kdl/frames.hpp>
#include <kdl/framevel.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/segment.hpp>
#include <kdl/chain.hpp>

#include <kdl_parser/kdl_parser.hpp>

#include <urdf/model.h>

#include <rtt_rosparam/rosparam.h>
#include <sensor_msgs/JointState.h>

namespace rtt_ros_kdl_tools{
    
    bool initChainFromString(const std::string& robot_description,
                             const std::string& root_link,
                             const std::string& tip_link,
                             KDL::Tree& kdl_tree,
                             KDL::Chain& kdl_chain);
    
    bool initChainFromROSParamURDF(RTT::TaskContext* task, 
                                   const std::string& root_link, 
                                   const std::string& tip_link, 
                                   KDL::Tree& kdl_tree, 
                                   KDL::Chain& kdl_chain, 
                                   const std::string& ros_name = "robot_description", 
                                   const std::string& rtt_name = "robot_description");
    
    void initJointStateFromKDLCHain(const KDL::Chain &kdl_chain,
                                    sensor_msgs::JointState &joint_state);
    
}   
#endif