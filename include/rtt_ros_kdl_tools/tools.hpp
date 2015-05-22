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
                             KDL::Chain& kdl_chain)
    {
        urdf::Model urdf_model;
                
        if(!urdf_model.initString(robot_description)) {
            RTT::log(RTT::Error) << "Could not Init URDF." <<RTT::endlog();
            return false;
        }
        

        if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree)){
            RTT::log(RTT::Error) <<("Failed to construct kdl tree");
            return false;
        }

        if(!kdl_tree.getChain(root_link, tip_link, kdl_chain))
        {
            return false;
        }
        RTT::log(RTT::Warning) <<"KDL chain from tree: "<<RTT::endlog();
        RTT::log(RTT::Warning) <<"  "<<root_link<<" --> "<<tip_link<<RTT::endlog();
        RTT::log(RTT::Warning) <<"  Tree has "<<kdl_tree.getNrOfJoints()<<" joints"<<RTT::endlog();
        RTT::log(RTT::Warning) <<"  Tree has "<<kdl_tree.getNrOfSegments()<<" segments"<<RTT::endlog();
        RTT::log(RTT::Warning) <<"  The segments are:"<<RTT::endlog();

        KDL::SegmentMap segment_map = kdl_tree.getSegments();
        KDL::SegmentMap::iterator it;

        for( it=segment_map.begin();
            it != segment_map.end();
            it++ )
        {
            RTT::log(RTT::Warning) <<"    "<<(*it).first<<RTT::endlog();
        }
        return true;
    }
    bool initChainFromROSParamURDF(RTT::TaskContext* task,
                             const std::string& ns,
                             const std::string& root_link,
                             const std::string& tip_link,
                             KDL::Tree& kdl_tree,
                             KDL::Chain& kdl_chain
                             )
    {
        if(task == NULL) return false;
        if(! task->getProperty("robot_description"))
        {
            RTT::log(RTT::Error) << "robot_description property not found, please add it to your class :\n"
                        <<"this->addProperty(\"robot_description\",robot_description);" << RTT::endlog();
            return false;
        }
        
        boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
            task->getProvider<rtt_rosparam::ROSParam>("rosparam");
    
        if(!rosparam) {
            RTT::log(RTT::Error) << "Could not load rosparam service." <<RTT::endlog();
            return false;
        }
        
        rosparam->getParam(ns+"/robot_description","robot_description");
        
        RTT::log(RTT::Debug) << "Trying to get robot description at "<<ns+"/robot_description" << RTT::endlog();
        
        RTT::Property<std::string> robot_description = task->getProperty("robot_description");
        return initChainFromString(robot_description.get(),root_link,tip_link,kdl_tree,kdl_chain);
    }
    void initJointStateFromKDLCHain(const KDL::Chain &kdl_chain,sensor_msgs::JointState &joint_state)
    {
        // Construct blank joint state message
        joint_state = sensor_msgs::JointState();

        // Add joint names
        for(std::vector<KDL::Segment>::const_iterator it=kdl_chain.segments.begin();
            it != kdl_chain.segments.end();
            it++)
        {
            joint_state.name.push_back(it->getJoint().getName());
        }

        // Get the #DOF
        unsigned int n_dof = kdl_chain.getNrOfJoints();

        // Resize joint vectors
        joint_state.position.resize(n_dof);
        joint_state.velocity.resize(n_dof);
        joint_state.effort.resize(n_dof);
        joint_state.position.assign(n_dof,0.0);
        joint_state.velocity.assign(n_dof,0.0);
        joint_state.effort.assign(n_dof,0.0);
    }
}   
#endif