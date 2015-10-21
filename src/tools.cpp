#include <rtt_ros_kdl_tools/tools.hpp>

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
    
    for (std::map<std::string,boost::shared_ptr<urdf::Joint> >::iterator joint = urdf_model.joints_.begin();joint != urdf_model.joints_.end(); ++joint)
    {
        if(joint->second->limits)
        {
            if(joint->second->limits->lower == joint->second->limits->upper)
            {
                // HACK: Setting pseudo fixed-joints to FIXED, so that KDL does not considers them.
                joint->second->type = urdf::Joint::FIXED;
                RTT::log(RTT::Warning) << "Removing fixed joint "<<joint->second->name<<std::endl;
            }
        }
    }

    if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree)){
        RTT::log(RTT::Error) <<("Failed to construct kdl tree");
        return false;
    }

    if(!kdl_tree.getChain(root_link, tip_link, kdl_chain))
    {
        return false;
    }
    return true;
}

void printChain(const KDL::Chain& kdl_chain)
{
    if(kdl_chain.getNrOfSegments() == 0)
        RTT::log(RTT::Warning) <<"KDL chain empty !"<<RTT::endlog();
    RTT::log(RTT::Warning) <<"KDL chain from tree: "<<RTT::endlog();
    if(kdl_chain.getNrOfSegments() > 0)
        RTT::log(RTT::Warning) <<"  "<<kdl_chain.getSegment(0).getName()<<" --> "<<kdl_chain.getSegment(kdl_chain.getNrOfSegments()-1).getName()<<RTT::endlog();
    RTT::log(RTT::Warning) <<"  Chain has "<<kdl_chain.getNrOfJoints()<<" joints"<<RTT::endlog();
    RTT::log(RTT::Warning) <<"  Chain has "<<kdl_chain.getNrOfSegments()<<" segments"<<RTT::endlog();

    for(unsigned int i=0;i<kdl_chain.getNrOfSegments();++i)
        RTT::log(RTT::Warning) <<"    "<<kdl_chain.getSegment(i).getName()<<RTT::endlog();
}

bool initChainFromROSParamURDF(RTT::TaskContext* task, 
                                   KDL::Tree& kdl_tree, 
                                   KDL::Chain& kdl_chain, 
                                   const std::string& robot_description_ros_name/* = "robot_description"*/, 
                                   const std::string& robot_description_rtt_name/* = "robot_description"*/,
                                   const std::string& root_link_ros_name/* = "root_link"*/,
                                   const std::string& root_link_rtt_name/* = "root_link"*/,
                                   const std::string& tip_link_ros_name/* = "tip_link"*/,
                                   const std::string& tip_link_rtt_name/* = "tip_link"*/)
{
    if(task == NULL) return false;
    if(! task->getProperty(robot_description_rtt_name))
    {
        RTT::log(RTT::Error) << "robot_description property not found, please add it to your class :\n"
                    <<"this->addProperty(\"robot_description\",robot_description);" << RTT::endlog();
        return false;
    }
    
    if(! task->getProperty(root_link_rtt_name))
    {
        RTT::log(RTT::Error) << "root_link property not found, please add it to your class :\n"
                    <<"this->addProperty(\"root_link\",root_link);" << RTT::endlog();
        return false;
    }
    
    if(! task->getProperty(tip_link_rtt_name))
    {
        RTT::log(RTT::Error) << "tip_link property not found, please add it to your class :\n"
                    <<"this->addProperty(\"tip_link\",tip_link);" << RTT::endlog();
        return false;
    }
    
    boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
        task->getProvider<rtt_rosparam::ROSParam>("rosparam");

    if(!rosparam) {
        RTT::log(RTT::Error) << "Could not load rosparam service." <<RTT::endlog();
        return false;
    }
    
    if(!rosparam->getParam(root_link_ros_name,root_link_rtt_name))
        RTT::log(RTT::Warning) << root_link_ros_name<<" not provided, using default : " << root_link_rtt_name <<RTT::endlog();
    if(!rosparam->getParam(tip_link_ros_name,tip_link_rtt_name))
        RTT::log(RTT::Warning) << tip_link_ros_name<<" not provided, using default : " << tip_link_rtt_name <<RTT::endlog();
    
    RTT::Property<std::string> root_link = task->getProperty(root_link_rtt_name);
    RTT::Property<std::string> tip_link = task->getProperty(tip_link_rtt_name);
    
    if(root_link.get().empty() || tip_link.get().empty())
    {
        RTT::log(RTT::Error) << "Could not get "<<root_link_ros_name <<" or " << tip_link_ros_name<<RTT::endlog();
        return false;
    }
    
    rosparam->getParam(robot_description_ros_name,robot_description_rtt_name);

    RTT::Property<std::string> robot_description = task->getProperty(robot_description_rtt_name);
    
    if(robot_description.get().empty())
        RTT::log(RTT::Error) << "Error to get robot description at "<<robot_description_ros_name << RTT::endlog();
    
    return initChainFromString(robot_description.get(),root_link,tip_link,kdl_tree,kdl_chain);
}
void initJointStateFromKDLCHain(const KDL::Chain &kdl_chain,sensor_msgs::JointState &joint_state)
{
    // Construct blank joint state message
    joint_state = sensor_msgs::JointState();
    // Add joint names
    for(unsigned int i=0;i<kdl_chain.getNrOfSegments();++i)
    {
        if(kdl_chain.getSegment(i).getJoint().getType()!=KDL::Joint::None)
        {
            const std::string name = kdl_chain.getSegment(i).getJoint().getName();
            joint_state.name.push_back(name);
            joint_state.position.push_back(0.);
            joint_state.velocity.push_back(0.);
            joint_state.effort.push_back(0.);
        }  
    }
}

sensor_msgs::JointState initJointStateFromKDLCHain(const KDL::Chain& kdl_chain)
{
    // Construct blank joint state message
    sensor_msgs::JointState joint_state;
    initJointStateFromKDLCHain(kdl_chain,joint_state);
    return joint_state;
}

}
