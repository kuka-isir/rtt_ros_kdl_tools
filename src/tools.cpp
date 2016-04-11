#include <rtt_ros_kdl_tools/tools.hpp>
#include <ros/param.h>
#include <rtt_rosparam/rosparam.h>
#include <urdf/model.h>

namespace rtt_ros_kdl_tools {
unsigned int getNumberOfJointsFromROSParamURDF(const std::string& robot_description_ros_name/* = "robot_description"*/,
                               const std::string& root_link_ros_name/* = "root_link"*/,
                               const std::string& tip_link_ros_name/* = "tip_link"*/)
{
    KDL::Chain c;
    KDL::Tree t;
    if(initChainFromROSParamURDF(t,c,robot_description_ros_name,root_link_ros_name,tip_link_ros_name))
        return c.getNrOfJoints();
    else
        return 0;
}

bool initChainFromString(const std::string& robot_description,
                         const std::string& root_link,
                         const std::string& tip_link,
                         KDL::Tree& kdl_tree,
                         KDL::Chain& kdl_chain)
{
    urdf::Model urdf_model;

    if(!urdf_model.initString(robot_description)) {
      ROS_ERROR("Could not init URDF");
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
            ROS_WARN_STREAM("Removing fixed joint "<<joint->second->name<<std::endl);
        }
      }
    }

    if (!kdl_parser::treeFromString(robot_description,kdl_tree)) {
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }

    const KDL::SegmentMap& segments(kdl_tree.getSegments());
    if(segments.find(root_link.c_str()) == segments.end())
    {
        std::cerr << "Key " << root_link << " not found in kdl tree"<< std::endl;
        return false;
    }

    if(segments.find(tip_link.c_str()) == segments.end())
    {
        std::cerr << "Key " << tip_link << " not found in kdl tree"<< std::endl;
        return false;
    }

    if(!kdl_tree.getChain(root_link, tip_link, kdl_chain))
    {
        ROS_ERROR("Failed to build the KDL chain with params :\n  root_link: [%s]\n  tip_link: [%s]",root_link.c_str(),tip_link.c_str());
        return false;
    }
    return true;
}

void printChain(const KDL::Chain& kdl_chain)
{
    if(kdl_chain.getNrOfSegments() == 0)
        ROS_WARN("KDL chain empty !");
    ROS_INFO("KDL chain from tree: ");
    if(kdl_chain.getNrOfSegments() > 0)
        ROS_INFO_STREAM("  "<<kdl_chain.getSegment(0).getName()<<" --> "<<kdl_chain.getSegment(kdl_chain.getNrOfSegments()-1).getName());
    ROS_INFO_STREAM("  Chain has "<<kdl_chain.getNrOfJoints()<<" joints");
    ROS_INFO_STREAM("  Chain has "<<kdl_chain.getNrOfSegments()<<" segments");

    for(unsigned int i=0; i<kdl_chain.getNrOfSegments(); ++i)
        ROS_INFO_STREAM("    "<<kdl_chain.getSegment(i).getName());
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

    std::string robot_description,root_link,tip_link;

    RTT::Property<std::string> root_link_prop =  task->getProperty(root_link_rtt_name);
    RTT::Property<std::string> tip_link_prop =  task->getProperty(tip_link_rtt_name);
    RTT::Property<std::string> robot_description_prop =  task->getProperty(robot_description_rtt_name);

    if(!task->getProperty(robot_description_rtt_name))
    {
        RTT::log(RTT::Error) << "robot_description property not found, please add it to your class :\n"
                             <<"this->addProperty(\"robot_description\",robot_description);" << RTT::endlog();
        return false;
    } else if(!robot_description_prop.get().empty())
        robot_description = robot_description_prop.get();

    if(!task->getProperty(root_link_rtt_name))
    {
        RTT::log(RTT::Error) << "root_link property not found, please add it to your class :\n"
                             <<"this->addProperty(\"root_link\",root_link);" << RTT::endlog();
        return false;
    } else if(!root_link_prop.get().empty())
        root_link = root_link_prop.get();


    if(!task->getProperty(tip_link_rtt_name))
    {
        RTT::log(RTT::Error) << "tip_link property not found, please add it to your class :\n"
                             <<"this->addProperty(\"tip_link\",tip_link);" << RTT::endlog();
        return false;
    } else if(!tip_link_prop.get().empty())
        tip_link = tip_link_prop.get();

    boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
        task->getProvider<rtt_rosparam::ROSParam>("rosparam");

    if(!rosparam) {
        RTT::log(RTT::Error) << "Could not load rosparam service." <<RTT::endlog();
        return false;
    }
    if(root_link.empty() || tip_link.empty() || robot_description.empty())
    {
        bool success = true;
        success &= rosparam->getParam(root_link_ros_name,root_link_rtt_name);
        success &= rosparam->getParam(tip_link_ros_name,tip_link_rtt_name);
        success &= rosparam->getParam(robot_description_ros_name,robot_description_rtt_name);

        robot_description = robot_description_prop.get();
        root_link = root_link_prop.get();
        tip_link = tip_link_prop.get();

        if(!success)
            RTT::log(RTT::Error) << "Error while getting "<<root_link_rtt_name<<" "<<" "<<tip_link_rtt_name<<" and "<<robot_description_rtt_name<< RTT::endlog();
    }

    return initChainFromString(robot_description_prop.get(),root_link,tip_link,kdl_tree,kdl_chain);
}

bool initChainFromROSParamURDF(KDL::Tree& kdl_tree,
                               KDL::Chain& kdl_chain,
                               const std::string& robot_description_ros_name/* = "robot_description"*/,
                               const std::string& root_link_ros_name/* = "root_link"*/,
                               const std::string& tip_link_ros_name/* = "tip_link"*/)
{
    std::string robot_description_string, root_link_string, tip_link_string;
    if(!ros::param::get(robot_description_ros_name, robot_description_string))
        ROS_ERROR( "Could not get %s param",robot_description_ros_name.c_str());

    if(!ros::param::get(root_link_ros_name, root_link_string))
        ROS_ERROR( "Could not get %s param",root_link_ros_name.c_str());

    if(!ros::param::get(tip_link_ros_name, tip_link_string))
        ROS_ERROR( "Could not get %s param",tip_link_ros_name.c_str());

    return initChainFromString(robot_description_string,root_link_string,tip_link_string,kdl_tree,kdl_chain);

}

bool readJntLimitsFromROSParamURDF(std::vector<std::string>& limited_jnt_names,
                                   std::vector<double>& lower_limits,
                                   std::vector<double>& upper_limits,
                                   KDL::Tree& kdl_tree,
                                   KDL::Chain& kdl_chain,
                                   const std::string& robot_description_ros_name/* = "robot_description"*/,
                                   const std::string& root_link_ros_name/* = "root_link"*/,
                                   const std::string& tip_link_ros_name/* = "tip_link"*/)
{
    limited_jnt_names.clear();
    lower_limits.clear();
    upper_limits.clear();

    std::string robot_description_string, root_link_string, tip_link_string;

    if(!ros::param::get(robot_description_ros_name, robot_description_string))
        ROS_ERROR( "Could not get %s param",robot_description_ros_name.c_str());

    if(!ros::param::get(root_link_ros_name, root_link_string))
        ROS_ERROR( "Could not get %s param",root_link_ros_name.c_str());

    if(!ros::param::get(tip_link_ros_name, tip_link_string))
        ROS_ERROR( "Could not get %s param",tip_link_ros_name.c_str());

    urdf::Model urdf_model;

    if(!urdf_model.initString(robot_description_string)) {
        ROS_ERROR("Could not init URDF");
        return false;
    }

    if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree)) {
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }

    if(!kdl_tree.getChain(root_link_string, tip_link_string, kdl_chain)) {
        ROS_ERROR("Failed to get kinematic chain");
        return false;
    }

    int nbr_segs = kdl_chain.getNrOfSegments();
    std::vector<std::string> seg_names;

    for(int i=0; i<nbr_segs; i++)
        seg_names.push_back(kdl_chain.getSegment(i).getJoint().getName());

    for (std::map<std::string,boost::shared_ptr<urdf::Joint> >::iterator joint = urdf_model.joints_.begin(); joint != urdf_model.joints_.end(); ++joint) {
        if ( joint->second->limits && std::find(seg_names.begin(), seg_names.end(),joint->second->name)!=seg_names.end() ) {
            if (joint->second->limits->lower != joint->second->limits->upper) {
                limited_jnt_names.push_back(joint->second->name);
                lower_limits.push_back(joint->second->limits->lower);
                upper_limits.push_back(joint->second->limits->upper);
            }
        }
    }
    return true;
}

void initJointStateFromKDLCHain(const KDL::Chain &kdl_chain,sensor_msgs::JointState &joint_state)
{
    // Construct blank joint state message
    joint_state = sensor_msgs::JointState();
    // Add joint names
    for(unsigned int i=0; i<kdl_chain.getNrOfSegments(); ++i)
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


bool initJointStateMsgFromString(const std::string& robot_description, sensor_msgs::JointState& joint_state)
{
    RTT::log(RTT::Debug)<<"Creating Joint State message from robot_description" << RTT::endlog();
    urdf::Model model;

    // Verify if provided robot_description is correct
    if(!model.initString(robot_description)) {
        RTT::log(RTT::Error) << "Could not init URDF." <<RTT::endlog();
        return false;
    }

    RTT::log(RTT::Debug) << "Robot name : "<<model.getName()<< RTT::endlog();
    // Create a blank joint state msg
    joint_state = sensor_msgs::JointState();

    // Reading Joints from urdf model
    for (std::map<std::string, boost::shared_ptr<urdf::Joint> >::iterator j=model.joints_.begin(); j!=model.joints_.end(); ++j)
    {
        if(j->second->limits)
        {
            if(j->second->limits->lower == j->second->limits->upper)
            {
                // NOTE: Setting pseudo fixed-joints to FIXED, so that KDL does not considers them.
                RTT::log(RTT::Debug) << "Removing fixed joint "<<j->second->name<<std::endl;
                continue;
            }
        }

        if(j->second->type != urdf::Joint::FIXED &&
                j->second->type != urdf::Joint::FLOATING)
        {
            const std::string name = j->first;
            RTT::log(RTT::Debug)<<"Adding Joint "<< name << RTT::endlog();
            joint_state.name.push_back(name);
            joint_state.position.push_back(0.);
            joint_state.velocity.push_back(0.);
            joint_state.effort.push_back(0.);
        }
    }
    return true;
}

bool getAllPropertiesFromROSParam(RTT::TaskContext* _this)
{
    // Get RosParameters if available
    boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
        _this->getProvider<rtt_rosparam::ROSParam>("rosparam");

    if(rosparam) {
        const RTT::PropertyBag::Properties &properties =  _this->properties()->getProperties();
        for(RTT::PropertyBag::Properties::const_iterator it = properties.begin();
            it != properties.end();++it)
        {
            if(rosparam->getParam(_this->getName() +"/"+(*it)->getName(),(*it)->getName()))
                RTT::log(RTT::Info) << _this->getName() +"/"+(*it)->getName() << " => "<< _this->getProperty((*it)->getName())<< RTT::endlog();
            else
                RTT::log(RTT::Info) << "No param found for "<<_this->getName() +"/"+(*it)->getName()<< RTT::endlog();
        }
    }else{
        RTT::log(RTT::Error) << "ROS Param could not be loaded "<< RTT::endlog();
        return false;
    }
    return true;
}

}
