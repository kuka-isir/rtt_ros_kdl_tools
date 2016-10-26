#include "rtt_ros_kdl_tools/standard_ports.hpp"
#include "rtt_ros_kdl_tools/tools.hpp"

namespace rtt_ros_kdl_tools
{
    StandardPorts::StandardPorts(RTT::TaskContext * parent)
    {
        parent->addPort("JointPosition",port_joint_position_in).doc("Current joint positions");
        parent->addPort("JointVelocity",port_joint_velocity_in).doc("Current joint velocities");
        parent->addPort("JointTorque",port_joint_torque_in).doc("Current joint torques");

        parent->addPort("JointPositionCommand",port_joint_position_cmd_out).doc("Commanded joint positions");
        parent->addPort("JointVelocityCommand",port_joint_velocity_cmd_out).doc("Commanded joint velocities");
        parent->addPort("JointTorqueCommand",port_joint_torque_cmd_out).doc("Commanded joint torques");
    }
    
    void init(
        const std::string& robot_description_ros_name /*= "robot_description"*/,
        const std::string& root_link_ros_name /*= "root_link"*/,
        const std::string& tip_link_ros_name /*= "tip_link"*/)
    {
        const int dof = getNumberOfJointsFromROSParamURDF(
            robot_description_ros_name,
            root_link_ros_name,
            tip_link_ros_name);
        
        this->jnt_pos_in.setZero(arm.getNrOfJoints());
        this->jnt_vel_in.setZero(arm.getNrOfJoints());
        this->jnt_trq_in.setZero(arm.getNrOfJoints());

        this->jnt_pos_cmd_out.setZero(arm.getNrOfJoints());
        this->jnt_vel_cmd_out.setZero(arm.getNrOfJoints());
        this->jnt_trq_cmd_out.setZero(arm.getNrOfJoints());

        this->port_joint_position_cmd_out.setDataSample(jnt_pos_cmd_out);
        this->port_joint_velocity_cmd_out.setDataSample(jnt_vel_cmd_out);
        this->port_joint_torque_cmd_out.setDataSample(jnt_trq_cmd_out);
    }
    
    RTT::InputPort<Eigen::VectorXd>& portJointPositionIn();
    {
        return this->port_joint_position_in;
    }
    Eigen::VectorXd& JointPositionIn()
    {
        return this->
    }
    RTT::InputPort<Eigen::VectorXd>& portJointVelocityIn()
    {
        return this->
    }
    Eigen::VectorXd& JointVelocityIn()
    {
        return this->
    }
    RTT::InputPort<Eigen::VectorXd>& portJointTorqueIn()
    {
        return this->
    }
    Eigen::VectorXd& JointTorqueIn()
    {
        return this->
    }
    
    RTT::InputPort<Eigen::VectorXd>& portJointPositionOut()
    {
        return this->
    }
    Eigen::VectorXd& JointPositionOut()
    {
        return this->
    }
    RTT::InputPort<Eigen::VectorXd>& portJointVelocityOut()
    {
        return this->
    }
    Eigen::VectorXd& JointVelocityOut()
    {
        return this->
    }
    RTT::InputPort<Eigen::VectorXd>& portJointTorqueOut()
    {
        return this->
    }
    Eigen::VectorXd& JointTorqueOut()
    {
        return this->
    }
    
}