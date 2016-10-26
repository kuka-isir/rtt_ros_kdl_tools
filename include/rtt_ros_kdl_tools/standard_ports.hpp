#ifndef __STANDARD_PORTS_HPP__
#define __STANDARD_PORTS_HPP__
#ifndef NO_OROCOS

#include <rtt/RTT.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt_rosparam/rosparam.h>
#include "rtt_ros_kdl_tools/eigen_port.hpp"

namespace rtt_ros_kdl_tools{
    
    class StandardPorts
    {
    public:
        StandardPorts(RTT::TaskContext * parent);
        
        RTT::InputPort<Eigen::VectorXd>& portJointPositionIn();
        Eigen::VectorXd& JointPositionIn();
        RTT::InputPort<Eigen::VectorXd>& portJointVelocityIn();
        Eigen::VectorXd& JointVelocityIn();
        RTT::InputPort<Eigen::VectorXd>& portJointTorqueIn();
        Eigen::VectorXd& JointTorqueIn();
        
        RTT::InputPort<Eigen::VectorXd>& portJointPositionOut();
        Eigen::VectorXd& JointPositionOut();
        RTT::InputPort<Eigen::VectorXd>& portJointVelocityOut();
        Eigen::VectorXd& JointVelocityOut();
        RTT::InputPort<Eigen::VectorXd>& portJointTorqueOut();
        Eigen::VectorXd& JointTorqueOut();
        
        void init(
            const std::string& robot_description_ros_name = "robot_description",
            const std::string& root_link_ros_name = "root_link",
            const std::string& tip_link_ros_name = "tip_link");
        
    private:
        RTT::InputPort<Eigen::VectorXd>  port_joint_position_in,
                                         port_joint_velocity_in,
                                         port_joint_torque_in;
        // Some input variables
        Eigen::VectorXd jnt_pos_in,
                        jnt_vel_in,
                        jnt_trq_in;
        // Output ports
        RTT::OutputPort<Eigen::VectorXd> port_joint_position_cmd_out,
                                         port_joint_velocity_cmd_out,
                                         port_joint_torque_cmd_out;
        // Some output variables
        Eigen::VectorXd jnt_pos_cmd_out,
                        jnt_vel_cmd_out,
                        jnt_trq_cmd_out;
    };
}

#endif
#endif // __STANDARD_PORTS_HPP__