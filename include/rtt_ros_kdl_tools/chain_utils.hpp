#ifndef __CHAIN_UTILS__
#define __CHAIN_UTILS__

#include <rtt_ros_kdl_tools/chain_utils_base.hpp>
#include <trac_ik/trac_ik.hpp>

namespace rtt_ros_kdl_tools{


class ChainUtils : public ChainUtilsBase
{
public:
    bool init(
        const std::string& robot_description_name = "robot_description"
        ,const std::string& root_link = "root_link"
        ,const std::string& tip_link = "tip_link"
        ,const KDL::Vector gravity_vector = KDL::Vector(0.,0.,-9.81));
    bool cartesianToJoint(KDL::JntArray joint_seed, KDL::Frame desired_end_effector_pose, KDL::JntArray& return_joints, KDL::Twist tolerances);
    KDL::Frame& jointToCartesian(const KDL::JntArray& joints);
protected:
    boost::shared_ptr<TRAC_IK::TRAC_IK> ik_solver;
    KDL::Frame fk_last_segment_;
};

}

#endif
