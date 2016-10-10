#ifndef __CHAIN_UTILS__
#define __CHAIN_UTILS__

#include <rtt_ros_kdl_tools/chain_utils_base.hpp>
#include <trac_ik/trac_ik.hpp>

namespace rtt_ros_kdl_tools{


class ChainUtils : public ChainUtilsBase
{
public:
    bool init();
    bool CartesianToJoint(KDL::JntArray joint_seed, KDL::Frame desired_end_effector_pose, KDL::JntArray& return_joints, KDL::Twist tolerances);
protected:
    boost::shared_ptr<TRAC_IK::TRAC_IK> ik_solver;
};

}

#endif
