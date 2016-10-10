#include "rtt_ros_kdl_tools/chain_utils.hpp"

namespace rtt_ros_kdl_tools {

bool ChainUtils::init()
{
    if(!ChainUtilsBase::init()) return false;

    KDL::JntArray lower_joint_limits(getJointLowerLimits().size());
    KDL::JntArray upper_joint_limits(getJointUpperLimits().size());

    for (int i = 0; i < getJointLowerLimits().size() && i < getJointUpperLimits().size(); i++)
    {
        lower_joint_limits(i) = getJointLowerLimits()[i];
        upper_joint_limits(i) = getJointUpperLimits()[i];
    }

    double timeout_in_secs=0.005;
    double error=1e-5;
    TRAC_IK::SolveType type=TRAC_IK::Speed;

    ik_solver.reset(new TRAC_IK::TRAC_IK(this->Chain(),lower_joint_limits,upper_joint_limits,timeout_in_secs,error,type));

    return true;
}

bool ChainUtils::CartesianToJoint(KDL::JntArray joint_seed, KDL::Frame desired_end_effector_pose, KDL::JntArray& return_joints, KDL::Twist tolerances)
{
    return ik_solver->CartToJnt(joint_seed,desired_end_effector_pose,return_joints,tolerances) >= 0;
}


}
