#include "rtt_ros_kdl_tools/chain_utils.hpp"
#include <ros/param.h>
#include <kdl_conversions/kdl_msg.h>
#include <eigen_conversions/eigen_kdl.h>
#include <kdl/frames_io.hpp>

namespace rtt_ros_kdl_tools {

ChainUtils::ChainUtils()
{
}
bool ChainUtils::init(
				const std::string& robot_description_name,
                const std::string& root_link_name,
                const std::string& tip_link_name,
                const KDL::Vector gravity_vector
)
{
	this->robot_description_ros_name = robot_description_name;
	this->root_link_ros_name = root_link_name;
	this->tip_link_ros_name = tip_link_name;
	this->gravity_vector = gravity_vector;
    this->ft_sensor_measure_link = tip_link_name;

	ros::param::get(root_link_name, root_link_name_);

    if(!rtt_ros_kdl_tools::initChainFromROSParamURDF(kdl_tree_, kdl_chain_))
    {
        std::cerr << "Error at rtt_ros_kdl_tools::initChainFromROSParamURDF" <<std::endl;
        return false;
    }
    if(!rtt_ros_kdl_tools::readJntLimitsFromROSParamURDF(joints_name_
            , joints_lower_limit_
            , joints_upper_limit_
            , kdl_tree_
            , kdl_chain_
            , robot_description_name
            , root_link_name
            , tip_link_name))
    {
        std::cerr << "Error at rtt_ros_kdl_tools::readJntLimitsFromROSParamURDF" <<std::endl;
        return false;
    }

    rtt_ros_kdl_tools::readJntDynamicsFromROSParamURDF(joints_name_
            , joints_friction_
            , joints_damping_
            , kdl_tree_
            , kdl_chain_
            , robot_description_name
            , root_link_name
            , tip_link_name);

    for(int i=0;i<kdl_chain_.getNrOfSegments();i++)
        seg_names_idx_.add(kdl_chain_.getSegment(i).getName(),i);

    q_.resize(kdl_chain_.getNrOfJoints());
    qd_.resize(kdl_chain_.getNrOfJoints());
    qqd_.resize(kdl_chain_.getNrOfJoints());

    massMatrix_.resize(kdl_chain_.getNrOfJoints());
    massMatrixInv_.resize(kdl_chain_.getNrOfJoints());
    gravityTorque_.resize(kdl_chain_.getNrOfJoints());
    corioCentriTorque_.resize(kdl_chain_.getNrOfJoints());
    jacobian_.resize(kdl_chain_.getNrOfJoints());
    tmp_jac_.resize(kdl_chain_.getNrOfJoints());
    seg_jacobian_.resize(kdl_chain_.getNrOfJoints());
    seg_jacobian_dot_.resize(kdl_chain_.getNrOfJoints());
    zero_kdl.resize(kdl_chain_.getNrOfJoints());
    zero_kdl.data.setZero();

    tmp_array_pos.resize(kdl_chain_.getNrOfJoints());
    tmp_array_vel.resize(kdl_chain_.getNrOfJoints());

    ext_torque_.resize(kdl_chain_.getNrOfJoints());
    f_ext_.resize(kdl_chain_.getNrOfSegments());
    std::fill(f_ext_.begin(),f_ext_.end(),KDL::Wrench());

    chainjacsolver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
    fksolver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
    fksolvervel_.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain_));
    dynModelSolver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_vector));
    jntToJacDotSolver_.reset(new KDL::ChainJntToJacDotSolver(kdl_chain_));
    inverseDynamicsSolver_.reset(new KDL::ChainIdSolver_RNE(kdl_chain_,gravity_vector));

    outdate();
    return true;
}

void ChainUtils::printChain() {
    rtt_ros_kdl_tools::printChain(kdl_chain_);

    ROS_INFO_STREAM(joints_name_.size()<<" joints limited :");
    for(int i=0; i<joints_name_.size(); i++)
        ROS_INFO_STREAM("  Joint "<<joints_name_[i]<<" has limits "<<joints_lower_limit_[i]<<" and "<< joints_upper_limit_[i]);
}

int ChainUtils::getNrOfSegments() {
    return kdl_chain_.getNrOfSegments();
}
int ChainUtils::getNrOfJoints()
{
    return kdl_chain_.getNrOfJoints();
}

const KDL::Segment& ChainUtils::getSegment(unsigned int segment) {
    return kdl_chain_.getSegment(segment);
}

KDL::Frame& ChainUtils::getSegmentPosition(unsigned int segment) {
    fksolver_->JntToCart(q_, seg_pos_, segment+1);
    return seg_pos_;
}

KDL::Frame& ChainUtils::getSegmentPosition(std::string& segment_name) {
    return getSegmentPosition(getSegmentIndex(segment_name));
}

KDL::Twist& ChainUtils::getSegmentVelocity(unsigned int segment) {
    fksolvervel_->JntToCart(qqd_, frame_vel_, segment+1);
    seg_vel_ = frame_vel_.GetTwist();
    return seg_vel_;
}

KDL::Twist& ChainUtils::getSegmentVelocity(std::string& segment_name) {
    return getSegmentVelocity(getSegmentIndex(segment_name));
}

KDL::Jacobian& ChainUtils::getSegmentJacobian(unsigned int segment) {
    chainjacsolver_->JntToJac(q_, seg_jacobian_, segment+1);
    return seg_jacobian_;
}
KDL::Jacobian& ChainUtils::getJacobian()
{
    return jacobian_;
}

void ChainUtils::computeJacobian()
{
    chainjacsolver_->JntToJac(q_, jacobian_);
}

KDL::Jacobian& ChainUtils::getSegmentJacobian(const std::string& segment_name) {
    return getSegmentJacobian(getSegmentIndex(segment_name));
}

const std::string& ChainUtils::getSegmentName(unsigned int index) {
    return kdl_chain_.getSegment(index).getName();
}

unsigned int ChainUtils::getSegmentIndex(const std::string& name) {
    return seg_names_idx_[name];
}

const std::string& ChainUtils::getRootSegmentName( ) {
	return this->root_link_name_;
}

void ChainUtils::getJointLimits(
    std::vector<std::string>& limited_joints
    , std::vector<double>& lower_limits
    , std::vector<double>& upper_limits)
{
    limited_joints = joints_name_;
    lower_limits = joints_lower_limit_;
    upper_limits = joints_upper_limit_;
}

KDL::JntArray& ChainUtils::getJointPositions() {
    return q_;
}

KDL::JntArray& ChainUtils::getJointVelocities() {
    return qd_;
}

KDL::Twist& ChainUtils::getSegmentJdotQdot(const std::string& segment_name) {
    return getSegmentJdotQdot(getSegmentIndex(segment_name));
}

KDL::Twist& ChainUtils::getSegmentJdotQdot(unsigned int segment) {
    jntToJacDotSolver_->JntToJacDot(qqd_, seg_jdot_qdot_, segment+1);
    return seg_jdot_qdot_;
}
KDL::Jacobian& ChainUtils::getSegmentJdot(const std::string& segment_name)
{
    return getSegmentJdot(getSegmentIndex(segment_name));
}
KDL::Jacobian& ChainUtils::getSegmentJdot(unsigned int segment)
{
    jntToJacDotSolver_->JntToJacDot(qqd_,seg_jacobian_dot_,segment+1);
    return seg_jacobian_dot_;
}

KDL::JntSpaceInertiaMatrix& ChainUtils::getInertiaMatrix() {
	return massMatrix_;
}

KDL::JntSpaceInertiaMatrix& ChainUtils::getInertiaInverseMatrix() {
	return massMatrixInv_;
}


KDL::JntArray& ChainUtils::getCoriolisTorque() {
    return corioCentriTorque_;
}

KDL::JntArray& ChainUtils::getGravityTorque() {
    return gravityTorque_;
}

KDL::Twist& ChainUtils::getJdotQdot()
{
    return jdot_qdot_;
}

std::vector< double >& ChainUtils::getJointLowerLimits()
{
    return joints_lower_limit_;
}
std::vector< double >& ChainUtils::getJointUpperLimits()
{
    return joints_upper_limit_;
}
std::vector< std::string >& ChainUtils::getLimitedJointNames()
{
    return joints_name_;
}

std::vector< double >& ChainUtils::getJointsFriction()
{
	return this->joints_friction_;
}

std::vector< double >& ChainUtils::getJointsDamping()
{
	return this->joints_damping_;
}


void ChainUtils::updateModel()
{
    computeJdotQdot();
    computeGravityTorque();
	computeInertiaMatrix();
	inverseInertiaMatrix();
    computeCorioCentriTorque();
    computeJacobian();
}

void ChainUtils::computeJdotQdot()
{
    jntToJacDotSolver_->JntToJacDot(qqd_, jdot_qdot_);
}
void ChainUtils::computeInertiaMatrix()
{
	dynModelSolver_->JntToMass(q_,massMatrix_);
}
void ChainUtils::inverseInertiaMatrix()
{
	massMatrixInv_.data = massMatrix_.data.inverse();
}
KDL::RotationalInertia& ChainUtils::getSegmentInertiaMatrix(unsigned int seg_idx)
{
    rot_intertia = this->Chain().getSegment(seg_idx).getInertia().getRotationalInertia();
    return rot_intertia;
}
KDL::RotationalInertia& ChainUtils::getSegmentInertiaMatrix(const std::string& seg_name)
{
    return getSegmentInertiaMatrix(getSegmentIndex(seg_name));
}

void ChainUtils::computeCorioCentriTorque() {
    dynModelSolver_->JntToCoriolis(q_,qd_,corioCentriTorque_);
}

void ChainUtils::computeGravityTorque() {
    dynModelSolver_->JntToGravity(q_,gravityTorque_);
}

void ChainUtils::setExternalMeasuredWrench(const geometry_msgs::Wrench& external_wrench, int segment_number)
{
    tf::wrenchMsgToKDL(external_wrench,W_ext_);
    if(0 <= segment_number && segment_number < f_ext_.size())
    {
        f_ext_[segment_number] = W_ext_;
    }
    else
    {
        std::cerr << "Wrong segment number provided "<<segment_number<< std::endl;
    }
}

KDL::JntArray& ChainUtils::computeExternalWrenchTorque(bool compute_gravity /* = true */)
{
    return computeExternalWrenchTorque(q_.data,qd_.data,compute_gravity);
}

KDL::JntArray& ChainUtils::computeExternalWrenchTorque(const Eigen::VectorXd& jnt_pos,
                                                       const Eigen::VectorXd& jnt_vel,
                                                       bool compute_gravity /*= true*/)
{
    ext_torque_.data.setZero();

    tmp_array_pos.data = jnt_pos;
    tmp_array_vel.data = jnt_vel;

    inverseDynamicsSolver_->CartToJnt(tmp_array_pos,tmp_array_vel,zero_kdl,f_ext_,ext_torque_);
    // Remove gravity
    if(compute_gravity)
        dynModelSolver_->JntToGravity(tmp_array_pos,gravityTorque_);

    ext_torque_.data -= getGravityTorque().data;

    return ext_torque_;
}



// void ChainUtils::computeFrictionTorque(){
// TODO
//   frictionTorque_.data <<  	FV1*qd(0) + FS1*sign(qd(0)),
// 							FV2*qd(1) + FS2*sign(qd(1)),
// 							FV3*qd(2) + FS3*sign(qd(2)),
// 							FV4*qd(3) + FS4*sign(qd(3)),
// 							FV5*qd(4) + FS5*sign(qd(4)),
// 							FV6*qd(5) + FS6*sign(qd(5)),
// 							FV7*qd(6) + FS7*sign(qd(6));
// }

bool ChainUtils::inertiaMatrixOutdated() {
    return inertiaMatrixOutdated_;
}

bool ChainUtils::corioCentriTorqueOutdated() {
    return corioCentriTorqueOutdated_;
}

bool ChainUtils::gravityOutdated() {
    return gravityOutdated_;
}
bool ChainUtils::isOutdated()
{
    return isOutdated_;
}
void ChainUtils::outdate() {
    inertiaMatrixOutdated_ = true;
    corioCentriTorqueOutdated_ = true;
    gravityOutdated_ = true;
    coriolisOutdated_ = true;
    jacobianOutdated_ = true;
    cartPosOutdated_ = true;
    cartVelOutdated_ = true;
    isOutdated_ = true;
}

}
