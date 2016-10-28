#include "rtt_ros_kdl_tools/chain_utils.hpp"
#include <ros/param.h>
#include <exception>

namespace rtt_ros_kdl_tools {

ChainUtilsBase::ChainUtilsBase()
{
}
bool ChainUtilsBase::init(
    const std::string& robot_description_name,
    const std::string& root_link_name,
    const std::string& tip_link_name,
    const KDL::Vector gravity_vector)
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

    joint_lower_limit_.resize(kdl_chain_.getNrOfJoints());
    joint_upper_limit_.resize(kdl_chain_.getNrOfJoints());
    joint_friction_.resize(kdl_chain_.getNrOfJoints());
    joint_damping_.resize(kdl_chain_.getNrOfJoints());

    joint_lower_limit_ = Eigen::VectorXd::Map(joints_lower_limit_.data(),kdl_chain_.getNrOfJoints());
    joint_upper_limit_ = Eigen::VectorXd::Map(joints_upper_limit_.data(),kdl_chain_.getNrOfJoints());
    joint_friction_ = Eigen::VectorXd::Map(joints_friction_.data(),kdl_chain_.getNrOfJoints());
    joint_damping_ = Eigen::VectorXd::Map(joints_damping_.data(),kdl_chain_.getNrOfJoints());

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

    ext_wrench_torque_.resize(kdl_chain_.getNrOfJoints());
    ext_add_torque_.resize(kdl_chain_.getNrOfJoints());
    ext_torque_all_.resize(kdl_chain_.getNrOfJoints());

    f_ext_.resize(kdl_chain_.getNrOfSegments());
    std::fill(f_ext_.begin(),f_ext_.end(),KDL::Wrench());

    chainjacsolver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
    fksolver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
    fksolvervel_.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain_));
    dynModelSolver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_vector));
    jntToJacDotSolver_.reset(new KDL::ChainJntToJacDotSolver(kdl_chain_));
    inverseDynamicsSolver_.reset(new KDL::ChainIdSolver_RNE(kdl_chain_,gravity_vector));

    return true;
}

Eigen::VectorXd& ChainUtilsBase::getJointLowerLimit()
{
    return this->joint_lower_limit_;
}

Eigen::VectorXd& ChainUtilsBase::getJointUpperLimit()
{
    return this->joint_upper_limit_;
}

Eigen::VectorXd& ChainUtilsBase::getJointDamping()
{
    return this->joint_damping_;
}

Eigen::VectorXd& ChainUtilsBase::getJointFriction()
{
    return this->joint_friction_;
}

void ChainUtilsBase::printChain() {
    rtt_ros_kdl_tools::printChain(kdl_chain_);

    ROS_INFO_STREAM(joints_name_.size()<<" joints limited :");
    for(int i=0; i<joints_name_.size(); i++)
        ROS_INFO_STREAM("  Joint "<<joints_name_[i]<<" has limits "<<joints_lower_limit_[i]<<" and "<< joints_upper_limit_[i]);
}

int ChainUtilsBase::getNrOfSegments() {
    return kdl_chain_.getNrOfSegments();
}
int ChainUtilsBase::getNrOfJoints()
{
    return kdl_chain_.getNrOfJoints();
}

const KDL::Segment& ChainUtilsBase::getSegment(unsigned int segment) {
    return kdl_chain_.getSegment(segment);
}

KDL::Frame& ChainUtilsBase::getSegmentPosition(unsigned int segment) {
    fksolver_->JntToCart(q_, seg_pos_, segment+1);
    return seg_pos_;
}

KDL::Frame& ChainUtilsBase::getSegmentPosition(std::string& segment_name) {
    return getSegmentPosition(getSegmentIndex(segment_name));
}

KDL::Twist& ChainUtilsBase::getSegmentVelocity(unsigned int segment) {
    fksolvervel_->JntToCart(qqd_, frame_vel_, segment+1);
    seg_vel_ = frame_vel_.GetTwist();
    return seg_vel_;
}

KDL::Twist& ChainUtilsBase::getSegmentVelocity(std::string& segment_name) {
    return getSegmentVelocity(getSegmentIndex(segment_name));
}

KDL::Jacobian& ChainUtilsBase::getSegmentJacobian(unsigned int segment) {
    chainjacsolver_->JntToJac(q_, seg_jacobian_, segment+1);
    return seg_jacobian_;
}
KDL::Jacobian& ChainUtilsBase::getJacobian()
{
    return jacobian_;
}

void ChainUtilsBase::computeJacobian()
{
    chainjacsolver_->JntToJac(q_, jacobian_);
}

KDL::Jacobian& ChainUtilsBase::getSegmentJacobian(const std::string& segment_name) {
    return getSegmentJacobian(getSegmentIndex(segment_name));
}

const std::string& ChainUtilsBase::getSegmentName(unsigned int index) {
    return kdl_chain_.getSegment(index).getName();
}

unsigned int ChainUtilsBase::getSegmentIndex(const std::string& name) {
    return seg_names_idx_[name];
}

const std::string& ChainUtilsBase::getRootSegmentName( ) {
	return this->root_link_name_;
}

const std::string& ChainUtilsBase::getTipSegmentName( ) {
        return this->tip_link_name_;
}

void ChainUtilsBase::getJointLimits(
    std::vector<std::string>& limited_joints
    , std::vector<double>& lower_limits
    , std::vector<double>& upper_limits)
{
    limited_joints = joints_name_;
    lower_limits = joints_lower_limit_;
    upper_limits = joints_upper_limit_;
}

KDL::JntArray& ChainUtilsBase::getJointPositions() {
    return q_;
}

KDL::JntArray& ChainUtilsBase::getJointVelocities() {
    return qd_;
}

KDL::Twist& ChainUtilsBase::getSegmentJdotQdot(const std::string& segment_name) {
    return getSegmentJdotQdot(getSegmentIndex(segment_name));
}

KDL::Twist& ChainUtilsBase::getSegmentJdotQdot(unsigned int segment) {
    jntToJacDotSolver_->JntToJacDot(qqd_, seg_jdot_qdot_, segment+1);
    return seg_jdot_qdot_;
}
KDL::Jacobian& ChainUtilsBase::getSegmentJdot(const std::string& segment_name)
{
    return getSegmentJdot(getSegmentIndex(segment_name));
}
KDL::Jacobian& ChainUtilsBase::getSegmentJdot(unsigned int segment)
{
    jntToJacDotSolver_->JntToJacDot(qqd_,seg_jacobian_dot_,segment+1);
    return seg_jacobian_dot_;
}

KDL::JntSpaceInertiaMatrix& ChainUtilsBase::getInertiaMatrix() {
	return massMatrix_;
}

KDL::JntSpaceInertiaMatrix& ChainUtilsBase::getInertiaInverseMatrix() {
	return massMatrixInv_;
}


KDL::JntArray& ChainUtilsBase::getCoriolisTorque() {
    return corioCentriTorque_;
}

KDL::JntArray& ChainUtilsBase::getGravityTorque() {
    return gravityTorque_;
}

KDL::Twist& ChainUtilsBase::getJdotQdot()
{
    return jdot_qdot_;
}

std::vector< double >& ChainUtilsBase::getJointLowerLimits()
{
    return joints_lower_limit_;
}
std::vector< double >& ChainUtilsBase::getJointUpperLimits()
{
    return joints_upper_limit_;
}
std::vector< std::string >& ChainUtilsBase::getLimitedJointNames()
{
    return joints_name_;
}

std::vector< double >& ChainUtilsBase::getJointsFriction()
{
	return this->joints_friction_;
}

std::vector< double >& ChainUtilsBase::getJointsDamping()
{
	return this->joints_damping_;
}


void ChainUtilsBase::updateModel()
{
    computeJdotQdot();
    computeGravityTorque();
	computeInertiaMatrix();
	inverseInertiaMatrix();
    computeCorioCentriTorque();
    computeJacobian();
}

void ChainUtilsBase::setInertiaMatrixAsDiagonal()
{
    massMatrix_.data =  Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>( massMatrix_.data.diagonal().asDiagonal() );
    this->inverseInertiaMatrix();
}

void ChainUtilsBase::computeJdotQdot()
{
    jntToJacDotSolver_->JntToJacDot(qqd_, jdot_qdot_);
}
void ChainUtilsBase::computeInertiaMatrix()
{
	dynModelSolver_->JntToMass(q_,massMatrix_);
}
void ChainUtilsBase::inverseInertiaMatrix()
{
	massMatrixInv_.data = massMatrix_.data.inverse();
}

KDL::RotationalInertia& ChainUtilsBase::getSegmentInertiaMatrix(unsigned int seg_idx)
{
    rot_intertia = this->Chain().getSegment(seg_idx).getInertia().getRotationalInertia();
    return rot_intertia;
}
KDL::RotationalInertia& ChainUtilsBase::getSegmentInertiaMatrix(const std::string& seg_name)
{
    return getSegmentInertiaMatrix(getSegmentIndex(seg_name));
}

void ChainUtilsBase::computeCorioCentriTorque() {
    dynModelSolver_->JntToCoriolis(q_,qd_,corioCentriTorque_);
}

void ChainUtilsBase::computeGravityTorque() {
    dynModelSolver_->JntToGravity(q_,gravityTorque_);
}

void ChainUtilsBase::setExternalMeasuredWrench(const KDL::Wrench& external_wrench, int segment_number)
{
    if(0 <= segment_number && segment_number < f_ext_.size())
    {
        f_ext_[segment_number] = external_wrench;
    }
    else
    {
        std::cerr << "Wrong segment number provided "<<segment_number<< std::endl;
    }
}

void ChainUtilsBase::computeExternalWrenchTorque(bool compute_gravity /* = true */)
{
    return computeExternalWrenchTorque(q_.data,compute_gravity);
}

void ChainUtilsBase::computeExternalWrenchTorque(
    const Eigen::VectorXd& jnt_pos,
    bool compute_gravity /*= true*/)
{
    ext_wrench_torque_.data.setZero();
    zero_kdl.data.setZero();

    tmp_array_pos.data = jnt_pos;

    inverseDynamicsSolver_->CartToJnt(tmp_array_pos,zero_kdl,zero_kdl,f_ext_,ext_wrench_torque_);
    // Remove gravity
    if(compute_gravity)
        dynModelSolver_->JntToGravity(tmp_array_pos,gravityTorque_);

    ext_wrench_torque_.data -= getGravityTorque().data;
}

KDL::JntArray& ChainUtilsBase::getExternalWrenchTorque()
{
    return ext_wrench_torque_;
}


void ChainUtilsBase::setExternalAdditionalTorque(const Eigen::VectorXd& external_add_torque)
{
    if(external_add_torque.size() != ext_add_torque_.data.size())
        throw std::runtime_error("Size mismatch");
    this->ext_add_torque_.data = external_add_torque;
}
KDL::JntArray& ChainUtilsBase::getExternalAdditionalTorque()
{
    return this->ext_add_torque_;
}

KDL::JntArray& ChainUtilsBase::getTotalExternalTorque()
{
    this->ext_torque_all_.data = this->ext_add_torque_.data + this->ext_wrench_torque_.data;
    return ext_torque_all_;
}

}
