#include "rtt_ros_kdl_tools/chain_utils.hpp"

namespace rtt_ros_kdl_tools {

ChainUtils::ChainUtils(bool init_kdl_chains,
                 const std::string& robot_description_name,
                 const std::string& root_link_name,
                 const std::string& tip_link_name,
                 const KDL::Vector gravity_vector):
robot_description_name(robot_description_name),
root_link_name(root_link_name),
tip_link_name(tip_link_name),
gravity_vector(gravity_vector),
is_initialized(false)
{

    if(init_kdl_chains)
        is_initialized = this->init();
}
bool ChainUtils::init()
{
    if(is_initialized)
    {
        std::cerr << "Chain is already initialized if you use the default constructor()"<<std::endl;
        std::cerr << "Maybe you called init() multiple times." <<std::endl;
        return true;
    }

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
    seg_jacobian_.resize(kdl_chain_.getNrOfJoints());
    seg_jacobian_dot_.resize(kdl_chain_.getNrOfJoints());

    chainjacsolver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
    fksolver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
    fksolvervel_.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain_));
    dynModelSolver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_vector));
    jntToJacDotSolver_.reset(new KDL::ChainJntToJacDotSolver(kdl_chain_));

    outdate();
    is_initialized = true;
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

void ChainUtils::setExternalMeasuredWrench(std::vector<double> &f, std::vector<double> &t) {
    for(unsigned int i=0; i<3; i++) {
        W_ext_.force(i) = f[i];
        W_ext_.torque(i) = t[i];
    }
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

void ChainUtils::updateModel()
{
    computeJdotQdot();
    computeGravityTorque();
	computeInertiaMatrix();
	inverseInertiaMatrix();
    computeCorioCentriTorque();
    computeJacobian();
}
void ChainUtils::setExternalWrenchPoint(std::vector<double> &p) {
    for(unsigned int i=0; i<3; i++) {
        W_ext_point_(i) = p[i];
    }
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


// void ChainUtils::computeExternalWrenchTorque(){
// TODO
//   KDL::Jacobian j(kdl_tree_.getNrOfJoints());
//   treejacsolver_->JntToJac(q_, j, "06");
//
//   KDL::Wrench w = W_ext_.RefPoint(-W_ext_point_);
//     Eigen::Vector3d f,t;
//
//   for(unsigned int i=0;i<3;i++){
//     f(i) = w.force.data[i];
//     t(i) = w.torque.data[i];
//   }
//
//   externalWrenchTorque_.data = j.data.transpose().block(0,0,7,3) * f + j.data.transpose().block(0,3,7,3) * t;
// }

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
