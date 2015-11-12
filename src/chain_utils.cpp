#include "rtt_ros_kdl_tools/chain_utils.hpp"

namespace rtt_ros_kdl_tools{
  
  ChainUtils::ChainUtils(const std::string& robot_description_name, const std::string& root_link_name, const std::string& tip_link_name ){
    rtt_ros_kdl_tools::initChainFromROSParamURDF(kdl_tree_, kdl_chain_);
    rtt_ros_kdl_tools::readJntLimitsFromROSParamURDF(joints_name_, joints_lower_limit_, joints_upper_limit_, kdl_tree_, kdl_chain_, robot_description_name, root_link_name, tip_link_name);
    
    ros::NodeHandle nh("");
    nh.getParam(root_link_name, root_link_);
    nh.getParam(tip_link_name, tip_link_);
    
    q_.resize(kdl_chain_.getNrOfJoints());
    qd_.resize(kdl_chain_.getNrOfJoints());
    massMatrix_.resize(kdl_chain_.getNrOfJoints());
    gravityTorque_.resize(kdl_chain_.getNrOfJoints());
    corioCentriTorque_.resize(kdl_chain_.getNrOfJoints());
    
    chainjacsolver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
    fksolver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
    fksolvervel_.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain_));
    dynModelSolver_.reset(new KDL::ChainDynParam(kdl_chain_, KDL::Vector(0.,0.,-9.81)));
    jntToJacDotSolver_.reset(new KDL::ChainJntToJacDotSolver(kdl_chain_));
    
    outdate();
  }  

  void ChainUtils::printChain(){
    if(kdl_chain_.getNrOfSegments() == 0)
      ROS_WARN("KDL chain empty !");
    ROS_INFO("KDL chain from tree: ");
    if(kdl_chain_.getNrOfSegments() > 0)
      ROS_INFO_STREAM("  root_link: "<<root_link_<<" --> tip_link: "<<tip_link_);
    ROS_INFO_STREAM("  Chain has "<<kdl_chain_.getNrOfJoints()<<" joints");
    ROS_INFO_STREAM("  Chain has "<<kdl_chain_.getNrOfSegments()<<" segments");

    for(unsigned int i=0;i<kdl_chain_.getNrOfSegments();++i)
      ROS_INFO_STREAM("    "<<kdl_chain_.getSegment(i).getName());
    
    ROS_INFO_STREAM(joints_name_.size()<<" joints limited :");
    for(int i=0; i<joints_name_.size(); i++)
      ROS_INFO_STREAM("  Joint "<<joints_name_[i]<<" has limits "<<joints_lower_limit_[i]<<" and "<< joints_upper_limit_[i]);
  }
  
  int ChainUtils::nbSegments(){
    return kdl_chain_.getNrOfSegments();
  }

  void ChainUtils::getSegment(int segment, KDL::Segment &kdl_segment){
    kdl_segment = kdl_chain_.getSegment(segment);
  }

  void ChainUtils::getSegmentPosition(int segment, KDL::Frame &kdl_frame){
    fksolver_->JntToCart(q_, kdl_frame, segment);
  }

  void ChainUtils::getSegmentPosition(std::string& segment_name, KDL::Frame &kdl_frame){
    fksolver_->JntToCart(q_, kdl_frame, getSegmentIndex(segment_name));
  }

  void ChainUtils::getSegmentVelocity(int segment, KDL::Twist &kdl_twist){
    KDL::FrameVel cart_vel;
    KDL::JntArrayVel vel(q_, qd_);
    fksolvervel_->JntToCart(vel, cart_vel, segment);
    kdl_twist = cart_vel.GetTwist();
  }

  void ChainUtils::getSegmentVelocity(std::string& segment_name, KDL::Twist &kdl_twist){
    KDL::FrameVel cart_vel;
    KDL::JntArrayVel vel(q_, qd_);
    fksolvervel_->JntToCart(vel, cart_vel, getSegmentIndex(segment_name));
    kdl_twist = cart_vel.GetTwist();
  }

  void ChainUtils::getSegmentJacobian(int segment, KDL::Jacobian &kdl_jacobian){
    kdl_jacobian.resize(kdl_chain_.getNrOfJoints());
    chainjacsolver_->JntToJac(q_, kdl_jacobian, segment);
  }

  void ChainUtils::getSegmentJacobian(std::string& segment_name, KDL::Jacobian &kdl_jacobian){
    kdl_jacobian.resize(kdl_chain_.getNrOfJoints());
    chainjacsolver_->JntToJac(q_, kdl_jacobian, getSegmentIndex(segment_name));
  }

  const std::string& ChainUtils::getSegmentName(int index){
    return kdl_chain_.getSegment(index).getName();
  }

  int ChainUtils::getSegmentIndex(const std::string name){
    return seg_names_idx_[name];
  }

  void ChainUtils::getJointLimits(std::vector<std::string>& limited_joints, std::vector<double>& lower_limits, std::vector<double>& upper_limits){
    limited_joints = joints_name_;
    lower_limits = joints_lower_limit_;
    upper_limits = joints_upper_limit_;
  }

  void ChainUtils::getJointPositions(KDL::JntArray &q){
    q = q_;
  }

  void ChainUtils::getJointVelocities(KDL::JntArray &qd){
    qd = qd_;
  }

  void ChainUtils::getJdotQdot(std::string& segment_name, KDL::Twist &kdl_twist){
    KDL::JntArrayVel jntArrVel;
    jntArrVel.q = q_;
    jntArrVel.qdot = qd_;
    jntToJacDotSolver_->JntToJacDot(jntArrVel, kdl_twist, getSegmentIndex(segment_name));
  }

  void ChainUtils::getJdotQdot(int segment, KDL::Twist &kdl_twist){
    KDL::JntArrayVel jntArrVel;
    jntArrVel.q = q_;
    jntArrVel.qdot = qd_;
    jntToJacDotSolver_->JntToJacDot(jntArrVel, kdl_twist, segment);
  }

  void ChainUtils::getInertiaMatrix(KDL::JntSpaceInertiaMatrix &massMatrix ){
    if(inertiaMatrixOutdated_ == true){
      computeMassMatrix();
      inertiaMatrixOutdated_ = false;
    }
    massMatrix = massMatrix_;
  }

  void ChainUtils::getCoriolisTorque(KDL::JntArray &corioCentriTorque){
    if(corioCentriTorqueOutdated_ == true){
      computeCorioCentriTorque();
      corioCentriTorqueOutdated_ = false;
    }
    corioCentriTorque = corioCentriTorque_;
  }

  void ChainUtils::getGravityTorque(KDL::JntArray &gravityTorque){
    if(gravityOutdated_ == true){
      computeGravityTorque();
      gravityOutdated_ = false;
    }
    gravityTorque = gravityTorque_;
  }

  void ChainUtils::setJointPosition(std::vector<double> &q_des){
    outdate();
    for(unsigned int i=0; i<kdl_chain_.getNrOfJoints(); i++){
      q_(i) = q_des[i];
    }
  }

  void ChainUtils::setJointVelocity(std::vector<double> &qd_des){
    outdate();
    for(unsigned int i=0; i<kdl_chain_.getNrOfJoints(); i++){
      qd_(i) = qd_des[i];
    }
  }

  void ChainUtils::setExternalMeasuredWrench(std::vector<double> &f, std::vector<double> &t){
    for(unsigned int i=0; i<3; i++){
      W_ext_.force(i) = f[i];
      W_ext_.torque(i) = t[i];
    }
  }

  void ChainUtils::setExternalWrenchPoint(std::vector<double> &p){
    for(unsigned int i=0; i<3; i++){
      W_ext_point_(i) = p[i];
    }
  }

  void ChainUtils::computeMassMatrix(){
    dynModelSolver_->JntToMass(q_,massMatrix_);
  }

  void ChainUtils::computeCorioCentriTorque(){
    dynModelSolver_->JntToCoriolis(q_,qd_,corioCentriTorque_);
  }

  void ChainUtils::computeGravityTorque(){
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

  bool ChainUtils::inertiaMatrixOutdated(){
    return inertiaMatrixOutdated_;
  }

  bool ChainUtils::corioCentriTorqueOutdated(){
    return corioCentriTorqueOutdated_;
  }

  bool ChainUtils::gravityOutdated(){
    return gravityOutdated_;
  }

  void ChainUtils::outdate(){
    inertiaMatrixOutdated_ = true;
    corioCentriTorqueOutdated_ = true;
    gravityOutdated_ = true;
  }

}
