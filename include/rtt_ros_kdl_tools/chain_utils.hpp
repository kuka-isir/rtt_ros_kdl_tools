#ifndef CHAIN_UTILS
#define CHAIN_UTILS

#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <boost/scoped_ptr.hpp>

#include <rtt_ros_kdl_tools/tools.hpp>
#include <rtt_ros_kdl_tools/chainjnttojacdotsolver.hpp>
#include <ros/ros.h>

namespace rtt_ros_kdl_tools{
  class SegmentIndice{
    public: const int operator()(const std::string& segment_name)
    {
        return this->operator[](segment_name);
    }
    public: const int operator[](const std::string& segment_name)
    {
        if(seg_idx_names.empty())
            return -1;
        if(seg_idx_names[segment_name])
            return seg_idx_names[segment_name];
        else{
            std::map<std::string,int>::iterator it;
            it = seg_idx_names.end();
            it--;
            std::cerr << "Segment name ["<<segment_name<<"] does not exists, returning last element in map ["<<it->second<<"]" << std::endl;
            return it->second;
        }
    }
    public: void add(const std::string& seg_name,int i)
    {
        seg_idx_names[seg_name] = i;
    }
    protected : std::map<std::string,int> seg_idx_names;
  };

  class ChainUtils{
    public:
      ChainUtils(bool init_kdl_chains = true
                ,const std::string& robot_description_name = "robot_description"
                ,const std::string& root_link = "root_link"
                ,const std::string& tip_link = "tip_link"
                ,const KDL::Vector gravity_vector = KDL::Vector(0.,0.,-9.81)
                );
      bool init();
      KDL::Chain& Chain(){return kdl_chain_;}
      KDL::Tree& Tree(){return kdl_tree_;}
      /**
	* @brief The root link of the kdl chain
	*/
      std::string root_link_;

      /**
	* @brief The tip link of the kdl chain
	*/
      std::string tip_link_;

      /**
	* @brief The joints name
	*/
      std::vector<std::string> joints_name_;

      /**
	* @brief The joints lower limits
	*/
      std::vector<double> joints_lower_limit_;

      /**
	* @brief The joints upper limits
	*/
      std::vector<double> joints_upper_limit_;

      /**
	* @brief The kinematic tree of the robot.
	*/
      KDL::Tree kdl_tree_;

      /**
	* @brief The kinematic chain of the robot.
	*/
      KDL::Chain kdl_chain_;

      /**
	* @brief The joint space mass matrix for the current joint position.
	*/
      KDL::JntSpaceInertiaMatrix massMatrix_,massMatrixInv_;

      /**
	* @brief The Coriolis, centrifugal and gravity induced joint torque
	* for the current joint position and velocity.
	*/
      KDL::JntArray corioCentriTorque_;

      /**
	* @brief The friction induced joint torque
	* for the current joint velocity.
	*/
      KDL::JntArray frictionTorque_;

      /**
	* @brief The gravity induced joint torque
	* for the current joint position.
	*/
      KDL::JntArray gravityTorque_;

      /**
	* @brief The external wrench induced joint torque
	* for the current joint position.
	*/
//         KDL::JntArray externalWrenchTorque_;

      /**
	* @brief Prints information about the kdl chain
	*/
      void printChain();


      /**
	* @brief Returns the number of segments
	* @return The number of segments
	*/
      int getNrOfSegments();
      /**
	* @brief Returns the number of joints
	* @return The number of joints
	*/
      int getNrOfJoints();
      /**
	* @brief Gives the kdl segment corresponding to the index
	* @param[in] segment Index number of the segment
	* @param[out] kdl_segment The corresponding kdl segment
	*/
      const KDL::Segment & getSegment(unsigned int segment);

      /**
	* @brief Gives the kdl frame position corresponding to the index
	* @param[in] segment Index number of the segment
	* @param[out] kdl_frame The corresponding positon of the segment
	*/
      KDL::Frame & getSegmentPosition(unsigned int segment);

      /**
	* @brief Gives the kdl frame position corresponding to its name
	* @param[in] segment_name The segment's name
	* @param[out] kdl_frame The corresponding positon of the segment
	*/
      KDL::Frame & getSegmentPosition(std::string& segment_name);

      /**
	* @brief Gives the kdl twist corresponding to the index
	* @param[in] segment Index number of the segment
	* @param[out] kdl_twist The corresponding twist of the segment
	*/
      KDL::Twist & getSegmentVelocity(unsigned int segment);

      /**
	* @brief Gives the kdl twist corresponding to its name
	* @param[in] segment_name The segment's name
	* @param[out] kdl_twist The corresponding twist of the segment
	*/
      KDL::Twist & getSegmentVelocity(std::string& segment_name);

      /**
	* @brief Gives the jacobian expressed in the base frame
	*        with the reference point at the end of the segment
	* @param[in] segment Index number of the segment
	* @param[out] segment The corresponding jacobian of the segment
	*/
      KDL::Jacobian & getSegmentJacobian(unsigned int segment);

      /**
	* @brief Gives the jacobian expressed in the base frame
	*        with the reference point at the end of the segment
	* @param[in] segment_name The segment's name
	* @param[out] segment The corresponding jacobian of the segment
	*/
      KDL::Jacobian & getSegmentJacobian(const std::string& segment_name);

      /**
	* @brief Returns the segment name corresponding to the index
	* @param[in] index Index number of the segment
	* @return The segment's name
	*/
      KDL::Jacobian & getJacobian();
      KDL::Twist& getSegmentJdotQdot(unsigned int segment);
      KDL::Twist& getSegmentJdotQdot(const std::string& segment_name);
      const std::string& getSegmentName(unsigned int index);

      /**
	* @brief Returns the segment index corresponding to its name
	* @param[in] name The segment's name
	* @return The segment's index
	*/
      unsigned int getSegmentIndex(const std::string& name);

      /**
	* @brief Gets the joints limits from the URDF
	* @param[out] limited_joints The names of the joints that have joint limits
	* @param[out] lower_limits The lower limits
	* @param[out] upper_limits The upper limits
	*/
      void getJointLimits(std::vector<std::string>& limited_joints
          , std::vector<double>& lower_limits
          , std::vector<double>& upper_limits);

      std::vector<double>& getJointLowerLimits();
      std::vector<double>& getJointUpperLimits();
      std::vector< std::string >& getLimitedJointNames();

      /**
	* @brief Gives a kdl JntArray containing the joints position
	* @param[out] q The joints position
	*/
      KDL::JntArray & getJointPositions();

      /**
	* @brief Gives a kdl JntArray containing the joints velocity
	* @param[out] qd The joints velocity
	*/
      KDL::JntArray & getJointVelocities();

      /**
	* @brief Gives a kdl JntSpaceInertiaMatrix containing the mass matrix
	* @param[out] massMatrix The mass matrix
	*/
      KDL::JntSpaceInertiaMatrix & getInertiaMatrix();

      /**
	* @brief Gives a kdl JntArray containing the Coriolis torque
	* @param[out] corioCentriTorque The coriolis torque
	*/
      KDL::JntArray & getCoriolisTorque();

      /**
	* @brief Gives a kdl JntArray containing the gravitational torque
	* @param[out] gravityTorque The gravitational torque
	*/
      KDL::JntArray & getGravityTorque();

      /**
	* @brief Gives the JdotQdot of segment in a kdl twist
	* @param[in] segment_name The segment's name
	* @param[out] kdl_twist JdotQdot
	*/
      KDL::Twist & getJdotQdot();

    template<class T> void setState(const T& q,const T& qd)
    {
        setJointPositions(q);
        setJointVelocities(qd);
    }

    template<class T> void setJointPositions(const T& q)
    {
      this->outdate();
      for(unsigned int i=0; i<kdl_chain_.getNrOfJoints() && i<q.size(); i++){
        q_(i) = qqd_.q(i) = q[i];
      }
    }
      /**
	* @brief Sets the joint velocity of the model.
	* @param[in] qd_des the joint velocity in rad.
	*/
    template<class T> void setJointVelocities(const T& qd)
    {
        this->outdate();
        for(unsigned int i=0; i<kdl_chain_.getNrOfJoints() && i<qd.size(); i++){
          qd_(i) = qqd_.qdot(i) = qd[i];
        }
    }
    void updateModel();
      /**
	* @brief Sets the external measured wrench.
	* @param[in] f the external measured force.
	* @param[in] t the external measured torque.
	*/
      void setExternalMeasuredWrench(std::vector<double> &f, std::vector<double> &t);

      /**
	* @brief Sets the external point of measure of the wrench wrt to the last link.
	* @param[in] p the  coordinates of the point.
	*/
      void setExternalWrenchPoint(std::vector<double> &p);

      /**
	* @brief Computes the mass matrix of the model.
	*/
      void computeInertiaMatrix();

      /**
	* @brief Computes the Coriolis, centrifugal and gravity induced joint torque of the model.
	*/
      void computeCorioCentriTorque();

      /**
	* @brief Computes the gravity induced joint torque of the model.
	*/
      void computeGravityTorque();

      /**
	* @brief Computes the friction induced joint torque of the model.
	*/
//         void computeFrictionTorque();

      /**
	* @brief Computes the external wrench induced joint torque of the model.
	*/
//         void computeExternalWrenchTorque();

      /**
	* @brief Checks if the inertia matrix is outdated.
	* @return True if inertia matrix needs to be re computed.
	*/
      bool inertiaMatrixOutdated();

      /**
	* @brief Checks if the coriolis torque is outdated.
	* @return True if coriolis torque needs to be re computed.
	*/
      bool corioCentriTorqueOutdated();

      /**
	* @brief Checks if the gravity torque is outdated.
	* @return True if gravity torque needs to be re computed.
	*/
      bool gravityOutdated();
      void computeJacobian();
      void computeJdotQdot();
    private:
      KDL::Jacobian jacobian_,seg_jacobian_;
      KDL::JntArrayVel qqd_;
      KDL::Twist jdot_qdot_,seg_jdot_qdot_;
      KDL::Frame ee_pos_,seg_pos_;
      KDL::Twist ee_vel_,seg_vel_;
      KDL::FrameVel frame_vel_;
      /**
	* @brief The mapping between segment indice and segment name
	*/
      SegmentIndice seg_names_idx_;

      /**
	* @brief The current joint position.
	*/
      KDL::JntArray q_;

      /**
	* @brief The current joint velocity.
	*/
      KDL::JntArray qd_;

      /**
	* @brief The current external measured wrench.
	*/
      KDL::Wrench W_ext_;

      /**
	* @brief The point of application wrt to the last link
	* 	of the current external measured wrench.
	*/
      KDL::Vector W_ext_point_;

      /**
	* @brief The forward kinematic solver for position
	*/
      boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fksolver_;

      /**
	* @brief The forward kinematic solver for velocity
	*/
      boost::scoped_ptr<KDL::ChainFkSolverVel_recursive> fksolvervel_;

      /**
	* @brief The jacobian solver.
	*/
      boost::scoped_ptr<KDL::ChainJntToJacSolver> chainjacsolver_;

      /**
	* @brief The dynamic solver.
	*/
      boost::scoped_ptr<KDL::ChainDynParam> dynModelSolver_;

      /**
	* @brief The JdotQdot solver.
	*/
      boost::scoped_ptr<KDL::ChainJntToJacDotSolver> jntToJacDotSolver_;

      /**
	* @brief Boolean specifying if the inertia matrix is outdated
	*/
      bool inertiaMatrixOutdated_;

      /**
	* @brief Boolean specifying if the coriolis torque is outdated
	*/
      bool corioCentriTorqueOutdated_;

      /**
	* @brief Boolean specifying if the gravity torque is outdated
	*/
      bool gravityOutdated_;
      bool coriolisOutdated_;
      bool jacobianOutdated_;
      bool cartPosOutdated_;
      bool cartVelOutdated_;
      bool isOutdated_;
      bool isOutdated();
      /**
	* @brief Sets the gravity torque, coriolis torque and inertia to outdated
	*/
      void outdate();
  private:
        std::string robot_description_name;
        std::string root_link_name;
        std::string tip_link_name;
        KDL::Vector gravity_vector;
  };

}

#endif
