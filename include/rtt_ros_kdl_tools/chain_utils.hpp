#ifndef CHAIN_UTILS
#define CHAIN_UTILS

#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

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
      ChainUtils();
    
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
      KDL::JntSpaceInertiaMatrix massMatrix_;

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
	* @brief Returns the number of segments
	* @return The number of segments
	*/
      int nbSegments();

      /**
	* @brief Returns the kdl segment corresponding to the index
	* @param[in] segment Index number of the segment
	* @return The corresponding kdl segment
	*/
      const KDL::Segment& getSegment(int segment);

      /**
	* @brief Returns the kdl frame position corresponding to the index
	* @param[in] segment Index number of the segment
	* @return The corresponding positon of the segment
	*/
      const KDL::Frame& getSegmentPosition(int segment);

      /**
	* @brief Returns the kdl frame position corresponding to its name
	* @param[in] segment_name The segment's name
	* @return The corresponding positon of the segment
	*/
      const KDL::Frame& getSegmentPosition(std::string& segment_name);

      /**
	* @brief Returns the kdl twist corresponding to the index
	* @param[in] segment Index number of the segment
	* @return The corresponding twist of the segment
	*/
      const KDL::Twist& getSegmentVelocity(int segment);

      /**
	* @brief Returns the kdl twist corresponding to its name
	* @param[in] segment_name The segment's name
	* @return The corresponding twist of the segment
	*/
      const KDL::Twist& getSegmentVelocity(std::string& segment_name);
      
      /**
	* @brief Returns the jacobian expressed in the base frame 
	*        with the reference point at the end of the segment
	* @param[in] segment Index number of the segment
	* @return The corresponding jacobian of the segment
	*/
      const KDL::Jacobian& getSegmentJacobian(int segment);

      /**
	* @brief Returns the jacobian expressed in the base frame 
	*        with the reference point at the end of the segment
	* @param[in] segment_name The segment's name
	* @return The corresponding jacobian of the segment
	*/
      const KDL::Jacobian& getSegmentJacobian(std::string& segment_name);

      /**
	* @brief Returns the segment name corresponding to the index
	* @param[in] index Index number of the segment
	* @return The segment's name
	*/
      const std::string& getSegmentName(int index);
      
      /**
	* @brief Returns the segment index corresponding to its name
	* @param[in] name The segment's name
	* @return The segment's index
	*/
      int getSegmentIndex(const std::string name);
      
      /**
	* @brief Gets the joints limits from the URDF
	* @param[out] limited_joints The names of the joints that have joint limits
	* @param[out] lower_limits The lower limits
	* @param[out] upper_limits The upper limits
	* @return True if the operation was successfull
	*/
      bool getJointLimits(std::vector<std::string>& limited_joints, std::vector<double>& lower_limits, std::vector<double>& upper_limits);
      
      /**
	* @brief Returns a kdl JntArray containing the joints position
	* @return The joints position
	*/
      const KDL::JntArray& getJointPositions();
      
      /**
	* @brief Returns a kdl JntArray containing the joints velocity
	* @return The joints velocity
	*/
      const KDL::JntArray& getJointVelocities();

      /**
	* @brief Returns a kdl JntSpaceInertiaMatrix containing the mass matrix
	* @return The mass matrix
	*/
      const KDL::JntSpaceInertiaMatrix& getInertiaMatrix();
      
      /**
	* @brief Returns a kdl JntArray containing the Coriolis torque
	* @return The coriolis torque
	*/
      const KDL::JntArray& getCoriolisTorque();
      
      /**
	* @brief Returns a kdl JntArray containing the gravitational torque
	* @return The gravitational torque
	*/
      const KDL::JntArray& getGravityTorque();
      
      /**
	* @brief Returns the JdotQdot of segment in a kdl twist
	* @param[in] segment_name The segment's name
	* @return JdotQdot
	*/
      const KDL::Twist& getJdotQdot(std::string& segment_name);

      /**
	* @brief Returns the JdotQdot of segment in a kdl twist
	* @param[in] segment The index corresponding to the segment
	* @return JdotQdot
	*/
      const KDL::Twist& getJdotQdot(int segment);

      /**
	* @brief Sets the joint position of the model.
	* @param[in] q_des the joint position in rad.
	*/
      void setJointPosition(std::vector<double> &q_des);

      /**
	* @brief Sets the joint velocity of the model.
	* @param[in] qd_des the joint velocity in rad.
	*/
      void setJointVelocity(std::vector<double> &qd_des);
      
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
      void computeMassMatrix();
      
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

    private:
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
      KDL::ChainFkSolverPos_recursive* fksolver_;
      
      /**
	* @brief The forward kinematic solver for velocity
	*/
      KDL::ChainFkSolverVel_recursive* fksolvervel_;

      /**
	* @brief The jacobian solver.
	*/
      KDL::ChainJntToJacSolver* chainjacsolver_;
      
      /**
	* @brief The dynamic solver.
	*/
      KDL::ChainDynParam* dynModelSolver_;
      
      /**
	* @brief The JdotQdot solver.
	*/
      KDL::ChainJntToJacDotSolver* jntToJacDotSolver_;

//         Eigen::VectorXd actuatedDofs_;
//         Eigen::VectorXd lowerLimits_;
//         Eigen::VectorXd upperLimits_;
      
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

      /**
	* @brief Sets the gravity torque, coriolis torque and inertia to outdated
	*/
      void outdate();

  };
    
}

#endif
