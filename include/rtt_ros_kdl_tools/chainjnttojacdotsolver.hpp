/*
    Computes the Jacobian time derivative in the hybrid representation
    Copyright (C) 2015  Antoine Hoarau <hoarau [at] isir.upmc.fr>

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/


#ifndef KDL_CHAINJNTTOJACDOTSOLVER_HPP
#define KDL_CHAINJNTTOJACDOTSOLVER_HPP

#include "kdl/solveri.hpp"
#include "kdl/frames.hpp"
#include "kdl/jntarrayvel.hpp"
#include "kdl/jacobian.hpp"
#include "kdl/chain.hpp"
#include "kdl/framevel.hpp"
#include "kdl/chainjnttojacsolver.hpp"

namespace KDL
{

class ChainJntToJacDotSolver : public SolverI
{
/*
 * Computes the Jacobian time derivative (Jdot) by calculating the partial derivatives
 * regarding to a joint angle, in the "Hybrid" representation. This is based on : 
 * 
 * Symbolic differentiation of the velocity mapping for a serial kinematic chain
 * H. Bruyninckx, J. De Schutter
 * doi:10.1016/0094-114X(95)00069-B
 * 
 * url : http://www.sciencedirect.com/science/article/pii/0094114X9500069B
*/
public:
    static const int E_JAC_DOT_FAILED= -100;
  
    explicit ChainJntToJacDotSolver(const Chain& chain);
    virtual ~ChainJntToJacDotSolver();
    /**
     * @brief Computes \f$ {}_{bs}\dot{J}^{ee}.\dot{q} \f$
     * 
     * @param q_in Current joint positions and velocities
     * @param jac_dot_q_dot The twist representing Jdot*qdot
     * @param seg_nr The final segment to compute
     * @return int 0 if no errors happened 
     */
    virtual int JntToJacDot(const KDL::JntArrayVel& q_in, KDL::Twist& jac_dot_q_dot, int seg_nr = -1);
    /**
     * @brief Computes \f$ {}_{bs}\dot{J}^{ee} \f$ 
     * 
     * @param q_in Current joint positions and velocities
     * @param jdot The jacobian time derivative in Hybrid representation 
     * (i.e. with the base frame as the reference frame and the end effector frame 
     * as the velocity reference frame 
     * @param seg_nr The final segment to compute
     * @return int 0 if no errors happened
     */
    virtual int JntToJacDot(const KDL::JntArrayVel& q_in, KDL::Jacobian& jdot, int seg_nr = -1);
    int setLockedJoints(const std::vector<bool> locked_joints);
    /**
     * @brief Computes \f$ \frac{\partial J^{i,ee}}{\partial q^{j}}.\dot{q}^{j} \f$
     * 
     * @param bs_J_ee Current joint positions and velocities
     * @param joint_idx The indice of the current joint (j in the formula)
     * @param column_idx The indice of the current column (i in the formula)
     * @return Twist The twist representing dJi/dqj .qdotj
     */
    Twist getPartialDerivativeHybrid(const Jacobian& bs_J_ee,const unsigned int& joint_idx,const unsigned int& column_idx);

    /// @copydoc KDL::SolverI::strError()
    virtual const char* strError(const int error) const;
    
    private:
        const Chain chain;
        std::vector<bool> locked_joints_;
        unsigned int nr_of_unlocked_joints_;
        ChainJntToJacSolver jac_solver_;
        Jacobian bs_J_ee_;
        Jacobian bs_Jdot_ee_;
};

}
#endif
