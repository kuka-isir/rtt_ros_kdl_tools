/*
    Copyright (C) 2015  Antoine Hoarau <hoarau at isir.upmc.fr>

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

namespace KDL
{

class ChainJntToJacDotSolver : public SolverI
{

public:
    static const int E_JAC_DOT_FAILED= -100;
  
    explicit ChainJntToJacDotSolver(const Chain& chain);
    virtual ~ChainJntToJacDotSolver();
    virtual int JntToJacDot(const KDL::JntArrayVel& q_in, KDL::Twist& jac_dot_q_dot, int seg_nr = -1);
    int setLockedJoints(const std::vector<bool> locked_joints);

    /// @copydoc KDL::SolverI::strError()
    virtual const char* strError(const int error) const;
    
    private:
        const Chain chain;
        Twist t_tmp;
        Frame T_tmp;
        std::vector<bool> locked_joints_;
        unsigned int nr_of_unlocked_joints_;
        Frame F_i_im1;
        Frame F_0_i;
        std::vector<Rotation> Q;
        std::vector<Vector> a;
        std::vector<Rotation> P;
        std::vector<Vector> e;
        std::vector<Vector> w;
        std::vector<Vector> ed;
        std::vector<Vector> rd;
        std::vector<Vector> ud;
        std::vector<Twist> v;
        std::vector<Vector> r;
        Twist ud_ed;
        Twist udi_edi;
        Twist Ui_v_i1;
};

}
#endif
