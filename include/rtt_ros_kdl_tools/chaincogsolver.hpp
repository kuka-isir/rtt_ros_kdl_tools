/*
    Computes the Jacobian time derivative
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


#ifndef __KDL_CHAIN_CENTER_OF_GRAVITY_SOLVER__
#define __KDL_CHAIN_CENTER_OF_GRAVITY_SOLVER__

#include <kdl/solveri.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>

namespace KDL
{
    
class ChainCoGSolver : public SolverI
{

public:
    static const int E_COG_FAILED= -100;
    
    explicit ChainCoGSolver(const Chain& chain);
    virtual ~ChainCoGSolver();
    int JntToCoG(const JntArray& q_in,Vector& center_of_mass_out, int seg_nr=-1);

private:
    
    const Chain chain;
    std::vector<bool> locked_joints_;
    Vector cog;
    Frame pos;
};

}
#endif // __KDL_CHAIN_CENTER_OF_MASS_SOLVER__
