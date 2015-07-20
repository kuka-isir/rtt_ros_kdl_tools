/*
    <one line to give the library's name and an idea of what it does.>
    Copyright (C) 2015  <copyright holder> <email>

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


#include "rtt_ros_kdl_tools/chainjnttojacdotsolver.hpp"

#include <kdl/kinfam_io.hpp>
#include <kdl/frames_io.hpp>

namespace KDL
{
ChainJntToJacDotSolver::ChainJntToJacDotSolver(const Chain& _chain):
chain(_chain),
locked_joints_(_chain.getNrOfJoints(),false),
nr_of_unlocked_joints_(_chain.getNrOfJoints()),
Q(_chain.getNrOfSegments()),
a(_chain.getNrOfSegments()),
P(_chain.getNrOfSegments()),
e(_chain.getNrOfSegments()),
w(_chain.getNrOfSegments()),
ed(_chain.getNrOfSegments()),
rd(_chain.getNrOfSegments()),
ud(_chain.getNrOfSegments()),
v(_chain.getNrOfSegments()),
r(_chain.getNrOfSegments()),
F_i_im1(Frame::Identity()),
F_0_i(Frame::Identity())
{

}
int ChainJntToJacDotSolver::JntToJacDot(const JntArrayVel& q_in, Twist& jac_dot_q_dot, int seg_nr)
{
    unsigned int segmentNr;
    if(seg_nr<0)
        segmentNr=chain.getNrOfSegments();
    else
        segmentNr = seg_nr;

    if(q_in.q.rows()!=chain.getNrOfJoints())
        return (error = E_JAC_DOT_FAILED);
    else if(segmentNr>chain.getNrOfSegments())
        return (error = E_JAC_DOT_FAILED);

    int j=0;
    F_0_i = Frame::Identity();
    
    j = 0;
    for (unsigned int i=0;i<segmentNr;++i)
    {
      if(chain.getSegment(i).getJoint().getType()!=Joint::None)
      {
        F_i_im1 = chain.getSegment(i).pose(q_in.q(j)); //Fk
        j++;
      }else{
        F_i_im1 = chain.getSegment(i).pose(0.0);
      }
      F_0_i = F_0_i * F_i_im1;
      
      P[i] = F_0_i.M;
      e[i] = Vector(P[i](0,2) , P[i](1,2), P[i](2,2));
      
      Q[i] = F_i_im1.M;
      a[i] = chain.getSegment(i).getFrameToTip().p;
    }
    
    // Step 1
    j=0;
    if(chain.getSegment(0).getJoint().getType()!=Joint::None){
      w[0] = q_in.qdot(0) * e[0];
      j++;
    }else{
      w[0] = Vector(0,0,0) * e[0];
    }
    for (unsigned int i=0;i<segmentNr-1;++i)
    {
      if(chain.getSegment(i+1).getJoint().getType()!=Joint::None){
        w[i+1] = q_in.qdot(j) * e[i] + Q[i].Inverse() * w[i];
        j++;
      }else{
        w[i+1] = Q[i].Inverse() * w[i];
      }
    }

    // Step 2
    for (unsigned int i=0;i<segmentNr;++i)
    {
      ed[i] = w[i] * e[i];
    }
    
    // Step 3
    rd[segmentNr-1] = w[segmentNr-1] * a[segmentNr-1];
    for (int i=segmentNr-2;i>=0;i--)
    {
      rd[i] = w[i] * a[i] + Q[i] * rd[i+1];
    }
    
    r[segmentNr-1] = a[segmentNr-1];
    for (int i=segmentNr-2;i>=0;i--)
    {
      r[i] = a[i] + Q[i] * r[i+1];
    }
     
    ud[0] = e[0] * rd[0];
    for (unsigned int i=1;i<segmentNr;++i)
    {
      ud[i] = ed[i] * r[i] + e[i] * rd[i];
      
    }

    // Step 4
      
    j = chain.getNrOfJoints() - 1;
    
    ud_ed.vel = ud[segmentNr-1];
    ud_ed.rot = ed[segmentNr-1];
    
    if(chain.getSegment(segmentNr-1).getJoint().getType()!=Joint::None){
      v[segmentNr-1] = q_in.qdot(j) * ud_ed;
      j--;
    }else{
      v[segmentNr-1] = 0.0 * ud_ed;
    }
    
    for (int i=segmentNr-2;i>=0;i--)
    {
      Ui_v_i1.vel = Q[i] * v[i+1].vel;
      Ui_v_i1.rot = Q[i] * v[i+1].rot;
      udi_edi.vel = ud[i];
      udi_edi.rot = ed[i];
            
      if(chain.getSegment(i).getJoint().getType()!=Joint::None){
        v[i] = q_in.qdot(j) * udi_edi + Ui_v_i1;
        j--;
      }else{
        v[i] = Ui_v_i1;
      }
    }
 
    jac_dot_q_dot = v[0];
    return (error = E_NOERROR);
}
int ChainJntToJacDotSolver::setLockedJoints(const std::vector< bool > locked_joints)
{
    if(locked_joints.size()!=locked_joints_.size())
        return -1;
    locked_joints_=locked_joints;
    nr_of_unlocked_joints_=0;
    for(unsigned int i=0;i<locked_joints_.size();i++){
        if(!locked_joints_[i])
            nr_of_unlocked_joints_++;
    }

    return 0;
}
const char* ChainJntToJacDotSolver::strError(const int error) const
{
        if (E_JAC_DOT_FAILED == error) return "Jac Dot Failed";
        else return SolverI::strError(error);
}

ChainJntToJacDotSolver::~ChainJntToJacDotSolver()
{

}
}
