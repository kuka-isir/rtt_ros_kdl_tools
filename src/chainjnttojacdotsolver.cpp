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


#include "rtt_ros_kdl_tools/chainjnttojacdotsolver.hpp"
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/kinfam_io.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/framevel_io.hpp>

namespace KDL
{
inline void SetToZero(Rotation& R)
{
    for(int i=0;i<9;++i)
                R.data[i] = 0;   
}

class SkewSymmetric : public Rotation
{
    public : SkewSymmetric(const Vector& w)
    {
        SetDiagToZero();
        SetSkewVector(w);
    }
    public : SkewSymmetric(){SetToZero(*this);}
    public : void SetSkewVector(const Vector& w)
    {
            this->data[1] = - w.z();
            this->data[2] =   w.y();
            this->data[3] =   w.z();
            this->data[5] = - w.x();
            this->data[6] = - w.y();
            this->data[7] =   w.x();    
    }
    private : friend void SetToZero(Rotation& R);
    private : void SetDiagToZero()
    {
        for(int i=0;i<3;++i)
                this->data[i+3*i] = 0.;
    }
};

class Mat6x6
{
public :
    Mat6x6(const Mat6x6& M)
    {
        this->data = M.data;
    }
    Mat6x6(const Eigen::Matrix<double,6,6>& data_eigen)
    {
        this->data = data_eigen;
    }
    Mat6x6()
    {
        data.setZero();
    }
    Mat6x6(const Rotation& R_0_0,const Rotation& R_0_3,const Rotation& R_3_0,const Rotation& R_3_3)
    {
        SetUpperLeftCorner(R_0_0);
        SetUpperRightCorner(R_0_3);
        SetLowerLeftCorner(R_3_0);
        SetLowerRightCorner(R_3_3); 
    }
    
    void SetUpperLeftCorner(const Rotation& R){ SetBlock(0,0,3,3,R); }
    void SetUpperRightCorner(const Rotation& R){ SetBlock(0,3,3,3,R); }
    void SetLowerLeftCorner(const Rotation& R){ SetBlock(3,0,3,3,R); }
    void SetLowerRightCorner(const Rotation& R){ SetBlock(3,3,3,3,R); }
    
    void SetBlock(const unsigned& i,const unsigned& j,
                  const unsigned& p,const unsigned& q,
                  const Rotation& R)
    {
        data.block(i,j,p,q) = Eigen::Matrix<double,3,3>::Map(R.data);
    }
    inline friend void SetToZero(Mat6x6& M);
    Mat6x6 operator-(){return Mat6x6(-this->data);}
    inline friend Jacobian operator*(const Mat6x6& M,const Jacobian& J);
    Eigen::Matrix<double,6,6> data;
};    

class Mat3x3 : public Rotation
{
public:
    Mat3x3()
    {
        SetToZero(*this);
    }
    static Mat3x3 Zero(){return Mat3x3();}
};

inline Jacobian operator*(const Mat6x6& M,const Jacobian& J)
{
    Jacobian J_tmp(J.columns());
    J_tmp.data = M.data * J.data;
    return J_tmp;
}

inline Twist operator*(const Mat6x6& M,const Twist& t)
{
    Eigen::Matrix<double,6,1> tw,tw_out;
    for(int i=0;i<6;i++)
        tw(i) = t(i);
    tw_out = M.data * tw;    
    return Twist(Vector(tw_out(0),tw_out(1),tw_out(2)),Vector(tw_out(3),tw_out(4),tw_out(5)));
}

inline void SetToZero(Mat6x6& M)
{
    M.data.setZero();
}

ChainJntToJacDotSolver::ChainJntToJacDotSolver(const Chain& _chain):
chain(_chain),
locked_joints_(chain.getNrOfJoints(),false),
nr_of_unlocked_joints_(chain.getNrOfJoints()),
jac_solver_(chain),
bs_J_ee_(chain.getNrOfJoints()),
bs_Jdot_ee_(chain.getNrOfJoints())
{

}

int ChainJntToJacDotSolver::JntToJacDot(const JntArrayVel& q_in, Twist& jac_dot_q_dot, int seg_nr)
{
    JntToJacDot(q_in,bs_Jdot_ee_,seg_nr);
    MultiplyJacobian(bs_Jdot_ee_,q_in.qdot,jac_dot_q_dot);
    return (error = E_NOERROR);
}

int ChainJntToJacDotSolver::JntToJacDot(const JntArrayVel& q_in, Jacobian& jdot, int seg_nr)
{
        unsigned int segmentNr;
        if(seg_nr<0)
            segmentNr=chain.getNrOfSegments();
        else
            segmentNr = seg_nr;

        //Initialize Jacobian to zero since only segmentNr colunns are computed
        SetToZero(jdot) ;

        if(q_in.q.rows()!=chain.getNrOfJoints()||nr_of_unlocked_joints_!=jdot.columns())
            return (error = E_JAC_DOT_FAILED);
        else if(segmentNr>chain.getNrOfSegments())
            return (error = E_JAC_DOT_FAILED);
                
        Twist J_dot_k;
        jac_solver_.JntToJac(q_in.q,bs_J_ee_);
        int k=0;
        for(unsigned int i=0;i<segmentNr;++i)
        {
            //Only increase joint nr if the segment has a joint
            if(chain.getSegment(i).getJoint().getType()!=Joint::None){
                
                for(unsigned int j=0;j<chain.getNrOfJoints();++j)
                {
                    // Column J is the sum of all partial derivatives  ref (41)
                    Twist dJdq = getPartialDerivativeHybrid(bs_J_ee_,j,k);
                    //std::cout << "dJ"<<k<<"dq"<<j<<" "<<dJdq<<std::endl;
                    //std::cout << "dJ"<<k<<"dq"<<j<<" * qdot"<<j<<dJdq * q_in.qdot(j)<<std::endl;
                    J_dot_k += dJdq * q_in.qdot(j);
                }
                jdot.setColumn(k++,J_dot_k);
                SetToZero(J_dot_k);
            }
        }
        
        return (error = E_NOERROR);
}
Twist ChainJntToJacDotSolver::getPartialDerivativeHybrid(const Jacobian& bs_J_ee,
                                      const unsigned int& joint_idx,
                                      const unsigned int& column_idx)
{
        int j=joint_idx;
        int i=column_idx;
        
        Twist bs_t_j_ee;
        SkewSymmetric bs_e_j_x;
        Mat6x6 P_d_bs_Jj;
        SkewSymmetric bs_v_j_ee_x;
        Mat6x6 M_d_J_j_ee;
        
        SetToZero(bs_t_j_ee);
        
        // ref (6)
        bs_e_j_x.SetSkewVector(bs_J_ee.getColumn(j).rot);
        
        // P_{\Delta}({}_{bs}J^{j})  ref (20)
        P_d_bs_Jj.SetUpperLeftCorner(bs_e_j_x);
        P_d_bs_Jj.SetLowerRightCorner(bs_e_j_x);
        
        bs_v_j_ee_x.SetSkewVector(bs_J_ee.getColumn(j).vel);
        
        // M_{\Delta}({}_{bs}J^{j})  ref (23)
        M_d_J_j_ee.SetUpperRightCorner(bs_v_j_ee_x);
        
        //std::cout << "******************** Calculating dJ"<<i<<"/dq"<<j<<std::endl;
        if(j < i)
        {
            //std::cout << "j < i" << std::endl;
            bs_t_j_ee = P_d_bs_Jj * bs_J_ee.getColumn(i);
            //std::cout << "P_d_bs_Jj_"<<i<<"\n"<<P_d_bs_Jj.data<<std::endl;
        }else if(j > i)
        {
            //std::cout << "j > i" << std::endl;
            bs_t_j_ee = -M_d_J_j_ee * bs_J_ee.getColumn(i);
            //std::cout << "M_d_J_j_ee_"<<i<<"\n"<<M_d_J_j_ee.data<<std::endl;
        }else if(j == i)
        {
             //std::cout << "j == i" << std::endl;
             SetToZero(bs_t_j_ee.rot);
             bs_t_j_ee.vel = bs_J_ee.getColumn(i).rot * (-bs_J_ee.getColumn(i).vel);
             //std::cout << "P_d_bs_Jj_"<<i<<"\n"<<P_d_bs_Jj.data<<std::endl;
             //std::cout << "Should be equal to : " << P_d_bs_Jj * bs_J_ee.getColumn(i)<<std::endl;
        }
        
        //std::cout <<"bs_J_ee_"<<i<<": "<< bs_J_ee.getColumn(i) << std::endl;
        // ! \\ TODO: Find out why we get minus the twist
        return -bs_t_j_ee;

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
