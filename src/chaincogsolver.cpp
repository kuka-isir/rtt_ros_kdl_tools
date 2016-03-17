#include <rtt_ros_kdl_tools/chaincogsolver.hpp>

namespace KDL{

ChainCoGSolver::ChainCoGSolver(const KDL::Chain& chain_):
chain(chain_)
{

}

int ChainCoGSolver::JntToCoG(const KDL::JntArray& q_in, KDL::Vector& center_of_mass_out, int seg_nr)
{
    unsigned int segmentNr;
    
    if(seg_nr<0)
        segmentNr = chain.getNrOfSegments();
    else
        segmentNr = seg_nr;

    if(q_in.rows()!=chain.getNrOfJoints())
        return -1;
    else if(segmentNr>chain.getNrOfSegments())
        return -1;
    else if(segmentNr == 0)
        return -1;
    
    if(q_in.rows() != chain.getNrOfJoints())
        return E_COG_FAILED;
    
    int j=0;
    
    SetToZero(cog);
    SetToZero(center_of_mass_out);
    pos = KDL::Frame::Identity();
    double total_mass = 0.0;
    double current_mass = 0.0;
    
    for(unsigned int i=0;i<segmentNr;i++){
        // Calculate Forward kinematics
        if(chain.getSegment(i).getJoint().getType()!=Joint::None){
            // Frame(i) -> Frame(i+1)
            pos = pos*chain.getSegment(i).pose(q_in(j));
            
            current_mass = chain.getSegment(i).getInertia().getMass();
            
            cog += current_mass * (pos * chain.getSegment(i).getInertia().getCOG());
             
            total_mass += current_mass;

            j++;
        }else{
            pos = pos*chain.getSegment(i).pose(0.0);
        }
    }
    center_of_mass_out = 1.0/total_mass *  cog;
    return 0;
}

ChainCoGSolver::~ChainCoGSolver()
{

}

} // KDL

