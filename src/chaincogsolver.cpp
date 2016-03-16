#include <rtt_ros_kdl_tools/chaincogsolver.hpp>

namespace KDL{

ChainCOGSolver::ChainCOGSolver(const KDL::Chain& chain_):
chain(chain_)
{

}

int ChainCOGSolver::JntToCOG(const KDL::JntArray& q_in, KDL::Vector& center_of_mass_out, int seg_nr)
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

    SetToZero(center_of_mass_out);
    if(q_in.rows() != chain.getNrOfJoints())
        return E_COG_FAILED;
    
    int j=0;
    
    //SetToIdentity(pos);
    double total_mass = 0.0;
    
    for(unsigned int i=0;i<segmentNr;i++){
        // Calculate Forward kinematics
        if(chain.getSegment(i).getJoint().getType()!=Joint::None){
            // Frame(i) -> Frame(i+1)
            pos = pos*chain.getSegment(i).pose(q_in(j));
            cog = cog + pos * chain.getSegment(i).getInertia().getCOG();
            total_mass += chain.getSegment(i).getInertia().getMass();
            j++;
        }else{
            pos = pos*chain.getSegment(i).pose(0.0);
        }
    }
}

ChainCOGSolver::~ChainCOGSolver()
{

}

} // KDL

