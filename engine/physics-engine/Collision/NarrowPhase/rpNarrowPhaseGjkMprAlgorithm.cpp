#include "rpNarrowPhaseGjkMprAlgorithm.h"
#include "GJK_EPA/rpComputeGjkEpaPenetration.h"
#include "GJK_EPA/rpGjkEpa.h"
#include "MPR/rpMPR.h"


namespace real_physics
{

rpNarrowPhaseGjkMprAlgorithm::rpNarrowPhaseGjkMprAlgorithm()
{

}

rpNarrowPhaseGjkMprAlgorithm::~rpNarrowPhaseGjkMprAlgorithm()
{

}

bool rpNarrowPhaseGjkMprAlgorithm::testCollision(const rpCollisionShapeInfo &shape1Info,
                                                 const rpCollisionShapeInfo &shape2Info, OutContactInfo &outInfo)
{

    rpGjkEpaSolver::sResults	results;
    Vector3	guessVector(shape1Info.getWorldTransform().getPosition() -
                        shape1Info.getWorldTransform().getPosition());//?? why not use the GJK input?

    if(rpMPR::ComputeMprColliderPenetration( &shape2Info , &shape1Info , outInfo ))
    {
        return true;
    }
    else
    {
        if(rpGjkEpaSolver::Distance(shape1Info,shape2Info,guessVector,results))
        {
            outInfo.pALocal = results.witnesses[0];
            outInfo.pBLocal = results.witnesses[1];

            outInfo.m_normal = results.normal;
            outInfo.m_penetrationDepth = results.distance;

            return false;
        }
    }

    return false;

}

}
