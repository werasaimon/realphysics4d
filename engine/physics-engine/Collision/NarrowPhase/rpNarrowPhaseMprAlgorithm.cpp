#include "rpNarrowPhaseMprAlgorithm.h"
#include "GJK_EPA/rpComputeGjkEpaPenetration.h"
#include "GJK_EPA/rpGjkEpa.h"
#include "MPR/rpMPRAlgorithm.h"
#include "GJK/rpGJKAlgorithm.h"


namespace real_physics
{

rpNarrowPhaseMprAlgorithm::rpNarrowPhaseMprAlgorithm()
{

}

rpNarrowPhaseMprAlgorithm::~rpNarrowPhaseMprAlgorithm()
{

}

bool rpNarrowPhaseMprAlgorithm::testCollision(const rpCollisionShapeInfo &shape1Info,
                                              const rpCollisionShapeInfo &shape2Info, OutContactInfo &outInfo)
{


    rpMPRAlgorithm MPR;
    return MPR.ComputeMprColliderPenetration( &shape1Info , &shape2Info , outInfo);


    /**
    GJKAlgorithm GJK;
    bool isGJK = GJK.computeGJK( shape2Info , shape1Info , outInfo );

    if(isGJK)
    {
         OutContactInfo outInfoA;
       if( rpMPR::ComputeMprColliderPenetration( &shape2Info , &shape1Info , outInfoA ))
       {
           outInfo = outInfoA;
       }
    }
    else
    {
        return isGJK;
    }
    /**/

}

}
