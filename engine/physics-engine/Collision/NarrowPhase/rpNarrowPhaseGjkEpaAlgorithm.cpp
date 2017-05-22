/*
 * rpGJKEPAAlgorithm.cpp
 *
 *  Created on: 14 янв. 2017 г.
 *      Author: wera
 */

#include "rpNarrowPhaseGjkEpaAlgorithm.h"
#include "GJK_EPA/rpGjkCollisionDescription.h"
#include "GJK_EPA/rpComputeGjkEpaPenetration.h"
#include "GJK_EPA/rpGjkEpa.h"
#include "GJK/rpGJKAlgorithm.h"
#include "MPR/rpMPRAlgorithm.h"

namespace real_physics
{

rpNarrowPhaseGjkEpaAlgorithm::rpNarrowPhaseGjkEpaAlgorithm()
{
	// TODO Auto-generated constructor stub

}

rpNarrowPhaseGjkEpaAlgorithm::~rpNarrowPhaseGjkEpaAlgorithm()
{
    // TODO Auto-generated destructor stub
}

bool rpNarrowPhaseGjkEpaAlgorithm::testCollision(const rpCollisionShapeInfo &shape1Info,
                                                 const rpCollisionShapeInfo &shape2Info,
                                                 OutContactInfo &outInfo)
{


        return   GjkEpaCalcPenDepth(shape1Info ,
                                    shape2Info ,
                                    outInfo.m_normal,
                                    outInfo.pALocal,
                                    outInfo.pBLocal,
                                    outInfo.m_penetrationDepth);


/**
    Vector3	guessVector(shape1Info.getWorldTransform().getPosition() -
                        shape2Info.getWorldTransform().getPosition());//?? why not use the GJK input?

    rpGjkEpaSolver::sResults	results;

    if(rpGjkEpaSolver::Penetration(shape2Info,shape1Info,guessVector,results))
    {
        outInfo.pALocal = results.witnesses[0];
        outInfo.pBLocal = results.witnesses[1];

        outInfo.m_normal = results.normal;
        outInfo.m_penetrationDepth  = results.distance;

        return true;

    }
    else
    {
        if(rpGjkEpaSolver::Distance(shape2Info,shape1Info,guessVector,results))
        {
            outInfo.pALocal = results.witnesses[0];
            outInfo.pBLocal = results.witnesses[1];

            outInfo.m_normal = results.normal;
            outInfo.m_penetrationDepth  = results.distance;

            return false;
        }     
    }


    return false;
/**/


}

} /* namespace real_physics */
