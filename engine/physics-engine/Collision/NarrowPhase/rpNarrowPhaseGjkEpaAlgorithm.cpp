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
#include "MPR/rpMPR.h"

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

    return   GjkEpaCalcPenDepth(shape2Info ,
                                shape1Info ,
                                outInfo.m_normal,
                                outInfo.pALocal,
                                outInfo.pBLocal,
                                outInfo.m_penetrationDepth);

}

} /* namespace real_physics */
