/*
 * rpGJKEPAAlgorithm.h
 *
 *  Created on: 14 янв. 2017 г.
 *      Author: wera
 */

#ifndef SOURCE_ENGIE_COLLISION_NARROWPHASE_GJK_EPA_RPGJKEPAALGORITHM_H_
#define SOURCE_ENGIE_COLLISION_NARROWPHASE_GJK_EPA_RPGJKEPAALGORITHM_H_

#include "../EPA/rpEPAAlgorithm.h"
#include "../rpNarrowPhaseCollisionAlgorithm.h"
#include "rpComputeGjkEpaPenetration.h"
#include "rpGjkCollisionDescription.h"

namespace real_physics
{

class rpGJK_EPAAlgorithm: public NarrowPhaseCollisionAlgorithm
{


	/// Private copy-constructor
	rpGJK_EPAAlgorithm(const rpGJK_EPAAlgorithm& algorithm);

	/// Private assignment operator
	rpGJK_EPAAlgorithm& operator=(const rpGJK_EPAAlgorithm& algorithm);


public:
	         rpGJK_EPAAlgorithm();
	virtual ~rpGJK_EPAAlgorithm();


	/// Compute a contact info if the two bounding volume collide
	virtual bool testCollision(const CollisionShapeInfo& shape1Info,
			                   const CollisionShapeInfo& shape2Info,
			                   OutContactInfo& outInfo)
	{

		btGjkCollisionDescription inputColleDscp;
		//inputColleDscp.m_maxGjkIterations = 10;
		//inputColleDscp.m_maximumDistanceSquared = 0.0001;
		return btGjkEpaCalcPenDepth(shape2Info ,
									shape1Info ,
									inputColleDscp,
									outInfo.m_normal,
									outInfo.pALocal,
									outInfo.pBLocal,
									outInfo.m_penetrationDepth);

	}
};

} /* namespace real_physics */

#endif /* SOURCE_ENGIE_COLLISION_NARROWPHASE_GJK_EPA_RPGJKEPAALGORITHM_H_ */
