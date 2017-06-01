/*
 * rpGJKEPAAlgorithm.h
 *
 *  Created on: 14 янв. 2017 г.
 *      Author: wera
 */

#ifndef SOURCE_ENGIE_COLLISION_NARROWPHASE_GJK_EPA_RPGJKEPAALGORITHM_H_
#define SOURCE_ENGIE_COLLISION_NARROWPHASE_GJK_EPA_RPGJKEPAALGORITHM_H_

#include "../rpCollisionShapeInfo.h"
#include "rpNarrowPhaseCollisionAlgorithm.h"



namespace real_physics
{

class rpNarrowPhaseGjkEpaAlgorithm: public rpNarrowPhaseCollisionAlgorithm
{


	/// Private copy-constructor
    rpNarrowPhaseGjkEpaAlgorithm(const rpNarrowPhaseGjkEpaAlgorithm& algorithm);

	/// Private assignment operator
    rpNarrowPhaseGjkEpaAlgorithm& operator=(const rpNarrowPhaseGjkEpaAlgorithm& algorithm);


public:
             rpNarrowPhaseGjkEpaAlgorithm();
    virtual ~rpNarrowPhaseGjkEpaAlgorithm();


	/// Compute a contact info if the two bounding volume collide
    virtual bool testCollision(const rpCollisionShapeInfo &shape1Info,
                               const rpCollisionShapeInfo &shape2Info,
                               OutContactInfo& outInfo);


};

} /* namespace real_physics */

#endif /* SOURCE_ENGIE_COLLISION_NARROWPHASE_GJK_EPA_RPGJKEPAALGORITHM_H_ */
