/*
 * rpNarrowPhaseAlgorithm.h
 *
 *  Created on: 24 нояб. 2016 г.
 *      Author: wera
 */

#ifndef SOURCE_ENGIE_COLLISION_NARROWPHASE_RPNARROWPHASECOLLISIONALGORITHM_H_
#define SOURCE_ENGIE_COLLISION_NARROWPHASE_RPNARROWPHASECOLLISIONALGORITHM_H_

#include "../../LinearMaths/mathematics.h"

namespace real_physics
{
    struct rpCollisionShapeInfo;
	class  OutContactInfo;
	class  rpOverlappingPair;
} /* namespace real_physics */

namespace real_physics
{



class OutContactInfo
{

    public:

        //-------- Attribute --------//
        Vector3 m_normal;
        scalar  m_penetrationDepth;
        Vector3 pALocal;
        Vector3 pBLocal;

        OutContactInfo()
        {

        }

        OutContactInfo( const Vector3& _normal ,  const scalar&  _depth ,
                        const Vector3& _ALocal ,
                        const Vector3& _BLocal )
            :m_normal(_normal),
             m_penetrationDepth(_depth),
             pALocal(_ALocal),
             pBLocal(_BLocal)
        {

        }


};





// Class NarrowPhaseAlgorithm
/**
 * This abstract class is the base class for a  narrow-phase collision
 * detection algorithm. The goal of the narrow phase algorithm is to
 * compute information about the contact between two proxy shapes.
 */
class rpNarrowPhaseCollisionAlgorithm
{

    protected :

        // -------------------- Attributes -------------------- //

        /// Overlapping pair of the bodies currently tested for collision
        rpOverlappingPair* mCurrentOverlappingPair;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        rpNarrowPhaseCollisionAlgorithm(const rpNarrowPhaseCollisionAlgorithm& algorithm);

        /// Private assignment operator
        rpNarrowPhaseCollisionAlgorithm& operator=(const rpNarrowPhaseCollisionAlgorithm& algorithm);

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        rpNarrowPhaseCollisionAlgorithm();

        /// Destructor
        virtual ~rpNarrowPhaseCollisionAlgorithm();


        /// Set the current overlapping pair of bodies
        void setCurrentOverlappingPair(rpOverlappingPair* overlappingPair);



        /// Compute a contact info if the two bounding volume collide
        virtual bool testCollision(const rpCollisionShapeInfo& shape1Info,
                                   const rpCollisionShapeInfo& shape2Info,
				                   OutContactInfo& _outInfo) = 0;
};

// Set the current overlapping pair of bodies
SIMD_INLINE void rpNarrowPhaseCollisionAlgorithm::setCurrentOverlappingPair(rpOverlappingPair* overlappingPair)
{
    mCurrentOverlappingPair = overlappingPair;
}

} /* namespace real_physics */

#endif /* SOURCE_ENGIE_COLLISION_NARROWPHASE_RPNARROWPHASECOLLISIONALGORITHM_H_ */
