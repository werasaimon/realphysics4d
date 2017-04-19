/*
 * rpNarrowPhaseAlgorithm.h
 *
 *  Created on: 24 нояб. 2016 г.
 *      Author: wera
 */

#ifndef SOURCE_ENGIE_COLLISION_NARROWPHASE_RPNARROWPHASECOLLISIONALGORITHM_H_
#define SOURCE_ENGIE_COLLISION_NARROWPHASE_RPNARROWPHASECOLLISIONALGORITHM_H_


namespace real_physics
{
	struct CollisionShapeInfo;
	class  OutContactInfo;
	class  rpOverlappingPair;
} /* namespace real_physics */

namespace real_physics
{

// Class NarrowPhaseAlgorithm
/**
 * This abstract class is the base class for a  narrow-phase collision
 * detection algorithm. The goal of the narrow phase algorithm is to
 * compute information about the contact between two proxy shapes.
 */
class NarrowPhaseCollisionAlgorithm
{

    protected :

        // -------------------- Attributes -------------------- //

        /// Overlapping pair of the bodies currently tested for collision
        rpOverlappingPair* mCurrentOverlappingPair;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        NarrowPhaseCollisionAlgorithm(const NarrowPhaseCollisionAlgorithm& algorithm);

        /// Private assignment operator
        NarrowPhaseCollisionAlgorithm& operator=(const NarrowPhaseCollisionAlgorithm& algorithm);

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        NarrowPhaseCollisionAlgorithm();

        /// Destructor
        virtual ~NarrowPhaseCollisionAlgorithm();


        /// Set the current overlapping pair of bodies
        void setCurrentOverlappingPair(rpOverlappingPair* overlappingPair);



        /// Compute a contact info if the two bounding volume collide
        virtual bool testCollision(const CollisionShapeInfo& shape1Info,
        	                       const CollisionShapeInfo& shape2Info,
				                   OutContactInfo& _outInfo) = 0;
};

// Set the current overlapping pair of bodies
inline void NarrowPhaseCollisionAlgorithm::setCurrentOverlappingPair(rpOverlappingPair* overlappingPair)
{
    mCurrentOverlappingPair = overlappingPair;
}

} /* namespace real_physics */

#endif /* SOURCE_ENGIE_COLLISION_NARROWPHASE_RPNARROWPHASECOLLISIONALGORITHM_H_ */
