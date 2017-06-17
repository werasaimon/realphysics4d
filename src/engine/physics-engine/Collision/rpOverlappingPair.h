/*
 * rpOverlappingPair.h
 *
 *  Created on: 23 нояб. 2016 г.
 *      Author: wera
 */

#ifndef SOURCE_ENGIE_COLLISION_RPOVERLAPPINGPAIR_H_
#define SOURCE_ENGIE_COLLISION_RPOVERLAPPINGPAIR_H_

#include <utility>

#include "../LinearMaths/mathematics.h"
#include "../Memory/memory.h"
#include "Manifold/rpContactManifoldSet.h"
#include  "rpProxyShape.h"
#include  "Shapes/rpCollisionShape.h"


namespace real_physics
{


// Type for the overlapping pair ID
typedef std::pair<uint, uint> overlappingpairid;

/**
 * This class represents a pair of two proxy collision shapes that are overlapping
 * during the broad-phase collision detection. It is created when
 * the two proxy collision shapes start to overlap and is destroyed when they do not
 * overlap anymore. This class contains a contact manifold that
 * store all the contact points between the two bodies.
 */
class rpOverlappingPair
{

    private:

        // -------------------- Attributes -------------------- //

	   /// Set of persistent contact manifolds
	    rpContactManifoldSet mContactManifoldSet;


		rpProxyShape* mShape1;
		rpProxyShape* mShape2;

        /// Cached previous separating axis
        Vector3 mCachedSeparatingAxis;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        rpOverlappingPair(const rpOverlappingPair& pair);

        /// Private assignment operator
        rpOverlappingPair& operator=(const rpOverlappingPair& pair);

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        rpOverlappingPair( rpProxyShape* shape1, rpProxyShape* shape2 , int nbMaxContactManifolds = 1);

        /// Destructor
        ~rpOverlappingPair();

        /// Return the pointer to first proxy collision shape
        rpProxyShape* getShape1() const;

        /// Return the pointer to second body
        rpProxyShape* getShape2() const;


		//        /// Add a contact to the contact cache
		//        void addContact(ContactPoint* contact);
		//
		//        /// Update the contact cache
		//        void update();



        /// Return the cached separating axis
        Vector3 getCachedSeparatingAxis() const;

        /// Set the cached separating axis
        void setCachedSeparatingAxis(const Vector3& axis);



        /// Return the number of contacts in the cache
        int getNbContactPoints() const;

        /// Return the a reference to the contact manifold set
        const rpContactManifoldSet& getContactManifoldSet() const;



        rpContactManifoldSet getContactManifoldSet2() const
        {
        	return mContactManifoldSet;
        }


		//        /// Clear the contact points of the contact manifold
		//        void clearContactPoints();


        /// Return the pair of bodies index
        static overlappingpairid computeID(rpProxyShape* shape1,
        		                           rpProxyShape* shape2);

        /// Return the pair of bodies index of the pair
        static bodyindexpair computeBodiesIndexPair( rpCollisionBody* body1,
        		                                     rpCollisionBody* body2);

        // -------------------- Friendship -------------------- //

        friend class rpContactManager;
        friend class rpCollisionWorld;
        friend class rpDynamicsWorld;

};

// Return the pointer to first body
SIMD_INLINE  rpProxyShape* rpOverlappingPair::getShape1() const
{
    return mShape1;
}
// Return the pointer to first body
SIMD_INLINE  rpProxyShape* rpOverlappingPair::getShape2() const
{
    return mShape2;
}



// Return the cached separating axis
SIMD_INLINE  Vector3 rpOverlappingPair::getCachedSeparatingAxis() const
{
    return mCachedSeparatingAxis;
}

// Set the cached separating axis
SIMD_INLINE  void rpOverlappingPair::setCachedSeparatingAxis(const Vector3& axis)
{
    mCachedSeparatingAxis = axis;
}




SIMD_INLINE int rpOverlappingPair::getNbContactPoints() const
{
	return mContactManifoldSet.getNbContactManifolds();
}

SIMD_INLINE const rpContactManifoldSet& rpOverlappingPair::getContactManifoldSet() const
{
	 return mContactManifoldSet;
}



// Return the pair of bodies index
SIMD_INLINE  bodyindexpair rpOverlappingPair::computeBodiesIndexPair(rpCollisionBody* body1,
                                                                     rpCollisionBody* body2)
{
    // Construct the pair of body index
    bodyindexpair indexPair = body1->getID() < body2->getID() ?
                                 std::make_pair(body1->getID(), body2->getID()) :
                                 std::make_pair(body2->getID(), body1->getID());
    assert(indexPair.first != indexPair.second);
    return indexPair;
}


// Return the pair of bodies index
SIMD_INLINE   overlappingpairid rpOverlappingPair::computeID(rpProxyShape* shape1,
		                                                     rpProxyShape* shape2)
{

	assert(shape1->mBroadPhaseID >= 0 && shape2->mBroadPhaseID >= 0);

	// Construct the pair of body index
	overlappingpairid pairID =  shape1->mBroadPhaseID < shape2->mBroadPhaseID ?
								std::make_pair(shape1->mBroadPhaseID, shape2->mBroadPhaseID) :
								std::make_pair(shape2->mBroadPhaseID, shape1->mBroadPhaseID);
	assert(pairID.first != pairID.second);
	return pairID;
}




} /* namespace real_physics */

#endif /* SOURCE_ENGIE_COLLISION_RPOVERLAPPINGPAIR_H_ */
