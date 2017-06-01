/*
 * rpContactManifold.h
 *
 *  Created on: 28 нояб. 2016 г.
 *      Author: wera
 */

#ifndef SOURCE_ENGIE_COLLISION_CONTACTMANIFLOD_RPCONTACTMANIFOLD_H_
#define SOURCE_ENGIE_COLLISION_CONTACTMANIFLOD_RPCONTACTMANIFOLD_H_


// Libraries
#include <vector>
#include <iostream>

#include "../rpProxyShape.h"
#include "rpContactPoint.h"
using namespace std;

namespace real_physics
{

// Constants
const uint MAX_CONTACT_POINTS_IN_MANIFOLD = 32;   // Maximum number of contacts in the manifold

// Class declarations
class rpContactManifold;

// Structure ContactManifoldListElement
/**
 * This structure represents a single element of a linked list of contact manifolds
 */
struct rpContactManifoldListElement
{

    public:

        // -------------------- Attributes -------------------- //

        /// Pointer to the actual contact manifold
        rpContactManifold* contactManifold;

        /// Next element of the list
        rpContactManifoldListElement* next;

        // -------------------- Methods -------------------- //

        /// Constructor
        rpContactManifoldListElement(rpContactManifold* initContactManifold, rpContactManifoldListElement* initNext)
        :contactManifold(initContactManifold), next(initNext)
        {

        }
};





class rpContactManifold
{

    private:


//	    uint mIdIndex1;
//	    uint mIdIndex2;


        // -------------------- Attributes -------------------- //

		/// Pointer to the first proxy shape of the contact
	    rpProxyShape* mShape1;

		/// Pointer to the second proxy shape of the contact
		rpProxyShape* mShape2;


		/// Number of contacts in the cache
		uint mNbContactPoints;


        /// Contact points in the manifold
        rpContactPoint* mContactPoints[MAX_CONTACT_POINTS_IN_MANIFOLD];


        /// Normal direction Id (Unique Id representing the normal direction)
        short int mNormalDirectionId;



        /// First friction vector of the contact manifold
        Vector3 mFrictionVector1;

        /// Second friction vector of the contact manifold
        Vector3 mFrictionVector2;

        /// First friction constraint accumulated impulse
        scalar mFrictionImpulse1;

        /// Second friction constraint accumulated impulse
        scalar mFrictionImpulse2;

        /// Twist friction constraint accumulated impulse
        scalar mFrictionTwistImpulse;

        /// Accumulated rolling resistance impulse
        Vector3 mRollingResistanceImpulse;

        /// True if the contact manifold has already been added into an island
        bool mIsAlreadyInIsland;


        /// Reference to the memory allocator
        ///MemoryAllocator& mMemoryAllocator;
        scalar mExtremalPenetration;


        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        rpContactManifold(const rpContactManifold& contactManifold);

        /// Private assignment operator
        rpContactManifold& operator=(const rpContactManifold& contactManifold);

        /// Return the index of maximum area
        uint getMaxArea(scalar area0, scalar area1, scalar area2, scalar area3) const;

        /// Return the index of the contact with the larger penetration depth.
        uint getIndexOfDeepestPenetration(rpContactPoint* newContact) const;

        /// Return the index that will be removed.
        uint getIndexToRemove(uint indexMaxPenetration, const Vector3& newPoint) const;

        /// Remove a contact point from the manifold
        void removeContactPoint(uint index);

        /// Return true if the contact manifold has already been added into an island
        bool isAlreadyInIsland() const;

    public:

        // -------------------- Methods -------------------- //

        rpContactManifold(void){}

        /// Constructor
        rpContactManifold( rpProxyShape* shape1, rpProxyShape* shape2 ,
        		         short int normalDirectionId = 0);

        /// Destructor
        ~rpContactManifold();


                //        /// Return a pointer to the first body of the contact manifold
                //        CollisionBody* getBody1() const;

                //        /// Return a pointer to the second body of the contact manifold
                //        CollisionBody* getBody2() const;

        /// Return the normal direction Id
        short int getNormalDirectionId() const;

        /// Add a contact point to the manifold
        void addContactPoint(rpContactPoint* contact);



//        void deleteAllPoint()
//        {
//        	for( int i = mNbContactPoints-1; i >=0 ; i-- )
//        	{
//        		removeContactPoint(i);
//        	}
//
//        }

//        void removeContactPoint2(int index)
//        {
//        	assert(index < mNbContactPoints);
//        	assert(mNbContactPoints > 0);
//
//        	delete mContactPoints[index];
//        	       mContactPoints[index] = NULL;
//
//        	// If we don't remove the last index
//        	if (index < mNbContactPoints - 1)
//        	{
//        		mContactPoints[index] = mContactPoints[mNbContactPoints - 1];
//        	}
//
//        	mNbContactPoints--;
//
//        }



        /// Update the contact manifold.
        void update(const Transform& transform1, const Transform& transform2);

        /// Clear the contact manifold
        void clear();

        /// Return the number of contact points in the manifold
        uint getNbContactPoints() const;

        /// Return the first friction vector at the center of the contact manifold
        const Vector3& getFrictionVector1() const;

        /// set the first friction vector at the center of the contact manifold
        void setFrictionVector1(const Vector3& mFrictionVector1);

        /// Return the second friction vector at the center of the contact manifold
        const Vector3& getFrictionVector2() const;

        /// set the second friction vector at the center of the contact manifold
        void setFrictionVector2(const Vector3& mFrictionVector2);

        /// Return the first friction accumulated impulse
        scalar getFrictionImpulse1() const;

        /// Set the first friction accumulated impulse
        void setFrictionImpulse1(scalar frictionImpulse1);

        /// Return the second friction accumulated impulse
        scalar getFrictionImpulse2() const;

        /// Set the second friction accumulated impulse
        void setFrictionImpulse2(scalar frictionImpulse2);

        /// Return the friction twist accumulated impulse
        scalar getFrictionTwistImpulse() const;

        /// Set the friction twist accumulated impulse
        void setFrictionTwistImpulse(scalar frictionTwistImpulse);

        /// Set the accumulated rolling resistance impulse
        void setRollingResistanceImpulse(const Vector3& rollingResistanceImpulse);

        /// Return a contact point of the manifold
        rpContactPoint* getContactPoint(uint index) const;

        /// Return the normalized averaged normal vector
        Vector3 getAverageContactNormal() const;

        /// Return the largest depth of all the contact points
        scalar getLargestContactDepth() const;



		const rpProxyShape* getShape1() const
		{
			return mShape1;
		}

		const rpProxyShape* getShape2() const
		{
			return mShape2;
		}

        // -------------------- Friendship -------------------- //

        friend class rpSequentialImpulseObjectSolver;
        friend class rpContactSolverSequentialImpulse;
        friend class rpContactManifoldSet;
        friend class rpDynamicsWorld;
        friend class rpIsland;
        friend class rpCollisionBody;
};




//// Return a pointer to the first proxy shape of the contact
//SIMD_INLINE rpProxyShape *ContactManifold::vgetShape1() const {
//    return mShape1;
//}
//
//// Return a pointer to the second proxy shape of the contact
//SIMD_INLINE rpProxyShape *ContactManifold::getShape2() const {
//    return mShape2;
//}

            //// Return a pointer to the first body of the contact manifold
            //SIMD_INLINE CollisionBody* ContactManifold::getBody1() const {
            //    return mShape1->getBody();
            //}

            //// Return a pointer to the second body of the contact manifold
            //SIMD_INLINE CollisionBody* ContactManifold::getBody2() const {
            //    return mShape2->getBody();
            //}




// Return the normal direction Id
SIMD_INLINE short int rpContactManifold::getNormalDirectionId() const
{
    return mNormalDirectionId;
}

// Return the number of contact points in the manifold
SIMD_INLINE uint rpContactManifold::getNbContactPoints() const
{
    return mNbContactPoints;
}

// Return the first friction vector at the center of the contact manifold
SIMD_INLINE const Vector3& rpContactManifold::getFrictionVector1() const
{
    return mFrictionVector1;
}

// set the first friction vector at the center of the contact manifold
SIMD_INLINE void rpContactManifold::setFrictionVector1(const Vector3& frictionVector1)
{
    mFrictionVector1 = frictionVector1;
}

// Return the second friction vector at the center of the contact manifold
SIMD_INLINE const Vector3& rpContactManifold::getFrictionVector2() const
{
    return mFrictionVector2;
}

// set the second friction vector at the center of the contact manifold
SIMD_INLINE void rpContactManifold::setFrictionVector2(const Vector3& frictionVector2)
{
    mFrictionVector2 = frictionVector2;
}

// Return the first friction accumulated impulse
SIMD_INLINE scalar rpContactManifold::getFrictionImpulse1() const
{
    return mFrictionImpulse1;
}

// Set the first friction accumulated impulse
SIMD_INLINE void rpContactManifold::setFrictionImpulse1(scalar frictionImpulse1)
{
    mFrictionImpulse1 = frictionImpulse1;
}

// Return the second friction accumulated impulse
SIMD_INLINE scalar rpContactManifold::getFrictionImpulse2() const
{
    return mFrictionImpulse2;
}

// Set the second friction accumulated impulse
SIMD_INLINE void rpContactManifold::setFrictionImpulse2(scalar frictionImpulse2)
{
    mFrictionImpulse2 = frictionImpulse2;
}

// Return the friction twist accumulated impulse
SIMD_INLINE scalar rpContactManifold::getFrictionTwistImpulse() const
{
	return mFrictionTwistImpulse;
}

// Set the friction twist accumulated impulse
SIMD_INLINE void rpContactManifold::setFrictionTwistImpulse(scalar frictionTwistImpulse)
{
    mFrictionTwistImpulse = frictionTwistImpulse;
}

// Set the accumulated rolling resistance impulse
SIMD_INLINE void rpContactManifold::setRollingResistanceImpulse(const Vector3& rollingResistanceImpulse)
{
    mRollingResistanceImpulse = rollingResistanceImpulse;
}

// Return a contact point of the manifold
SIMD_INLINE rpContactPoint* rpContactManifold::getContactPoint(uint index) const
{
	//cout<< index << "  " << mNbContactPoints <<endl;
    assert(index < mNbContactPoints);
    return mContactPoints[index];
}

// Return true if the contact manifold has already been added into an island
SIMD_INLINE bool rpContactManifold::isAlreadyInIsland() const
{
    return mIsAlreadyInIsland;
}

// Return the normalized averaged normal vector
SIMD_INLINE Vector3 rpContactManifold::getAverageContactNormal() const
{
    Vector3 averageNormal;

    for (uint i=0; i<mNbContactPoints; i++)
    {
        averageNormal += mContactPoints[i]->getNormal();
    }

    return averageNormal.getUnit();
}

// Return the largest depth of all the contact points
SIMD_INLINE scalar rpContactManifold::getLargestContactDepth() const
{
    scalar largestDepth = 0.0f;

    for (uint i=0; i<mNbContactPoints; i++)
    {
        scalar depth = mContactPoints[i]->getPenetrationDepth();
        if (depth > largestDepth)
        {
            largestDepth = depth;
        }
    }

    return largestDepth;
}

} /* namespace real_physics */

#endif /* SOURCE_ENGIE_COLLISION_CONTACTMANIFLOD_RPCONTACTMANIFOLD_H_ */
