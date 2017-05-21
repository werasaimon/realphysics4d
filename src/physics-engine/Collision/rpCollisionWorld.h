/*
 * rpCollisionWorld.h
 *
 *  Created on: 27 нояб. 2016 г.
 *      Author: wera
 */

#ifndef SOURCE_ENGIE_COLLISION_RPCOLLISIONWORLD_H_
#define SOURCE_ENGIE_COLLISION_RPCOLLISIONWORLD_H_


// Libraries
#include <vector>
#include <set>
#include <list>
#include <algorithm>
#include "../LinearMaths/mathematics.h"
#include "Body/rpCollisionBody.h"
#include "rpRaycastInfo.h"
#include "rpOverlappingPair.h"
#include "rpCollisionDetection.h"


namespace real_physics
{



// Class CollisionWorld
/**
 * This class represent a world where it is possible to move bodies
 * by hand and to test collision between each other. In this kind of
 * world, the bodies movement is not computed using the laws of physics.
 */
class rpCollisionWorld
{

    protected:

		// -------------------- Attributes -------------------- //

		/// Reference to the collision detection
		rpCollisionDetection       mCollisionDetection;

		/// All the bodies (rigid and soft) of the world
		std::set<rpCollisionBody*> mBodies;

		/// Current body ID
		bodyindex                  mCurrentBodyID;

		/// List of free ID for rigid bodies
		std::vector<luint>         mFreeBodiesIDs;

		/// Overlapping pairs in contact (during the current Narrow-phase collision detection)
	    std::map<overlappingpairid, rpOverlappingPair*> mCollisionContactOverlappingPairs;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        rpCollisionWorld(const rpCollisionWorld& world);

        /// Private assignment operator
        rpCollisionWorld& operator=(const rpCollisionWorld& world);

        /// Return the next available body ID
        bodyindex computeNextAvailableBodyID();

        /// Reset all the contact manifolds linked list of each body
        void resetContactManifoldListsOfBodies();



    public:
        // -------------------- Methods -------------------- //

        /// Constructor
        rpCollisionWorld();

        /// Destructor
        virtual ~rpCollisionWorld();

        /// Return an iterator to the beginning of the bodies of the physics world
        std::set<rpCollisionBody*>::iterator getBodiesBeginIterator();

        /// Return an iterator to the end of the bodies of the physics world
        std::set<rpCollisionBody*>::iterator getBodiesEndIterator();

        /// Create a collision body
        rpCollisionBody* createCollisionBody(const Transform& transform);

        /// Destroy a collision body
        void destroyCollisionBody(rpCollisionBody* collisionBody);


        void UpdateCollision();

        /// Set the collision dispatch configuration
       // void setCollisionDispatch(CollisionDispatch* collisionDispatch);

        /// Ray cast method
        void raycast(const Ray& ray, RaycastCallback* raycastCallback,
                     unsigned short raycastWithCategoryMaskBits = 0xFFFF) const;
//
//        /// Test if the AABBs of two bodies overlap
//
//
//        /// Test and report collisions between all shapes of the world
//        virtual void testCollision(CollisionCallback* callback);



        void addNoCollisionPair(rpCollisionBody* body1, rpCollisionBody* body2)
        {
            mCollisionDetection.addNoCollisionPair( body1 , body2 );
        }

        // -------------------- Friendship -------------------- //

        friend class rpDynamicsWorld;
        friend class rpCollisionDetection;
        friend class rpCollisionBody;
        friend class rpRigidPhysicsBody;
        friend class rpConvexMeshShape;
};




// Return an iterator to the beginning of the bodies of the physics world
/**
 * @return An starting iterator to the set of bodies of the world
 */
SIMD_INLINE std::set<rpCollisionBody*>::iterator rpCollisionWorld::getBodiesBeginIterator()
{
    return mBodies.begin();
}

// Return an iterator to the end of the bodies of the physics world
/**
 * @return An ending iterator to the set of bodies of the world
 */
SIMD_INLINE std::set<rpCollisionBody*>::iterator rpCollisionWorld::getBodiesEndIterator()
{
    return mBodies.end();
}


} /* namespace real_physics */

#endif /* SOURCE_ENGIE_COLLISION_RPCOLLISIONWORLD_H_ */
