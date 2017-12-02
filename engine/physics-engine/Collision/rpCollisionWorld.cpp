/*
 * rpCollisionWorld.cpp
 *
 *  Created on: 27 нояб. 2016 г.
 *      Author: wera
 */

#include "rpCollisionWorld.h"
#include <algorithm>

namespace real_physics
{


// Constructor
rpCollisionWorld::rpCollisionWorld()
 : mCollisionDetection(),
   mCurrentBodyID(0)
{

}

// Destructor
rpCollisionWorld::~rpCollisionWorld()
{

    // Destroy all the collision bodies that have not been removed
    std::set<rpCollisionBody*>::iterator itBodies;
    for (itBodies = mBodies.begin(); itBodies != mBodies.end(); )
    {
        std::set<rpCollisionBody*>::iterator itToRemove = itBodies;
        ++itBodies;
        destroyCollisionBody(*itToRemove);
    }


    assert(mBodies.empty());
}

// Create a collision body and add it to the world
/**
 * @param transform Transformation mapping the local-space of the body to world-space
 * @return A pointer to the body that has been created in the world
 */
rpCollisionBody* rpCollisionWorld::createCollisionBody(const Transform &transform)
{
    // Get the next available body ID
    bodyindex bodyID = computeNextAvailableBodyID();

    // Largest index cannot be used (it is used for invalid index)
    assert(bodyID < std::numeric_limits<bodyindex>::max());

    // Create the collision body
    rpCollisionBody* collisionBody = new rpCollisionBody(transform, &mCollisionDetection, bodyID);

    assert(collisionBody != NULL);

    // Add the collision body to the world
    mBodies.insert(collisionBody);

    // Return the pointer to the rigid body
    return collisionBody;
}

// Destroy a collision body
/**
 * @param collisionBody Pointer to the body to destroy
 */
void rpCollisionWorld::destroyCollisionBody(rpCollisionBody* collisionBody)
{

    // Remove all the collision shapes of the body
    collisionBody->removeAllCollisionShapes();

    // Add the body ID to the list of free IDs
    mFreeBodiesIDs.push_back(collisionBody->getID());

    // Call the destructor of the collision body
   // collisionBody->~rpCollisionBody();
    delete collisionBody;

    // Remove the collision body from the list of bodies
    mBodies.erase(collisionBody);

    // Free the object from the memory allocator
    //mMemoryAllocator.release(collisionBody, sizeof(CollisionBody));
}

// Return the next available body ID
bodyindex rpCollisionWorld::computeNextAvailableBodyID()
{

    // Compute the body ID
    bodyindex bodyID;
    if (!mFreeBodiesIDs.empty())
    {
        bodyID = mFreeBodiesIDs.back();
        mFreeBodiesIDs.pop_back();
    }
    else
    {
        bodyID = mCurrentBodyID;
        mCurrentBodyID++;
    }

    return bodyID;
}


// Reset all the contact manifolds linked list of each body
void rpCollisionWorld::resetContactManifoldListsOfBodies()
{
    // For each rigid body of the world
    for (auto it = mBodies.begin(); it != mBodies.end(); ++it)
    {
        // Reset the contact manifold list of the body
        (*it)->resetContactManifoldsList();
    }
}


// Update collision tohet all bodies
void rpCollisionWorld::UpdateCollision()
{

    resetContactManifoldListsOfBodies();

	/**/
	for( auto itBodies = mBodies.begin(); itBodies != mBodies.end(); ++itBodies )
	{
		(*itBodies)->updateBroadPhaseState();
	}
	/**/


    mCollisionDetection.computeCollisionDetectionAllPairs();




	/********************************************************************/
    for( auto pair : mCollisionDetection.mContactOverlappingPairs )
	{
        const int NbSize =  pair.second->getContactManifoldSet().getNbContactManifolds();

        for (int i = 0; i < NbSize; ++i)
        {
        	const rpContactManifold* maniflod = pair.second->getContactManifoldSet().getContactManifold(i);
		}
	}
	/********************************************************************/

}

// Set the collision dispatch configuration
/// This can be used to replace default collision detection algorithms by your
/// custom algorithm for instance.
/**
 * @param CollisionDispatch Pointer to a collision dispatch object describing
 * which collision detection algorithm to use for two given collision shapes
 */
//inline void rpCollisionWorld::setCollisionDispatch(CollisionDispatch* collisionDispatch)
//{
//    mCollisionDetection.setCollisionDispatch(collisionDispatch);
//}

// Ray cast method
/**
 * @param ray Ray to use for raycasting
 * @param raycastCallback Pointer to the class with the callback method
 * @param raycastWithCategoryMaskBits Bits mask corresponding to the category of
 *                                    bodies to be raycasted
 */

SIMD_INLINE void rpCollisionWorld::raycast(const Ray& ray, RaycastCallback* raycastCallback,
                                           unsigned short raycastWithCategoryMaskBits) const
{
    mCollisionDetection.raycast(raycastCallback, ray, raycastWithCategoryMaskBits);
}



} /* namespace real_physics */
