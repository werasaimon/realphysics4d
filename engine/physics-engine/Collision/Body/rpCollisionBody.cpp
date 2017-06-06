/*
 * CollisionBody.cpp
 *
 *  Created on: 25 нояб. 2016 г.
 *      Author: wera
 */

#include "rpCollisionBody.h"
#include "../../LinearMaths/rpVector3D.h"
#include "../rpProxyShape.h"
#include "../rpRaycastInfo.h"
#include "../Shapes/rpAABB.h"
#include "../Shapes/rpCollisionShape.h"
#include "../../Collision/rpCollisionDetection.h"
#include "../../Geometry/QuickHull/Structs/Ray.hpp"
#include "rpBody.h"

namespace real_physics
{


// Constructor
/**
 * @param transform The transform of the body
 * @param world The physics world where the body is created
 * @param id ID of the body
 */
rpCollisionBody::rpCollisionBody(const Transform& transform, rpCollisionDetection* collideWorld , bodyindex id)
              : rpBody(id), mType(DYNAMIC), mTransform(transform), mProxyCollisionShapes(NULL),
                mNbCollisionShapes(0) , mCollisionDetection(collideWorld) , mContactManifoldsList(NULL) //, mWorld(world)
{

}

// Destructor
rpCollisionBody::~rpCollisionBody()
{
    assert(mContactManifoldsList == NULL);
    // Remove all the proxy collision shapes of the body
    removeAllCollisionShapes();
}





// Add a collision shape to the body. Note that you can share a collision
// shape between several bodies using the same collision shape instance to
// when you add the shape to the different bodies. Do not forget to delete
// the collision shape you have created at the end of your program.
/// This method will return a pointer to a new proxy shape. A proxy shape is
/// an object that links a collision shape and a given body. You can use the
/// returned proxy shape to get and set information about the corresponding
/// collision shape for that body.
/**
 * @param collisionShape A pointer to the collision shape you want to add to the body
 * @param transform The transformation of the collision shape that transforms the
 *        local-space of the collision shape into the local-space of the body
 * @return A pointer to the proxy shape that has been created to link the body to
 *         the new collision shape you have added.
 */
rpProxyShape* rpCollisionBody::addCollisionShape(rpCollisionShape* collisionShape, scalar massa , const Transform& transform)
{

    // Create a new proxy collision shape to attach the collision shape to the body
    rpProxyShape* proxyShape = new rpProxyShape( this, collisionShape, transform , massa);

    // Add it to the list of proxy collision shapes of the body
    if (mProxyCollisionShapes == NULL)
    {
        mProxyCollisionShapes = proxyShape;
    }
    else
    {
        proxyShape->mNext = mProxyCollisionShapes;
        mProxyCollisionShapes = proxyShape;
    }
    mNbCollisionShapes++;


    /**********************************/
    // Compute the world-space AABB of the new collision shape
    rpAABB aabb;
    collisionShape->computeAABB(aabb, mTransform , transform);

    // Notify the collision detection about this new collision shape
    mCollisionDetection->addProxyCollisionShape(proxyShape, aabb);
    /*********************************/

    // Return a pointer to the collision shape
    return proxyShape;
}



// Remove a collision shape from the body
/// To remove a collision shape, you need to specify the pointer to the proxy
/// shape that has been returned when you have added the collision shape to the
/// body
/**
 * @param proxyShape The pointer of the proxy shape you want to remove
 */
void rpCollisionBody::removeCollisionShapee(const rpProxyShape* proxyShape)
{

	rpProxyShape* current = mProxyCollisionShapes;

	// If the the first proxy shape is the one to remove
	if (current == proxyShape)
	{
		mProxyCollisionShapes = current->mNext;

        if (mIsActive)
		{
		   mCollisionDetection->removeProxyCollisionShape(current);
		}

		current->~rpProxyShape();
		delete current;

		mNbCollisionShapes--;
		return;
	}

	// Look for the proxy shape that contains the collision shape in parameter
	while(current->mNext != NULL)
	{
		// If we have found the collision shape to remove
		if (current->mNext == proxyShape)
		{

			// Remove the proxy collision shape
			rpProxyShape* elementToRemove = current->mNext;
			current->mNext = elementToRemove->mNext;

			if (mIsActive)
			{
				mCollisionDetection->removeProxyCollisionShape(elementToRemove);
			}

			elementToRemove->~rpProxyShape();
			delete elementToRemove;

			mNbCollisionShapes--;
			return;
		}

		// Get the next element in the list
		current = current->mNext;
	}
}



// Remove all the collision shapes
void rpCollisionBody::removeAllCollisionShapes()
{

	    rpProxyShape* current = mProxyCollisionShapes;

	    // Look for the proxy shape that contains the collision shape in parameter
	    while(current != NULL)
	    {

	        // Remove the proxy collision shape
	        rpProxyShape* nextElement = current->mNext;

            if (mIsActive)
	        {
	            mCollisionDetection->removeProxyCollisionShape(current);
	        }

	        current->~rpProxyShape();

	        delete current->mCollisionShape;
	        delete current;
	        current = NULL;
	        // Get the next element in the list
	        current = nextElement;

	    }

	    mProxyCollisionShapes = NULL;
}


// Reset the contact manifold lists
void rpCollisionBody::resetContactManifoldsList()
{
    // Delete the linked list of contact manifolds of that body
    rpContactManifoldListElement* currentElement = mContactManifoldsList;
    while (currentElement != NULL)
    {
        rpContactManifoldListElement* nextElement = currentElement->next;

        // Delete the current element
        delete currentElement;

        currentElement = nextElement;
    }
    mContactManifoldsList = NULL;
}



// Update the broad-phase state for this body (because it has moved for instance)
void rpCollisionBody::updateBroadPhaseState() const
{
    // For all the proxy collision shapes of the body
    for (rpProxyShape* shape = mProxyCollisionShapes; shape != NULL; shape = shape->mNext)
    {
        // Update the proxy
        updateProxyShapeInBroadPhase(shape , Vector3::ZERO);
    }
}



// Update the broad-phase state of a proxy collision shape of the body
void rpCollisionBody::updateProxyShapeInBroadPhase(rpProxyShape* proxyShape, const Vector3& displacement, bool forceReinsert) const
{

	// Recompute the world-space AABB of the collision shape
	rpAABB aabb;
	proxyShape->getCollisionShape()->computeAABB(aabb, mTransform , proxyShape->getLocalToBodyTransform() , mRelativityMotion.getLorentzMatrix());


	// Update the broad-phase state for the proxy collision shape
    mCollisionDetection->updateProxyCollisionShape(proxyShape, aabb , displacement , forceReinsert);

}



// Ask the broad-phase to test again the collision shapes of the body for collision
// (as if the body has moved).
void rpCollisionBody::askForBroadPhaseCollisionCheck() const
{
    // For all the proxy collision shapes of the body
    for (rpProxyShape* shape = mProxyCollisionShapes; shape != NULL; shape = shape->mNext)
    {
       mCollisionDetection->askForBroadPhaseCollisionCheck(shape);
    }
}



// Set whether or not the body is active
/**
 * @param isActive True if you want to activate the body
 */
void rpCollisionBody::setIsActive(bool isActive)
{

    // If the state does not change
    if (mIsActive == isActive) return;

    rpBody::setIsActive(isActive);

    // If we have to activate the body
    if (isActive)
    {

        // For each proxy shape of the body
        for (rpProxyShape* shape = mProxyCollisionShapes; shape != NULL; shape = shape->mNext)
        {

            // Compute the world-space AABB of the new collision shape
            rpAABB aabb;
            shape->getCollisionShape()->computeAABB(aabb, mTransform , shape->mLocalToBodyTransform , mRelativityMotion.getLorentzMatrix());

            // Add the proxy shape to the collision detection
             mCollisionDetection->addProxyCollisionShape(shape, aabb);
        }
    }
    else
    {  // If we have to deactivate the body

        // For each proxy shape of the body
        for (rpProxyShape* shape = mProxyCollisionShapes; shape != NULL; shape = shape->mNext)
        {
            // Remove the proxy shape from the collision detection
            mCollisionDetection->removeProxyCollisionShape(shape);
        }

        // Reset the contact manifold list of the body
        resetContactManifoldsList();
    }
}



//// Reset the mIsAlreadyInIsland variable of the body and contact manifolds.
///// This method also returns the number of contact manifolds of the body.
int rpCollisionBody::resetIsAlreadyInIslandAndCountManifolds()
{

    mIsAlreadyInIsland = false;

    // Reset the mIsAlreadyInIsland variable of the contact manifolds for
    // this body
    int nbManifolds = 0;
    rpContactManifoldListElement* currentElement = mContactManifoldsList;
    while (currentElement != NULL)
    {
        currentElement->contactManifold->mIsAlreadyInIsland = false;
        currentElement = currentElement->next;
        nbManifolds++;
    }
    return nbManifolds;
}





// Return true if a point is inside the collision body
/// This method returns true if a point is inside any collision shape of the body
/**
 * @param worldPoint The point to test (in world-space coordinates)
 * @return True if the point is inside the body
 */
bool rpCollisionBody::testPointInside(const Vector3& worldPoint) const
{
    // For each collision shape of the body
    for (rpProxyShape* shape = mProxyCollisionShapes; shape != NULL; shape = shape->mNext)
    {
        // Test if the point is inside the collision shape
        if (shape->testPointInside(worldPoint)) return true;
    }

    return false;
}



// Raycast method with feedback information
/// The method returns the closest hit among all the collision shapes of the body
/**
* @param ray The ray used to raycast agains the body
* @param[out] raycastInfo Structure that contains the result of the raycasting
*                         (valid only if the method returned true)
* @return True if the ray hit the body and false otherwise
*/
bool rpCollisionBody::raycast(const Ray& ray, RaycastInfo& raycastInfo)
{
    // If the body is not active, it cannot be hit by rays
    if (!mIsActive) return false;

    bool isHit = false;
    Ray rayTemp(ray);


    // For each collision shape of the body
    for (rpProxyShape* shape = mProxyCollisionShapes; shape != NULL; shape = shape->mNext)
    {
        // Test if the ray hits the collision shape
        if (shape->raycast(rayTemp, raycastInfo))
        {
            rayTemp.maxFraction = raycastInfo.hitFraction;
            isHit = true;
        }
    }

    return isHit;
}




// Compute and return the AABB of the body by merging all proxy shapes AABBs
/**
* @return The axis-aligned bounding box (AABB) of the body in world-space coordinates
*/
rpAABB rpCollisionBody::getAABB() const
{

    rpAABB bodyAABB;

    if (mProxyCollisionShapes == NULL) return bodyAABB;

    mProxyCollisionShapes->getCollisionShape()->computeAABB(bodyAABB, mTransform , mProxyCollisionShapes->getLocalToBodyTransform() , mRelativityMotion.getLorentzMatrix());

    // For each proxy shape of the body
    for (rpProxyShape* shape = mProxyCollisionShapes->mNext; shape != NULL; shape = shape->mNext)
    {
        // Compute the world-space AABB of the collision shape
        rpAABB aabb;
        shape->getCollisionShape()->computeAABB(aabb, mTransform , shape->getLocalToBodyTransform() , mRelativityMotion.getLorentzMatrix());

        // Merge the proxy shape AABB with the current body AABB
        bodyAABB.mergeWithAABB(aabb);
    }

    return bodyAABB;
}



} /* namespace real_physics */
