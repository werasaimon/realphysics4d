/*
 * rbBroadPhaseAlgorithm.cpp
 *
 *  Created on: 23 нояб. 2016 г.
 *      Author: wera
 */

#include "rbBroadPhaseAlgorithm.h"

#include <stdlib.h>
#include <algorithm>
#include <cassert>
#include <cstring>

#include "../../Memory/memory.h"
#include "../rpCollisionDetection.h"
#include "../rpRaycastInfo.h"

namespace real_physics
{



// Constructor
rpBroadPhaseAlgorithm::rpBroadPhaseAlgorithm(rpCollisionDetection* collisionDetection)
                    :mDynamicAABBTree(DYNAMIC_TREE_AABB_GAP), mNbMovedShapes(0), mNbAllocatedMovedShapes(8),
                     mNbNonUsedMovedShapes(0), mNbPotentialPairs(0), mNbAllocatedPotentialPairs(8),
                     mCollisionDetection(collisionDetection)
{

    // Allocate memory for the array of non-static proxy shapes IDs
    mMovedShapes = (int*) malloc(mNbAllocatedMovedShapes * sizeof(int));
    assert(mMovedShapes != NULL);

    // Allocate memory for the array of potential overlapping pairs
    mPotentialPairs = (rpBroadPhasePair*) malloc(mNbAllocatedPotentialPairs * sizeof(rpBroadPhasePair));
    assert(mPotentialPairs != NULL);
}

// Destructor
rpBroadPhaseAlgorithm::~rpBroadPhaseAlgorithm()
{

    // Release the memory for the array of non-static proxy shapes IDs
    free(mMovedShapes);

    // Release the memory for the array of potential overlapping pairs
    free(mPotentialPairs);
}




// Add a collision shape in the array of shapes that have moved in the last simulation step
// and that need to be tested again for broad-phase overlapping.
void rpBroadPhaseAlgorithm::addMovedCollisionShape(int broadPhaseID)
{

    // Allocate more elements in the array of shapes that have moved if necessary
    if (mNbAllocatedMovedShapes == mNbMovedShapes)
    {
        mNbAllocatedMovedShapes *= 2;
        int* oldArray = mMovedShapes;
        mMovedShapes = (int*) malloc(mNbAllocatedMovedShapes * sizeof(int));
        assert(mMovedShapes != NULL);
        memcpy(mMovedShapes, oldArray, mNbMovedShapes * sizeof(int));
        free(oldArray);
    }

    // Store the broad-phase ID into the array of shapes that have moved
    assert(mNbMovedShapes < mNbAllocatedMovedShapes);
    assert(mMovedShapes != NULL);
    mMovedShapes[mNbMovedShapes] = broadPhaseID;
    mNbMovedShapes++;
}

// Remove a collision shape from the array of shapes that have moved in the last simulation step
// and that need to be tested again for broad-phase overlapping.
void rpBroadPhaseAlgorithm::removeMovedCollisionShape(int broadPhaseID)
{

    assert(mNbNonUsedMovedShapes <= mNbMovedShapes);

    // If less than the quarter of allocated elements of the non-static shapes IDs array
    // are used, we release some allocated memory
    if ((mNbMovedShapes - mNbNonUsedMovedShapes) < mNbAllocatedMovedShapes / 4 &&
         mNbAllocatedMovedShapes > 8)
    {

        mNbAllocatedMovedShapes /= 2;
        int* oldArray = mMovedShapes;
        mMovedShapes = (int*) malloc(mNbAllocatedMovedShapes * sizeof(int));
        assert(mMovedShapes != NULL);
        uint nbElements = 0;
        for (uint i=0; i<mNbMovedShapes; i++)
        {
            if (oldArray[i] != -1)
            {
                mMovedShapes[nbElements] = oldArray[i];
                nbElements++;
            }
        }
        mNbMovedShapes = nbElements;
        mNbNonUsedMovedShapes = 0;
        free(oldArray);
    }

    // Remove the broad-phase ID from the array
    for (uint i=0; i<mNbMovedShapes; i++)
    {
        if (mMovedShapes[i] == broadPhaseID)
        {
            mMovedShapes[i] = -1;
            mNbNonUsedMovedShapes++;
            break;
        }
    }
}

// Add a proxy collision shape into the broad-phase collision detection
void rpBroadPhaseAlgorithm::addProxyCollisionShape(rpProxyShape* proxyShape, const rpAABB& aabb)
{

    // Add the collision shape into the dynamic AABB tree and get its broad-phase ID
    int nodeId = mDynamicAABBTree.addObject(aabb, proxyShape);

    // Set the broad-phase ID of the proxy shape
    proxyShape->mBroadPhaseID = nodeId;

    // Add the collision shape into the array of bodies that have moved (or have been created)
    // during the last simulation step
    addMovedCollisionShape(proxyShape->mBroadPhaseID);
}

// Remove a proxy collision shape from the broad-phase collision detection
void rpBroadPhaseAlgorithm::removeProxyCollisionShape(rpProxyShape* proxyShape)
{

    int broadPhaseID = proxyShape->mBroadPhaseID;

    // Remove the collision shape from the dynamic AABB tree
    mDynamicAABBTree.removeObject(broadPhaseID);

    // Remove the collision shape into the array of shapes that have moved (or have been created)
    // during the last simulation step
    removeMovedCollisionShape(broadPhaseID);
}

// Notify the broad-phase that a collision shape has moved and need to be updated
void rpBroadPhaseAlgorithm::updateProxyCollisionShape(rpProxyShape* proxyShape, const rpAABB& aabb,
                                                    const Vector3& displacement, bool forceReinsert)
{

    int broadPhaseID = proxyShape->mBroadPhaseID;

    assert(broadPhaseID >= 0);

    // Update the dynamic AABB tree according to the movement of the collision shape
    bool hasBeenReInserted = mDynamicAABBTree.updateObject(broadPhaseID, aabb, displacement, forceReinsert);

    // If the collision shape has moved out of its fat AABB (and therefore has been reinserted
    // into the tree).
    if (hasBeenReInserted)
    {
        // Add the collision shape into the array of shapes that have moved (or have been created)
        // during the last simulation step
        addMovedCollisionShape(broadPhaseID);
    }
}

// Compute all the overlapping pairs of collision shapes
void rpBroadPhaseAlgorithm::computeOverlappingPairs()
{

    // Reset the potential overlapping pairs
    mNbPotentialPairs = 0;

    // For all collision shapes that have moved (or have been created) during the
    // last simulation step
    for (uint i=0; i<mNbMovedShapes; i++)
    {
        int shapeID = mMovedShapes[i];

        if (shapeID == -1) continue;

        rpAABBOverlapCallback callback(*this, shapeID);

        // Get the AABB of the shape
        const rpAABB& shapeAABB = mDynamicAABBTree.getFatAABB(shapeID);

        // Ask the dynamic AABB tree to report all collision shapes that overlap with
        // this AABB. The method BroadPhase::notifiyOverlappingPair() will be called
        // by the dynamic AABB tree for each potential overlapping pair.
        mDynamicAABBTree.reportAllShapesOverlappingWithAABB(shapeAABB, callback);
    }

    // Reset the array of collision shapes that have move (or have been created) during the
    // last simulation step
    mNbMovedShapes = 0;

    // Sort the array of potential overlapping pairs in order to remove duplicate pairs
    std::sort(mPotentialPairs, mPotentialPairs + mNbPotentialPairs, rpBroadPhasePair::smallerThan);

    // Check all the potential overlapping pairs avoiding duplicates to report unique
    // overlapping pairs
    uint i=0;
    while (i < mNbPotentialPairs)
    {

        // Get a potential overlapping pair
        rpBroadPhasePair* pair = mPotentialPairs + i;
        i++;

        assert(pair->collisionShape1ID != pair->collisionShape2ID);

        // Get the two collision shapes of the pair
        rpProxyShape* shape1 = static_cast<rpProxyShape*>(mDynamicAABBTree.getNodeDataPointer(pair->collisionShape1ID));
        rpProxyShape* shape2 = static_cast<rpProxyShape*>(mDynamicAABBTree.getNodeDataPointer(pair->collisionShape2ID));

        // Notify the collision detection about the overlapping pair
        mCollisionDetection->broadPhaseNotifyOverlappingPair( shape1 , shape2 );

        // Skip the duplicate overlapping pairs
        while (i < mNbPotentialPairs)
        {

            // Get the next pair
            rpBroadPhasePair* nextPair = mPotentialPairs + i;

            // If the next pair is different from the previous one, we stop skipping pairs
            if (nextPair->collisionShape1ID != pair->collisionShape1ID ||
                nextPair->collisionShape2ID != pair->collisionShape2ID)
            {
                break;
            }
            i++;
        }
    }

    // If the number of potential overlapping pairs is less than the quarter of allocated
    // number of overlapping pairs
    if (mNbPotentialPairs < mNbAllocatedPotentialPairs / 4 && mNbPotentialPairs > 8)
    {
        // Reduce the number of allocated potential overlapping pairs
        rpBroadPhasePair* oldPairs = mPotentialPairs;
        mNbAllocatedPotentialPairs /= 2;
        mPotentialPairs = (rpBroadPhasePair*) malloc(mNbAllocatedPotentialPairs * sizeof(rpBroadPhasePair));
        assert(mPotentialPairs);
        memcpy(mPotentialPairs, oldPairs, mNbPotentialPairs * sizeof(rpBroadPhasePair));
        free(oldPairs);
    }
}

// Notify the broad-phase about a potential overlapping pair in the dynamic AABB tree
void rpBroadPhaseAlgorithm::notifyOverlappingNodes(int node1ID, int node2ID)
{

    // If both the nodes are the same, we do not create store the overlapping pair
    if (node1ID == node2ID) return;

    // If we need to allocate more memory for the array of potential overlapping pairs
    if (mNbPotentialPairs == mNbAllocatedPotentialPairs)
    {
        // Allocate more memory for the array of potential pairs
        rpBroadPhasePair* oldPairs = mPotentialPairs;
        mNbAllocatedPotentialPairs *= 2;
        mPotentialPairs = (rpBroadPhasePair*) malloc(mNbAllocatedPotentialPairs * sizeof(rpBroadPhasePair));
        assert(mPotentialPairs);
        memcpy(mPotentialPairs, oldPairs, mNbPotentialPairs * sizeof(rpBroadPhasePair));
        free(oldPairs);
    }

    // Add the new potential pair into the array of potential overlapping pairs
    mPotentialPairs[mNbPotentialPairs].collisionShape1ID = Min(node1ID, node2ID);
    mPotentialPairs[mNbPotentialPairs].collisionShape2ID = Max(node1ID, node2ID);
    mNbPotentialPairs++;
}

// Called when a overlapping node has been found during the call to
// DynamicAABBTree:reportAllShapesOverlappingWithAABB()
void rpAABBOverlapCallback::notifyOverlappingNode(int nodeId)
{
    mBroadPhaseAlgorithm.notifyOverlappingNodes(mReferenceNodeId, nodeId);
}

// Called for a broad-phase shape that has to be tested for raycast
scalar rpBroadPhaseRaycastCallback::raycastBroadPhaseShape(int32 nodeId, const Ray& ray)
{

    scalar hitFraction = scalar(-1.0);

    // Get the proxy shape from the node
    rpProxyShape* proxyShape = static_cast<rpProxyShape*>(mDynamicAABBTree.getNodeDataPointer(nodeId));

    // Check if the raycast filtering mask allows raycast against this shape
    if ((mRaycastWithCategoryMaskBits & proxyShape->getCollisionCategoryBits()) != 0)
    {

        // Ask the collision detection to perform a ray cast test against
        // the proxy shape of this node because the ray is overlapping
        // with the shape in the broad-phase
        hitFraction = mRaycastTest.raycastAgainstShape(proxyShape, ray);
    }

    return hitFraction;
}




} /* namespace real_physics */
