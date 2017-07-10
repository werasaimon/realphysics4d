/*
 * rbBroadPhaseAlgorithm.h
 *
 *  Created on: 23 нояб. 2016 г.
 *      Author: wera
 */

#ifndef SOURCE_ENGIE_COLLISION_BROADPHASE_RBBROADPHASEALGORITHM_H_
#define SOURCE_ENGIE_COLLISION_BROADPHASE_RBBROADPHASEALGORITHM_H_




// Libraries
#include <vector>
//#include "body/CollisionBody.h"
#include "../rpProxyShape.h"
#include "rpDynamicAABBTree.h"

#include "../../LinearMaths/mathematics.h"


namespace real_physics
{




// Declarations
class rpCollisionManager;
class rpBroadPhaseAlgorithm;

// Structure BroadPhasePair
/**
 * This structure represent a potential overlapping pair during the
 * broad-phase collision detection.
 */
struct rpBroadPhasePair
{
    // -------------------- Attributes -------------------- //

    /// Broad-phase ID of the first collision shape
    int collisionShape1ID;

    /// Broad-phase ID of the second collision shape
    int collisionShape2ID;

    // -------------------- Methods -------------------- //

    /// Method used to compare two pairs for sorting algorithm
    static bool smallerThan(const rpBroadPhasePair& pair1, const rpBroadPhasePair& pair2);
};







// class AABBOverlapCallback
class rpAABBOverlapCallback : public rpDynamicAABBTreeOverlapCallback
{

    private:

	rpBroadPhaseAlgorithm& mBroadPhaseAlgorithm;

        int mReferenceNodeId;

    public:

        // Constructor
        rpAABBOverlapCallback(rpBroadPhaseAlgorithm& broadPhaseAlgo, int referenceNodeId)
         : mBroadPhaseAlgorithm(broadPhaseAlgo),
		   mReferenceNodeId(referenceNodeId)
        {

        }

        // Called when a overlapping node has been found during the call to
        // DynamicAABBTree:reportAllShapesOverlappingWithAABB()
        virtual void notifyOverlappingNode(int nodeId);

};




// Class BroadPhaseRaycastCallback
/**
 * Callback called when the AABB of a leaf node is hit by a ray the
 * broad-phase Dynamic AABB Tree.
 */
class rpBroadPhaseRaycastCallback : public rpDynamicAABBTreeRaycastCallback
{

    private :

        const rpDynamicAABBTree& mDynamicAABBTree;

        unsigned short mRaycastWithCategoryMaskBits;

        RaycastTest& mRaycastTest;

    public:

        // Constructor
        rpBroadPhaseRaycastCallback(const rpDynamicAABBTree& dynamicAABBTree,
        		                  unsigned short raycastWithCategoryMaskBits,
                                  RaycastTest& raycastTest)
            : mDynamicAABBTree(dynamicAABBTree),
			  mRaycastWithCategoryMaskBits(raycastWithCategoryMaskBits),
              mRaycastTest(raycastTest)
        {

        }

        // Called for a broad-phase shape that has to be tested for raycast
        virtual scalar raycastBroadPhaseShape(int32 nodeId, const Ray& ray);

};



// Class BroadPhaseAlgorithm
/**
 * This class represents the broad-phase collision detection. The
 * goal of the broad-phase collision detection is to compute the pairs of proxy shapes
 * that have their AABBs overlapping. Only those pairs of bodies will be tested
 * later for collision during the narrow-phase collision detection. A dynamic AABB
 * tree data structure is used for fast broad-phase collision detection.
 */
class rpBroadPhaseAlgorithm
{

    protected :

        // -------------------- Attributes -------------------- //

        /// Dynamic AABB tree
	    rpDynamicAABBTree mDynamicAABBTree;

        /// Array with the broad-phase IDs of all collision shapes that have moved (or have been
        /// created) during the last simulation step. Those are the shapes that need to be tested
        /// for overlapping in the next simulation step.
        int* mMovedShapes;

        /// Number of collision shapes in the array of shapes that have moved during the last
        /// simulation step.
        uint mNbMovedShapes;

        /// Number of allocated elements for the array of shapes that have moved during the last
        /// simulation step.
        uint mNbAllocatedMovedShapes;

        /// Number of non-used elements in the array of shapes that have moved during the last
        /// simulation step.
        uint mNbNonUsedMovedShapes;

        /// Temporary array of potential overlapping pairs (with potential duplicates)
        rpBroadPhasePair* mPotentialPairs;

        /// Number of potential overlapping pairs
        uint mNbPotentialPairs;

        /// Number of allocated elements for the array of potential overlapping pairs
        uint mNbAllocatedPotentialPairs;

        /// Reference to the collision detection object
        rpCollisionManager *mCollisionDetection;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        rpBroadPhaseAlgorithm(const rpBroadPhaseAlgorithm& algorithm);

        /// Private assignment operator
        rpBroadPhaseAlgorithm& operator=(const rpBroadPhaseAlgorithm& algorithm);

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        rpBroadPhaseAlgorithm(rpCollisionManager *collisionDetection);

        /// Destructor
        ~rpBroadPhaseAlgorithm();

        /// Add a proxy collision shape into the broad-phase collision detection
        void addProxyCollisionShape(rpProxyShape* proxyShape, const rpAABB& aabb);

        /// Remove a proxy collision shape from the broad-phase collision detection
        void removeProxyCollisionShape(rpProxyShape* proxyShape);

        /// Notify the broad-phase that a collision shape has moved and need to be updated
        void updateProxyCollisionShape(rpProxyShape* proxyShape, const rpAABB& aabb, const Vector3& displacement, bool forceReinsert = false);

        /// Add a collision shape in the array of shapes that have moved in the last simulation step
        /// and that need to be tested again for broad-phase overlapping.
        void addMovedCollisionShape(int broadPhaseID);

        /// Remove a collision shape from the array of shapes that have moved in the last simulation
        /// step and that need to be tested again for broad-phase overlapping.
        void removeMovedCollisionShape(int broadPhaseID);

        /// Notify the broad-phase about a potential overlapping pair in the dynamic AABB tree
        void notifyOverlappingNodes(int broadPhaseId1, int broadPhaseId2);

        /// Compute all the overlapping pairs of collision shapes
        void computeOverlappingPairs();

        /// Return true if the two broad-phase collision shapes are overlapping
        bool testOverlappingShapes(const rpProxyShape* shape1, const rpProxyShape* shape2) const;

        /// Ray casting method
        void raycast(const Ray& ray, RaycastTest& raycastTest , unsigned short raycastWithCategoryMaskBits) const;
};

// Method used to compare two pairs for sorting algorithm
SIMD_INLINE bool rpBroadPhasePair::smallerThan(const rpBroadPhasePair& pair1, const rpBroadPhasePair& pair2)
{

    if (pair1.collisionShape1ID < pair2.collisionShape1ID) return true;
    if (pair1.collisionShape1ID == pair2.collisionShape1ID)
    {
        return pair1.collisionShape2ID < pair2.collisionShape2ID;
    }
    return false;
}

// Return true if the two broad-phase collision shapes are overlapping
SIMD_INLINE bool rpBroadPhaseAlgorithm::testOverlappingShapes(const rpProxyShape* shape1,
                                                         const rpProxyShape* shape2) const
{
    // Get the two AABBs of the collision shapes
    const rpAABB& aabb1 = mDynamicAABBTree.getFatAABB(shape1->mBroadPhaseID);
    const rpAABB& aabb2 = mDynamicAABBTree.getFatAABB(shape2->mBroadPhaseID);

    // Check if the two AABBs are overlapping
    return aabb1.testCollision(aabb2);
}

// Ray casting method
SIMD_INLINE void rpBroadPhaseAlgorithm::raycast(const Ray& ray, RaycastTest& raycastTest,
                                         unsigned short raycastWithCategoryMaskBits) const
{

    //PROFILE("BroadPhaseAlgorithm::raycast()");

    rpBroadPhaseRaycastCallback broadPhaseRaycastCallback(mDynamicAABBTree, raycastWithCategoryMaskBits, raycastTest);
    mDynamicAABBTree.raycast(ray, broadPhaseRaycastCallback);
}





} /* namespace real_physics */

#endif /* SOURCE_ENGIE_COLLISION_BROADPHASE_RBBROADPHASEALGORITHM_H_ */
