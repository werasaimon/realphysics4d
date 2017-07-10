/*
 * rpCollisionManager.h
 *
 *  Created on: 23 нояб. 2016 г.
 *      Author: wera
 */

#ifndef SOURCE_ENGIE_COLLISION_RPCOLLISIONMANAGER_H_
#define SOURCE_ENGIE_COLLISION_RPCOLLISIONMANAGER_H_



// Libraries
//#include "body/CollisionBody.h"

#include <map>
#include <set>


#include "../LinearMaths/mathematics.h"
#include "../Memory/memory.h"
#include "BroadPhase/rbBroadPhaseAlgorithm.h"
#include "Manifold/rpContactManifoldSet.h"
#include "Manifold/rpContactGeneration.h"
#include "rpOverlappingPair.h"




namespace real_physics
{




// Declarations
class rpBroadPhaseAlgorithm;
class rpCollisionWorld;



// Class CollisionDetection
/**
 * This class computes the collision detection algorithms. We first
 * perform a broad-phase algorithm to know which pairs of bodies can
 * collide and then we run a narrow-phase algorithm to compute the
 * collision contacts between bodies.
 */
class rpCollisionManager
{

    private :


        // -------------------- Attributes -------------------- //


		/// Set of pair of bodies that cannot collide between each other
		std::set<bodyindexpair> mNoCollisionPairs;

		/// Broad-phase overlapping pairs
        std::map<overlappingpairid, rpOverlappingPair*> mOverlappingPairs;
        std::map<overlappingpairid, rpOverlappingPair*> mContactOverlappingPairs;


		/// Broad-phase algorithm
		rpBroadPhaseAlgorithm mBroadPhaseAlgorithm;


	    /// True if some collision shapes have been added previously
	    bool mIsCollisionShapesAdded;



        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        rpCollisionManager(const rpCollisionManager& collisionDetection);

        /// Private assignment operator
        rpCollisionManager& operator=(const rpCollisionManager& collisionDetection);

        /// Compute the broad-phase collision detection
        void computeBroadPhase();

        /// Compute the narrow-phase collision detection
        void computeNarrowPhase();

        /// Add a contact manifold to the linked list of contact manifolds of the two bodies
        /// involed in the corresponding contact.
        void addContactManifoldToBody(rpOverlappingPair* pair);

        /// Add all the contact manifold of colliding pairs to their bodies
        void addAllContactManifoldsToBodies();


	    void broadPhaseNotifyOverlappingPair( rpProxyShape* shape1 ,
	    		                              rpProxyShape* shape2 );


    public :



        // -------------------- Methods -------------------- //

        /// Constructor
         rpCollisionManager();

        /// Destructor
        ~rpCollisionManager();



        void createContact(rpOverlappingPair* overlappingPair , const rpContactPointInfo& contactInfo);
        void createContact(rpOverlappingPair* overlappingPair , rpContactPoint* contact);


        void computeCollisionDetectionAllPairs();


        /// Add a proxy collision shape to the collision detection
        void addProxyCollisionShape(rpProxyShape* proxyShape, const rpAABB& aabb);

        /// Remove a proxy collision shape from the collision detection
        void removeProxyCollisionShape(rpProxyShape* proxyShape);

        /// Update a proxy collision shape (that has moved for instance)
        void updateProxyCollisionShape(rpProxyShape* shape, const rpAABB& aabb,
        		                       const Vector3& displacement = Vector3(0, 0, 0),
									   bool forceReinsert = false);

        /// Add a pair of bodies that cannot collide with each other
        void addNoCollisionPair(rpCollisionBody* body1, rpCollisionBody* body2);

        /// Remove a pair of bodies that cannot collide with each other
        void removeNoCollisionPair(rpCollisionBody* body1, rpCollisionBody* body2);


        /// Ask for a collision shape to be tested again during broad-phase.
        void askForBroadPhaseCollisionCheck(rpProxyShape* shape);


        /// Delete all the contact points in the currently overlapping pairs
        void clearContactPoints();

        /// Ray casting method
        void raycast(RaycastCallback* raycastCallback, const Ray& ray,
                       unsigned short raycastWithCategoryMaskBits) const;

        // -------------------- Friendships -------------------- //

        friend class rpDynamicsWorld;
        friend class rpCollisionWorld;
        friend class rpConvexShape;
        friend class rpBroadPhaseAlgorithm;

        // -------------------- Friendship -------------------- //

		//            friend class rpDynamicsWorld;
		//            friend class ConvexMeshShape;
};



} /* namespace real_physics */

#endif /* SOURCE_ENGIE_COLLISION_RPCOLLISIONMANAGER_H_ */
