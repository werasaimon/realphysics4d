/*
 * rpContactManager.h
 *
 *  Created on: 23 нояб. 2016 г.
 *      Author: wera
 */

#ifndef SOURCE_ENGIE_COLLISION_RPCONTACTMANAGER_H_
#define SOURCE_ENGIE_COLLISION_RPCONTACTMANAGER_H_



// Libraries
//#include "body/CollisionBody.h"

#include <map>
#include <set>


#include "../LinearMaths/mathematics.h"
#include "../Memory/memory.h"
#include "BroadPhase/rbBroadPhaseAlgorithm.h"
#include "ContactManiflod/rpContactManifoldSet.h"
#include "ContactManiflod/rpGenerationContactManiflodSet.h"
#include "rpOverlappingPair.h"




namespace real_physics
{




// Declarations
class rpBroadPhaseAlgorithm;
class rpCollisionWorld;
class CollisionCallback;




// Class CollisionDetection
/**
 * This class computes the collision detection algorithms. We first
 * perform a broad-phase algorithm to know which pairs of bodies can
 * collide and then we run a narrow-phase algorithm to compute the
 * collision contacts between bodies.
 */
class rpContactManager
{

    private :


        // -------------------- Attributes -------------------- //


		/// Set of pair of bodies that cannot collide between each other
		std::set<bodyindexpair> mNoCollisionPairs;

		/// Broad-phase overlapping pairs
		std::map<overlappingpairid, rpOverlappingPair*> mOverlappingPairs;


		/// Broad-phase algorithm
		rpBroadPhaseAlgorithm mBroadPhaseAlgorithm;


	    /// True if some collision shapes have been added previously
	    bool mIsCollisionShapesAdded;



        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        rpContactManager(const rpContactManager& collisionDetection);

        /// Private assignment operator
        rpContactManager& operator=(const rpContactManager& collisionDetection);

        /// Compute the broad-phase collision detection
        void computeBroadPhase();

        /// Compute the narrow-phase collision detection
        void computeNarrowPhase( std::map<overlappingpairid, rpOverlappingPair*>& ContactOverlappingPairs );

        /// Add a contact manifold to the linked list of contact manifolds of the two bodies
        /// involed in the corresponding contact.
        void addContactManifoldToBody(rpOverlappingPair* pair);

        /// Add all the contact manifold of colliding pairs to their bodies
        void addAllContactManifoldsToBodies(std::map<overlappingpairid, rpOverlappingPair *> &ContactOverlappingPairs);


	    void broadPhaseNotifyOverlappingPair( rpProxyShape* shape1 ,
	    		                              rpProxyShape* shape2 );


    public :



        // -------------------- Methods -------------------- //

        /// Constructor
         rpContactManager();

        /// Destructor
        ~rpContactManager();





        void computeCollisionDetectionAllPairs(  std::map<overlappingpairid, rpOverlappingPair*> &OverlappingPairs );


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


        /// Ray casting method
        void raycast(RaycastCallback* raycastCallback, const Ray& ray,
                       unsigned short raycastWithCategoryMaskBits) const;

        // -------------------- Friendships -------------------- //

        friend class rpDynamicsWorld;
        friend class rpConvexShape;
        friend class rpBroadPhaseAlgorithm;

        // -------------------- Friendship -------------------- //

		//            friend class rpDynamicsWorld;
		//            friend class ConvexMeshShape;
};



} /* namespace real_physics */

#endif /* SOURCE_ENGIE_COLLISION_RPCONTACTMANAGER_H_ */
