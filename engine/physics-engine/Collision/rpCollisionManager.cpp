/*
 * rpCollisionManager.cpp
 *
 *  Created on: 23 нояб. 2016 г.
 *      Author: wera
 */

#include "rpCollisionManager.h"


#include <stddef.h>
#include <cassert>
#include <cmath>
#include <iostream>
#include <utility>

#include "../LinearMaths/rpLinearMtah.h"
#include "../LinearMaths/rpMatrix3x3.h"
#include "../LinearMaths/rpVector3D.h"
#include "NarrowPhase/rpNarrowPhaseGjkEpaAlgorithm.h"
#include "NarrowPhase/rpNarrowPhaseMprAlgorithm.h"
#include "NarrowPhase/GJK/rpGJKAlgorithm.h"
#include "rpCollisionShapeInfo.h"
#include "rpProxyShape.h"
#include "rpRaycastInfo.h"


#include "../Body/rpBody.h"
#include "../Body/Dynamics/rpPhysicsRigidBody.h"
#include "../Body/Collision/rpCollisionBody.h"


using namespace std;


namespace real_physics
{


rpCollisionManager::rpCollisionManager()
: mBroadPhaseAlgorithm(this)
{

}


void rpCollisionManager::computeCollisionDetectionAllPairs()
{
    // Compute the broad-phase collision detection
    computeBroadPhase();

    // Compute the narrow-phase collision detection
    computeNarrowPhase();
}


void rpCollisionManager::computeBroadPhase()
{

    // If new collision shapes have been added to bodies
    if (mIsCollisionShapesAdded)
    {

        // Ask the broad-phase to recompute the overlapping pairs of collision
        // shapes. This call can only add new overlapping pairs in the collision
        // detection.
         mBroadPhaseAlgorithm.computeOverlappingPairs();
    }

}




void rpCollisionManager::computeNarrowPhase()
{


    ///-----------------------------------///
    for( auto pair : mContactOverlappingPairs )
    {
        pair.second->isFakeCollision = true;
    }


    /*********************************
     * clear memory all pairs
     ********************************
    if(!mContactOverlappingPairs.empty())
    {
        for( auto pair : mContactOverlappingPairs )
        {
            delete pair.second;
        }

        mContactOverlappingPairs.clear();
    }
    /*********************************/



    int CollisionPairNbCount = 0;

    // For each possible collision pair of bodies
    // std::map<overlappingpairid, OverlappingPair*>::iterator it;
    for (auto it = mOverlappingPairs.begin(); it != mOverlappingPairs.end(); )
    {

        rpOverlappingPair* pair = it->second;

        rpProxyShape* shape1 = pair->getShape1();
        rpProxyShape* shape2 = pair->getShape2();

        assert(shape1->mBroadPhaseID != shape2->mBroadPhaseID);

        // Check if the collision filtering allows collision between the two shapes and
        // that the two shapes are still overlapping. Otherwise, we destroy the
        // overlapping pair
        if (((shape1->getCollideWithMaskBits() & shape2->getCollisionCategoryBits()) == 0  ||
             (shape1->getCollisionCategoryBits() & shape2->getCollideWithMaskBits()) == 0) ||
             !mBroadPhaseAlgorithm.testOverlappingShapes(shape1, shape2))
        {

            std::map<overlappingpairid, rpOverlappingPair*>::iterator itToRemove = it;
            ++it;

            // TODO : Remove all the contact manifold of the overlapping pair from the contact manifolds list of the two bodies involved

            // Destroy the overlapping pair
            delete itToRemove->second;
            mOverlappingPairs.erase(itToRemove);

            continue;
        }
        else
        {
            ++it;
        }



        rpCollisionBody* const body1 = shape1->getBody();
        rpCollisionBody* const body2 = shape2->getBody();

        // Update the contact cache of the overlapping pair
        pair->update();

        // Check that at least one body is awake and not static
        bool isBody1Active = !body1->isSleeping() && body1->getType() != STATIC;
        bool isBody2Active = !body2->isSleeping() && body2->getType() != STATIC;
        if (!isBody1Active && !isBody2Active) continue;

        // Check if the bodies are in the set of bodies that cannot collide between each other
        bodyindexpair bodiesIndex = rpOverlappingPair::computeBodiesIndexPair(body1, body2);
        if (mNoCollisionPairs.count(bodiesIndex) > 0) continue;

        CollisionPairNbCount++;



        /**********************************************************************/

        rpNarrowPhaseCollisionAlgorithm* narrowPhaseAlgorithm = new rpNarrowPhaseGjkEpaAlgorithm;// mCollisionMatrix[shape1Type][shape2Type];

        // If there is no collision algorithm between those two kinds of shapes
        if (narrowPhaseAlgorithm == NULL) continue;

        // Notify the narrow-phase algorithm about the overlapping pair we are going to test
        narrowPhaseAlgorithm->setCurrentOverlappingPair(pair);

        // Create the CollisionShapeInfo objects
        rpCollisionShapeInfo shape1Info( shape1->getCollisionShape(),
                                         shape1->getWorldTransform(),
                                         shape1->getCachedCollisionData());

        rpCollisionShapeInfo shape2Info( shape2->getCollisionShape(),
                                         shape2->getWorldTransform(),
                                         shape2->getCachedCollisionData());



        // Use the narrow-phase collision detection algorithm to check
        // if there really is a collision. If a collision occurs, the
        // notifyContact() callback method will be called.
        OutContactInfo infoContact;
        bool testCollision = (narrowPhaseAlgorithm->testCollision( shape2Info , shape1Info , infoContact ));
        if(testCollision)
        {

            Vector3 normal = infoContact.m_normal;
            scalar  penetration = infoContact.m_penetrationDepth;

            overlappingpairid pairId = rpOverlappingPair::computeID(shape1,  shape2);

            if( mContactOverlappingPairs.find(pairId) == mContactOverlappingPairs.end())
            {
                const int maxContacts = 2;
                mContactOverlappingPairs.insert( std::make_pair( pairId , new rpOverlappingPair(shape1,shape2,maxContacts)) );
            }


            //const int maxContacts = 2;
            //mContactOverlappingPairs[pairId] = new rpOverlappingPair(shape1,shape2,maxContacts);

            rpContactGeneration generatorManiflod( shape1 , shape2 , normal );
            generatorManiflod.computeContacteOverlappingPair( mContactOverlappingPairs[pairId] , this );

            mContactOverlappingPairs[pairId]->update();

            mContactOverlappingPairs[pairId]->isFakeCollision = false;
            mContactOverlappingPairs[pairId]->setCachedSeparatingAxis(normal);
            mContactOverlappingPairs[pairId]->mContactManifoldSet.setExtermalPenetration(penetration);

        }


        delete narrowPhaseAlgorithm;
        /*********************************************************************/

    }

    // Delete contacts
    for( auto pair : mContactOverlappingPairs )
    {
        if(pair.second->isFakeCollision)
        {
            delete pair.second;
            mContactOverlappingPairs.erase(pair.first);
        }
    }


    // Add all the contact manifolds (between colliding bodies) to the bodies
    addAllContactManifoldsToBodies();


}



void rpCollisionManager::addAllContactManifoldsToBodies()
{
    for (auto it = mContactOverlappingPairs.begin(); it != mContactOverlappingPairs.end(); ++it)
    {
          // Add all the contact manifolds of the pair into the list of contact manifolds
          // of the two bodies involved in the contact
          addContactManifoldToBody(it->second);
    }

}

void rpCollisionManager::addContactManifoldToBody(rpOverlappingPair *pair)
{

    assert(pair != NULL);

    rpCollisionBody* body1 = pair->getShape1()->getBody();
    rpCollisionBody* body2 = pair->getShape2()->getBody();
    const rpContactManifoldSet& manifoldSet = pair->getContactManifoldSet();

    // For each contact manifold in the set of manifolds in the pair
    for (int i=0; i<manifoldSet.getNbContactManifolds(); i++)
    {

        rpContactManifold* contactManifold = manifoldSet.getContactManifold(i);

        assert(contactManifold->getNbContactPoints() > 0);

        // Add the contact manifold at the beginning of the linked
        // list of contact manifolds of the first body
        ContactManifoldListElement *listElement1 = new ContactManifoldListElement( contactManifold , body1->mContactManifoldsList , NULL );
        body1->mContactManifoldsList = listElement1;


        // Add the contact manifold at the beginning of the linked
        // list of the contact manifolds of the second body
        ContactManifoldListElement *listElement2 = new ContactManifoldListElement( contactManifold , body2->mContactManifoldsList , NULL );
        body2->mContactManifoldsList = listElement2;

    }




}

void rpCollisionManager::broadPhaseNotifyOverlappingPair( rpProxyShape* shape1 ,
                                                          rpProxyShape* shape2 )
{

	assert(shape1->mBroadPhaseID != shape2->mBroadPhaseID);

	// If the two proxy collision shapes are from the same body, skip it
	if (shape1->getBody()->getID() == shape2->getBody()->getID()) return;

	// Check if the collision filtering allows collision between the two shapes
	if ((shape1->getCollideWithMaskBits() & shape2->getCollisionCategoryBits()) == 0 ||
		(shape1->getCollisionCategoryBits() & shape2->getCollideWithMaskBits()) == 0) return;

	// Compute the overlapping pair ID
	overlappingpairid pairID = rpOverlappingPair::computeID(shape1, shape2);

	// Check if the overlapping pair already exists
	if (mOverlappingPairs.find(pairID) != mOverlappingPairs.end()) return;

	// Compute the maximum number of contact manifolds for this pair
	//    int nbMaxManifolds = CollisionShape::computeNbMaxContactManifolds(shape1->getCollisionShape()->getType(),
	//                                                                      shape2->getCollisionShape()->getType());

	// Create the overlapping pair and add it into the set of overlapping pairs
	rpOverlappingPair* newPair = new rpOverlappingPair(shape1, shape2);
	assert(newPair != NULL);

#ifndef NDEBUG
	std::pair<std::map<overlappingpairid, rpOverlappingPair*>::iterator, bool> check =
#endif
			mOverlappingPairs.insert(make_pair(pairID, newPair));

	assert(check.second);

    // Wake up the two bodies
    shape1->getBody()->setIsSleeping(false);
    shape2->getBody()->setIsSleeping(false);

}

rpCollisionManager::~rpCollisionManager()
{

}


// Create a new contact
void rpCollisionManager::createContact(rpOverlappingPair *overlappingPair , const rpContactPointInfo& contactInfo )
{
    // Create a new contact
    rpContactPoint* contact = new rpContactPoint(contactInfo);

    // Add the contact to the contact manifold set of the corresponding overlapping pair
    overlappingPair->addContact(contact);

    // Add the overlapping pair into the set of pairs in contact during narrow-phase
    overlappingpairid pairId = rpOverlappingPair::computeID(overlappingPair->getShape1(),
                                                            overlappingPair->getShape2());
    mContactOverlappingPairs[pairId] = overlappingPair;
}



//void rpCollisionManager::createContact(rpOverlappingPair *overlappingPair,  rpContactPoint *contact )
//{
//    // Add the contact to the contact manifold set of the corresponding overlapping pair
//    overlappingPair->addContact(contact);

//    // Add the overlapping pair into the set of pairs in contact during narrow-phase
//    overlappingpairid pairId = rpOverlappingPair::computeID(overlappingPair->getShape1(),
//                                                            overlappingPair->getShape2());
//    mContactOverlappingPairs[pairId] = overlappingPair;
//}



void rpCollisionManager::addProxyCollisionShape(rpProxyShape* proxyShape, const rpAABB& aabb)
{
	// Add the body to the broad-phase
	mBroadPhaseAlgorithm.addProxyCollisionShape(proxyShape, aabb);

	mIsCollisionShapesAdded = true;
}

void rpCollisionManager::removeProxyCollisionShape(rpProxyShape* proxyShape)
{

	// Remove all the overlapping pairs involving this proxy shape
    for ( auto it = mOverlappingPairs.begin(); it != mOverlappingPairs.end(); )
	{
		if (it->second->getShape1()->mBroadPhaseID == proxyShape->mBroadPhaseID||
			it->second->getShape2()->mBroadPhaseID == proxyShape->mBroadPhaseID)
		{
			std::map<overlappingpairid, rpOverlappingPair*>::iterator itToRemove = it;
			++it;

			// TODO : Remove all the contact manifold of the overlapping pair from the contact manifolds list of the two bodies involved

            // Destroy the overlapping pair
            //itToRemove->second->~rpOverlappingPair();
            //itToRemove->second->mContactManifoldSet.clear();

			// Destroy the overlapping pair
			delete itToRemove->second;
			mOverlappingPairs.erase(itToRemove);
		}
		else
		{
			++it;
		}
	}

	    // Remove the body from the broad-phase
	    mBroadPhaseAlgorithm.removeProxyCollisionShape(proxyShape);

}


void rpCollisionManager::updateProxyCollisionShape(rpProxyShape* shape, const rpAABB& aabb,
                                                     const Vector3& displacement, bool forceReinsert)
{
	  mBroadPhaseAlgorithm.updateProxyCollisionShape(shape, aabb, displacement);

}

void rpCollisionManager::addNoCollisionPair(rpCollisionBody* body1, rpCollisionBody* body2)
{
	 mNoCollisionPairs.insert(rpOverlappingPair::computeBodiesIndexPair(body1, body2));
}

void rpCollisionManager::removeNoCollisionPair(rpCollisionBody* body1, rpCollisionBody* body2)
{
	 mNoCollisionPairs.erase(rpOverlappingPair::computeBodiesIndexPair(body1, body2));
}

void rpCollisionManager::askForBroadPhaseCollisionCheck(rpProxyShape* shape)
{
    mBroadPhaseAlgorithm.addMovedCollisionShape(shape->mBroadPhaseID);
}

/// Delete all the contact points in the currently overlapping pairs
void rpCollisionManager::clearContactPoints()
{
    // For each overlapping pair
     for (auto it = mOverlappingPairs.begin(); it != mOverlappingPairs.end(); ++it)
     {
         it->second->clearContactPoints();
     }
}

void rpCollisionManager::raycast(RaycastCallback* raycastCallback, const Ray& ray,
		                           unsigned short raycastWithCategoryMaskBits) const
{
	    RaycastTest rayCastTest(raycastCallback);

	    // Ask the broad-phase algorithm to call the testRaycastAgainstShape()
	    // callback method for each proxy shape hit by the ray in the broad-phase
	    mBroadPhaseAlgorithm.raycast(ray, rayCastTest, raycastWithCategoryMaskBits);
}

} /* namespace real_physics */


