/*
 * rpCollisionDetection.cpp
 *
 *  Created on: 23 нояб. 2016 г.
 *      Author: wera
 */

#include "rpCollisionDetection.h"


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


#include "../Dynamics/Body/rpRigidPhysicsBody.h"
#include "Body/rpBody.h"
#include "Body/rpCollisionBody.h"

using namespace std;


namespace real_physics
{


/**
bool m_bAllowIntersections = true;

void GetInterval_CenExt( const rpProxyShape* shape , const Vector3& xAxis, float& cen, float& ext)
{
	float min, max;
    shape->getIntervalWorld(xAxis, min, max);
	ext = (max - min) * 0.5f;
	cen = (max + min) * 0.5f;
}


void GetInterval_MinMax( const rpProxyShape* shape , const Vector3& xAxis, float& min, float& max)
{
	float cen, ext;
	GetInterval_CenExt( shape , xAxis, cen, ext);
	min = cen - ext;
	max = cen + ext;
}


bool IntervalIntersects(const rpProxyShape* shapeA ,				    // A interval
		                const rpProxyShape* shapeB ,				    // B interval
						const Vector3& xAxis, 						// static parameters
						const Vector3& xVel,				    	// dynamic parameters
						bool& bValidMTD,
						scalar& tfirst , scalar& tlast,
						Vector3& Nfirst, Vector3& Nlast,	        // collision
						Vector3& MTD)						   	    // intersection
{

	const scalar INVALID_FLOAT = 1.0E20f;
	const scalar COLLISION_VELOCITY_THRESHOLD = 1.0E-7f;

	//---------------------------------------------------
	// projection calculations
	//---------------------------------------------------
	float minb, maxb;
	float c, e;

	// both intervals
	GetInterval_CenExt( shapeA , xAxis, c, e);
	GetInterval_MinMax( shapeB , xAxis, minb, maxb);

	// reduce the problem of a single point versus a larger interval
	minb -= e;
	maxb += e;
	//---------------------------------------------------
	// intersection test calculations
	//---------------------------------------------------
	// the two potential overlaps
	scalar d0 = minb - c;
	scalar d1 = maxb - c;

	// are the objects separated along that axis?
	bool bIntersect = (d0 > 0.0f || d1 > 0.0f);


	scalar fAxisLengthSquared = 1.0;//xAxis.lengthSquare();

	// this mode deals with intersections
	if (m_bAllowIntersections)
	{
		// if so, then we can have no potential interection
		bValidMTD &= bIntersect;

		// if the intersection is still potentially valid
		if (bValidMTD)
		{
			// Find the MTD along that axis
			// then update the global MTD with it if it is smaller.
			Vector3 Sep = xAxis;

			if (fabs(d0) < fabs(d1))
			{
				Sep *= -d0 / fAxisLengthSquared;
			}
			else
			{
				Sep *= -d1 / fAxisLengthSquared;
			}

			if (Sep.lengthSquare() < MTD.lengthSquare())
			{
				MTD = Sep;
			}
		}
	}


	//---------------------------------------------------
	// collision test calculations
	//---------------------------------------------------
	scalar v = xVel.dot(xAxis);

	// boxes virtual never move on that axis.
	// it's worth continuing only if the box are not separated.
	if (fabs(v) < 1.0E-6f)
	{
		return bIntersect;
	}

	//---------------------------------------------------
	// time of intersection along that axis
	//---------------------------------------------------
	scalar t0 = -(minb - c) / v;
	scalar t1 = -(maxb - c) / v;
	scalar sign = -1.0f;

	//---------------------------------------------------
	// Update the overall times of collision
	//---------------------------------------------------
	// order the times of collision properly
	if (t0 > t1)
	{
		Swap(t0, t1);
		sign = 1.0f;
	}

	// make sure the axis intersection provides a valid intersection interval
	// with the global intersection interval
	if(tlast != INVALID_FLOAT && t0 > tlast)
	{
		return false;
	}

	if (tfirst != -INVALID_FLOAT && t1 < tfirst)
	{
		return false;
	}

	// then reduce the global intersection interval
	if (t1 < tlast || tlast == INVALID_FLOAT)
	{
		tlast = t1;
		Nlast = xAxis * sign;
	}

	if (t0 > tfirst || tfirst == -INVALID_FLOAT)
	{
		tfirst = t0;
		Nfirst = xAxis * -sign;
	}

	return true;
}



bool  timeOfImpact( const rpProxyShape* shape_a ,
		            const rpProxyShape* shape_b  ,
		            const Vector3& xAxis , const Vector3& xVel ,
					scalar tmax, scalar& tcoll, Vector3& Ncoll )
{

	Vector3 MTD;
	bool bValidMTD = true;


	const scalar INVALID_FLOAT = 0.0;

	// Collision test. Calculate the min and max times of collisions of the objects
	// could be negative (then the data returned will be an intersection MTD data).
	tcoll = tmax;
	scalar tfirst = -INVALID_FLOAT;
	scalar tlast  =  INVALID_FLOAT;
	Vector3 Nfirst = Vector3::ZERO;
	Vector3 Nlast  = Vector3::ZERO;




	if(!IntervalIntersects( shape_a , shape_b , xAxis, xVel, bValidMTD, tfirst, tlast, Nfirst, Nlast, MTD))
	{
		return false;
	}


	// boxes miss collision
	if (tfirst > tmax || tlast < 0.0f)
	{
		return false;
	}



	// boxes are intersecting (or velocity too small)
	if (m_bAllowIntersections)
	{
		if (tfirst <= 0.0f)
		{
			if (!bValidMTD)
			{
				return false;
			}

			tcoll = 0.0f;
			Ncoll = MTD;
			return true;
		}
		// boxes are colliding
		else
		{
			tcoll = tfirst;
			Ncoll = Nfirst;
			return true;
		}
	}
	else
	{
		if (fabs(tfirst) <= fabs(tlast))
		{
			tcoll = tfirst;
			Ncoll = Nfirst;
			return true;
		}
		else
		{
			tcoll = tlast;
			Ncoll = Nlast;
			return true;
		}
	}
}

/**/


rpCollisionDetection::rpCollisionDetection()
: mBroadPhaseAlgorithm(this)
{

}


void rpCollisionDetection::computeCollisionDetection( std::map<overlappingpairid, rpOverlappingPair*> &OverlappingPairs )
{


    // Compute the broad-phase collision detection
    computeBroadPhase();

    // Compute the narrow-phase collision detection
    computeNarrowPhase(OverlappingPairs);
}


void rpCollisionDetection::computeBroadPhase()
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




void rpCollisionDetection::computeNarrowPhase( std::map<overlappingpairid, rpOverlappingPair*> &ContactOverlappingPairs )
{


	/*********************************
	 * clear memory all pairs
	 ********************************
	if(!ContactOverlappingPairs.empty())
	{
		for( auto pair : ContactOverlappingPairs )
		{
			delete pair.second;
		}

	  ContactOverlappingPairs.clear();
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
	        if (((shape1->getCollideWithMaskBits() & shape2->getCollisionCategoryBits()) == 0 ||
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
            //pair->update();

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


				//	        const Vector3 &p1 = shape1->getWorldTransform().getPosition();
				//	        const Vector3 &p2 = shape2->getWorldTransform().getPosition();
				//	        glPushMatrix();
				//	        glColor3f(0, 0, 1);
				//	        glLineWidth(2);
				//	        glBegin(GL_LINES);
				//	           glVertex3f(p1.x, p1.y, p1.z);
				//	           glVertex3f(p2.x, p2.y, p2.z);
				//	        glEnd();
				//	        glPopMatrix();


	        	// Use the narrow-phase collision detection algorithm to check
	        	// if there really is a collision. If a collision occurs, the
	        	// notifyContact() callback method will be called.
	        	OutContactInfo infoContact;
                bool testCollision = (narrowPhaseAlgorithm->testCollision( shape2Info , shape1Info , infoContact ));



                /**
                rpRigidPhysicsBody* physBdoy1 = static_cast<rpRigidPhysicsBody*>(body1);
                rpRigidPhysicsBody* physBdoy2 = static_cast<rpRigidPhysicsBody*>(body2);
                const Vector3& xVel = physBdoy1->getLinearVelocity() -
                                      physBdoy2->getLinearVelocity();

                scalar  dt = scalar(1.0/60.0);
                scalar  tcoll;
                Vector3 Ncoll;
                Vector3 Axis = infoContact.m_normal;
                bool collideSweep = (timeOfImpact(shape1, shape2 , Axis , xVel , dt , tcoll , Ncoll));
                /**/


                if(testCollision)
	        	{


	        		Vector3 normal = infoContact.m_normal.getUnit();
	        		scalar  penetration = infoContact.m_penetrationDepth;

	        		//normal = Ncoll;
	        		//penetration = tcoll;

	        		const int maxContacts = 1;
	        		overlappingpairid pairId = rpOverlappingPair::computeID(shape1,  shape2);
	        		ContactOverlappingPairs[pairId] = new rpOverlappingPair(shape1,shape2, maxContacts);


	        		rpGenerationContactManiflodSet generatorManiflod( shape1 , shape2 , normal );
	        		generatorManiflod.computeContacteManiflodSet(ContactOverlappingPairs[pairId]->mContactManifoldSet);

	        		//ContactOverlappingPairs[pairId]->mContactManifoldSet.setPairIndex(IdIndexCollid2 , IdIndexCollid1);

	        		ContactOverlappingPairs[pairId]->setCachedSeparatingAxis(normal);
	        		ContactOverlappingPairs[pairId]->mContactManifoldSet.setExtermalPenetration(penetration);

	        	}


	        delete narrowPhaseAlgorithm;
	        /*********************************************************************/

	    }


        // Add all the contact manifolds (between colliding bodies) to the bodies
        addAllContactManifoldsToBodies(ContactOverlappingPairs);

}



void rpCollisionDetection::addAllContactManifoldsToBodies( std::map<overlappingpairid, rpOverlappingPair*> &ContactOverlappingPairs )
{
    for (auto it = ContactOverlappingPairs.begin(); it != ContactOverlappingPairs.end(); ++it)
    {
          // Add all the contact manifolds of the pair into the list of contact manifolds
          // of the two bodies involved in the contact
          addContactManifoldToBody(it->second);
    }

}

void rpCollisionDetection::addContactManifoldToBody(rpOverlappingPair *pair)
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
        rpContactManifoldListElement* listElement1 = new rpContactManifoldListElement( contactManifold , body1->mContactManifoldsList );
        body1->mContactManifoldsList = listElement1;


        // Add the contact manifold at the beginning of the linked
        // list of the contact manifolds of the second body
        rpContactManifoldListElement* listElement2 = new rpContactManifoldListElement( contactManifold , body2->mContactManifoldsList );
        body2->mContactManifoldsList = listElement2;
    }

}

void rpCollisionDetection::broadPhaseNotifyOverlappingPair( rpProxyShape* shape1 ,
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

rpCollisionDetection::~rpCollisionDetection()
{

}

void rpCollisionDetection::addProxyCollisionShape(rpProxyShape* proxyShape, const rpAABB& aabb)
{
	// Add the body to the broad-phase
	mBroadPhaseAlgorithm.addProxyCollisionShape(proxyShape, aabb);

	mIsCollisionShapesAdded = true;
}

void rpCollisionDetection::removeProxyCollisionShape(rpProxyShape* proxyShape)
{

	// Remove all the overlapping pairs involving this proxy shape
	//std::map<overlappingpairid, OverlappingPair*>::iterator it;
	for (auto it = mOverlappingPairs.begin(); it != mOverlappingPairs.end(); )
	{
		if (it->second->getShape1()->mBroadPhaseID == proxyShape->mBroadPhaseID||
			it->second->getShape2()->mBroadPhaseID == proxyShape->mBroadPhaseID)
		{
			std::map<overlappingpairid, rpOverlappingPair*>::iterator itToRemove = it;
			++it;

			// TODO : Remove all the contact manifold of the overlapping pair from the contact manifolds list of the two bodies involved

			// Destroy the overlapping pair
			itToRemove->second->~rpOverlappingPair();
			itToRemove->second->mContactManifoldSet.clear();

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


void rpCollisionDetection::updateProxyCollisionShape(rpProxyShape* shape, const rpAABB& aabb,
                                                     const Vector3& displacement, bool forceReinsert)
{
	  mBroadPhaseAlgorithm.updateProxyCollisionShape(shape, aabb, displacement);

}

void rpCollisionDetection::addNoCollisionPair(rpCollisionBody* body1, rpCollisionBody* body2)
{
	 mNoCollisionPairs.insert(rpOverlappingPair::computeBodiesIndexPair(body1, body2));
}

void rpCollisionDetection::removeNoCollisionPair(rpCollisionBody* body1, rpCollisionBody* body2)
{
	 mNoCollisionPairs.erase(rpOverlappingPair::computeBodiesIndexPair(body1, body2));
}

void rpCollisionDetection::askForBroadPhaseCollisionCheck(rpProxyShape* shape)
{
	mBroadPhaseAlgorithm.addMovedCollisionShape(shape->mBroadPhaseID);
}

void rpCollisionDetection::raycast(RaycastCallback* raycastCallback, const Ray& ray,
		                           unsigned short raycastWithCategoryMaskBits) const
{
	    RaycastTest rayCastTest(raycastCallback);

	    // Ask the broad-phase algorithm to call the testRaycastAgainstShape()
	    // callback method for each proxy shape hit by the ray in the broad-phase
	    mBroadPhaseAlgorithm.raycast(ray, rayCastTest, raycastWithCategoryMaskBits);
}

} /* namespace real_physics */


