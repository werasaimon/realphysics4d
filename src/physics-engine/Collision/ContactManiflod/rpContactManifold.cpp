/*
 * rpContactManifold.cpp
 *
 *  Created on: 28 нояб. 2016 г.
 *      Author: wera
 */

#include "rpContactManifold.h"
#include "../../config.h"


namespace real_physics
{




rpContactManifold::rpContactManifold(rpProxyShape* shape1, rpProxyShape* shape2 , short  normalDirectionId)
: mShape1(shape1), mShape2(shape2),
  mNormalDirectionId(normalDirectionId),
  mNbContactPoints(0),
  mFrictionImpulse1(0.0),
  mFrictionImpulse2(0.0),
  mFrictionTwistImpulse(0.0),
  mIsAlreadyInIsland(false)
{
	mNbContactPoints = 0;
}


// Destructor
rpContactManifold::~rpContactManifold()
{
	clear();
}

// Add a contact point in the manifold
void rpContactManifold::addContactPoint(rpContactPoint* contact)
{

	// For contact already in the manifold
	for (uint i=0; i<mNbContactPoints; i++)
	{

		// Check if the new point point does not correspond to a same contact point
		// already in the manifold.
		scalar distance = (mContactPoints[i]->getWorldPointOnBody1() - contact->getWorldPointOnBody1()).lengthSquare();
		if (distance <= PERSISTENT_CONTACT_DIST_THRESHOLD*PERSISTENT_CONTACT_DIST_THRESHOLD)
		{
			// Delete the new contact
			//contact->~ContactPoint();
			//mMemoryAllocator.release(contact, sizeof(ContactPoint));


			// Delete the new contact
			delete contact;
			assert(mNbContactPoints > 0);

			return;
		}
	}

	// If the contact manifold is full
	if (mNbContactPoints == MAX_CONTACT_POINTS_IN_MANIFOLD)
	{
		int indexMaxPenetration = getIndexOfDeepestPenetration(contact);
		int indexToRemove = getIndexToRemove(indexMaxPenetration, contact->getLocalPointOnBody1());
		removeContactPoint(indexToRemove);
	}

	// Add the new contact point in the manifold
	mContactPoints[mNbContactPoints] = contact;
	mNbContactPoints++;

	assert(mNbContactPoints > 0);

}

// Remove a contact point from the manifold
void rpContactManifold::removeContactPoint(uint index)
{
    assert(index < mNbContactPoints);
    assert(mNbContactPoints > 0);

    // Call the destructor explicitly and tell the memory allocator that
    // the corresponding memory block is now free
    delete mContactPoints[index];
           mContactPoints[index] = NULL;
   // mMemoryAllocator.release(mContactPoints[index], sizeof(ContactPoint));

    // If we don't remove the last index
    if (index < mNbContactPoints - 1)
    {
        mContactPoints[index] = mContactPoints[mNbContactPoints - 1];
    }

    mNbContactPoints--;
}



// Update the contact manifold
/// First the world space coordinates of the current contacts in the manifold are recomputed from
/// the corresponding transforms of the bodies because they have moved. Then we remove the contacts
/// with a negative penetration depth (meaning that the bodies are not penetrating anymore) and also
/// the contacts with a too large distance between the contact points in the plane orthogonal to the
/// contact normal.
void rpContactManifold::update(const Transform &transform1, const Transform &transform2)
{

    if (mNbContactPoints == 0) return;

    // Update the world coordinates and penetration depth of the contact points in the manifold
    for (uint i=0; i<mNbContactPoints; i++)
    {
        mContactPoints[i]->setWorldPointOnBody1(/*transform1 * */mContactPoints[i]->getLocalPointOnBody1());
        mContactPoints[i]->setWorldPointOnBody2(/*transform2 **/ mContactPoints[i]->getLocalPointOnBody2());
        mContactPoints[i]->setPenetrationDepth((mContactPoints[i]->getWorldPointOnBody2() -
                                                mContactPoints[i]->getWorldPointOnBody1()).dot(mContactPoints[i]->getNormal()));
    }


    const scalar squarePersistentContactThreshold = PERSISTENT_CONTACT_DIST_THRESHOLD *
                                             		PERSISTENT_CONTACT_DIST_THRESHOLD;


    // Remove the contact points that don't represent very well the contact manifold
    for (int i=static_cast<int>(mNbContactPoints)-1; i>=0; i--)
    {
    	assert(i < static_cast<int>(mNbContactPoints));

    	// Compute the distance between contact points in the normal direction
    	scalar distanceNormal = -mContactPoints[i]->getPenetrationDepth();

    	// If the contacts points are too far from each other in the normal direction
    	if (distanceNormal > squarePersistentContactThreshold)
    	{
    		removeContactPoint(i);
    	}
    	else
    	{
    		// Compute the distance of the two contact points in the plane
    		// orthogonal to the contact normal
    		Vector3 projOfPoint1 = mContactPoints[i]->getWorldPointOnBody1() + mContactPoints[i]->getNormal() * distanceNormal;
    		Vector3 projDifference = mContactPoints[i]->getWorldPointOnBody2() - projOfPoint1;

    		// If the orthogonal distance is larger than the valid distance
    		// threshold, we remove the contact
    		if (projDifference.lengthSquare() > squarePersistentContactThreshold)
    		{
    			removeContactPoint(i);
    		}
    	}
    }
}

// Return the index of the contact point with the larger penetration depth.
/// This corresponding contact will be kept in the cache. The method returns -1 is
/// the new contact is the deepest.
uint rpContactManifold::getIndexOfDeepestPenetration(rpContactPoint* newContact) const
{
    assert(mNbContactPoints == MAX_CONTACT_POINTS_IN_MANIFOLD);
    int indexMaxPenetrationDepth = -1;
    scalar maxPenetrationDepth = newContact->getPenetrationDepth();

    // For each contact in the cache
    for (uint i=0; i<mNbContactPoints; i++)
    {
        // If the current contact has a larger penetration depth
        if (mContactPoints[i]->getPenetrationDepth() > maxPenetrationDepth)
        {
            maxPenetrationDepth = mContactPoints[i]->getPenetrationDepth();
            indexMaxPenetrationDepth = i;
        }
    }

    // Return the index of largest penetration depth
    return indexMaxPenetrationDepth;
}

// Return the index that will be removed.
/// The index of the contact point with the larger penetration
/// depth is given as a parameter. This contact won't be removed. Given this contact, we compute
/// the different area and we want to keep the contacts with the largest area. The new point is also
/// kept. In order to compute the area of a quadrilateral, we use the formula :
/// Area = 0.5 * | AC x BD | where AC and BD form the diagonals of the quadrilateral. Note that
/// when we compute this area, we do not calculate it exactly but we
/// only estimate it because we do not compute the actual diagonals of the quadrialteral. Therefore,
/// this is only a guess that is faster to compute. This idea comes from the Bullet Physics library
/// by Erwin Coumans (http://wwww.bulletphysics.org).
uint rpContactManifold::getIndexToRemove(uint indexMaxPenetration, const Vector3& newPoint) const
{

    assert(mNbContactPoints == MAX_CONTACT_POINTS_IN_MANIFOLD);

    scalar area0 = 0.0;       // Area with contact 1,2,3 and newPoint
    scalar area1 = 0.0;       // Area with contact 0,2,3 and newPoint
    scalar area2 = 0.0;       // Area with contact 0,1,3 and newPoint
    scalar area3 = 0.0;       // Area with contact 0,1,2 and newPoint

    if (indexMaxPenetration != 0)
    {
        // Compute the area
        Vector3 vector1 = newPoint - mContactPoints[1]->getLocalPointOnBody1();
        Vector3 vector2 = mContactPoints[3]->getLocalPointOnBody1() - mContactPoints[2]->getLocalPointOnBody1();

        Vector3 crossProduct = vector1.cross(vector2);
        area0 = crossProduct.lengthSquare();

    }

    if (indexMaxPenetration != 1)
    {
        // Compute the area
        Vector3 vector1 = newPoint - mContactPoints[0]->getLocalPointOnBody1();
        Vector3 vector2 = mContactPoints[3]->getLocalPointOnBody1() - mContactPoints[2]->getLocalPointOnBody1();
        Vector3 crossProduct = vector1.cross(vector2);
        area1 = crossProduct.lengthSquare();

    }

    if (indexMaxPenetration != 2)
    {
        // Compute the area
        Vector3 vector1 = newPoint - mContactPoints[0]->getLocalPointOnBody1();
        Vector3 vector2 = mContactPoints[3]->getLocalPointOnBody1() - mContactPoints[1]->getLocalPointOnBody1();
        Vector3 crossProduct = vector1.cross(vector2);
        area2 = crossProduct.lengthSquare();

    }


    if (indexMaxPenetration != 3)
    {
        // Compute the area
        Vector3 vector1 = newPoint - mContactPoints[0]->getLocalPointOnBody1();
        Vector3 vector2 = mContactPoints[2]->getLocalPointOnBody1() -  mContactPoints[1]->getLocalPointOnBody1();
        Vector3 crossProduct = vector1.cross(vector2);
        area3 = crossProduct.lengthSquare();
    }

    // Return the index of the contact to remove
    return getMaxArea(area0, area1, area2, area3);
}

// Return the index of maximum area
uint rpContactManifold::getMaxArea(scalar area0, scalar area1, scalar area2, scalar area3) const
{
    if (area0 < area1)
    {
        if (area1 < area2)
        {
            if (area2 < area3) return 3;
            else return 2;
        }
        else
        {
            if (area1 < area3) return 3;
            else return 1;
        }
    }
    else
    {
        if (area0 < area2)
        {
            if (area2 < area3) return 3;
            else return 2;
        }
        else
        {
            if (area0 < area3) return 3;
            else return 0;
        }
    }
}

// Clear the contact manifold
void rpContactManifold::clear()
{
	for ( uint i=0; i<mNbContactPoints; i++)
	{
		// Call the destructor explicitly and tell the memory allocator that
		// the corresponding memory block is now free
		delete mContactPoints[i];
		       mContactPoints[i] = NULL;
	}

	mNbContactPoints = 0;
}





} /* namespace real_physics */
