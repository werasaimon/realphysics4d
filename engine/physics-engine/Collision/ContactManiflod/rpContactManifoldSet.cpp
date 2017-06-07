/*
 * rpContactManifoldSet.cpp
 *
 *  Created on: 28 нояб. 2016 г.
 *      Author: wera
 */

#include "rpContactManifoldSet.h"
#include "rpContactManifold.h"

namespace real_physics
{


// Constructor
rpContactManifoldSet::rpContactManifoldSet(rpProxyShape* shape1, rpProxyShape* shape2,
                                       int nbMaxManifolds)
                   : mNbMaxManifolds(nbMaxManifolds), mNbManifolds(0),
					 mShape1(shape1),
                     mShape2(shape2)
{
    assert(nbMaxManifolds >= 1);
}

// Destructor
rpContactManifoldSet::~rpContactManifoldSet()
{
    // Clear all the contact manifolds
    clear();
}

// Add a contact point to the manifold set
void rpContactManifoldSet::addContactPoint(rpContactPoint* contact)
{

    // Compute an Id corresponding to the normal direction (using a cubemap)
    short int normalDirectionId = computeCubemapNormalId(contact->getNormal());

    // If there is no contact manifold yet
    if (mNbManifolds == 0)
    {

        createManifold(normalDirectionId);
        mManifolds[0]->addContactPoint(contact);
        assert(mManifolds[mNbManifolds-1]->getNbContactPoints() > 0);
        for (int i=0; i<mNbManifolds; i++)
        {
            assert(mManifolds[i]->getNbContactPoints() > 0);
        }

        return;
    }

    // Select the manifold with the most similar normal (if exists)
    int similarManifoldIndex = 0;
    if (mNbMaxManifolds > 1)
    {
        similarManifoldIndex = selectManifoldWithSimilarNormal(normalDirectionId);
    }

    // If a similar manifold has been found
    if (similarManifoldIndex != -1)
    {

        // Add the contact point to that similar manifold
        mManifolds[similarManifoldIndex]->addContactPoint(contact);
        assert(mManifolds[similarManifoldIndex]->getNbContactPoints() > 0);

        return;
    }

    // If the maximum number of manifold has not been reached yet
    if (mNbManifolds < mNbMaxManifolds)
    {
        // Create a new manifold for the contact point
        createManifold(normalDirectionId);
        mManifolds[mNbManifolds-1]->addContactPoint(contact);
        for (int i=0; i<mNbManifolds; i++)
        {
            assert(mManifolds[i]->getNbContactPoints() > 0);
        }
        return;
    }

    // The contact point will be in a new contact manifold, we now have too much
    // manifolds condidates. We need to remove one. We choose to keep the manifolds
    // with the largest contact depth among their points
    int smallestDepthIndex = -1;
    scalar minDepth = contact->getPenetrationDepth();
    assert(mNbManifolds == mNbMaxManifolds);
    for (int i=0; i<mNbManifolds; i++)
    {
        scalar depth = mManifolds[i]->getLargestContactDepth();
        if (depth < minDepth)
        {
            minDepth = depth;
            smallestDepthIndex = i;
        }
    }

    // If we do not want to keep to new manifold (not created yet) with the
    // new contact point
    if (smallestDepthIndex == -1)
    {
    	// Delete the new contact
        //       contact->~ContactPoint();
    	//mMemoryAllocator.release(contact, sizeof(ContactPoint));

    	// Delete the new contact
        delete contact;
        return;
    }

    assert(smallestDepthIndex >= 0 && smallestDepthIndex < mNbManifolds);

    // Here we need to replace an existing manifold with a new one (that contains
    // the new contact point)
    removeManifold(smallestDepthIndex);
    createManifold(normalDirectionId);
    mManifolds[mNbManifolds-1]->addContactPoint(contact);
    assert(mManifolds[mNbManifolds-1]->getNbContactPoints() > 0);
    for (int i=0; i<mNbManifolds; i++)
    {
        assert(mManifolds[i]->getNbContactPoints() > 0);
    }

    return;
}

// Return the index of the contact manifold with a similar average normal.
// If no manifold has close enough average normal, it returns -1
int rpContactManifoldSet::selectManifoldWithSimilarNormal(short int normalDirectionId) const
{

    // Return the Id of the manifold with the same normal direction id (if exists)
    for (int i=0; i<mNbManifolds; i++)
    {
        if (normalDirectionId == mManifolds[i]->getNormalDirectionId())
        {
            return i;
        }
    }

    return -1;
}

// Map the normal vector into a cubemap face bucket (a face contains 4x4 buckets)
// Each face of the cube is divided into 4x4 buckets. This method maps the
// normal vector into of the of the bucket and returns a unique Id for the bucket
short int rpContactManifoldSet::computeCubemapNormalId(const Vector3& normal) const
{

    assert(normal.lengthSquare() > MACHINE_EPSILON);

    int faceNo;
    scalar u, v;
    scalar max = max3(Abs(normal.x), Abs(normal.y), Abs(normal.z));
    Vector3 normalScaled = normal / max;

    if (normalScaled.x >= normalScaled.y && normalScaled.x >= normalScaled.z)
    {
        faceNo = normalScaled.x > 0 ? 0 : 1;
        u = normalScaled.y;
        v = normalScaled.z;
    }
    else if (normalScaled.y >= normalScaled.x && normalScaled.y >= normalScaled.z)
    {
        faceNo = normalScaled.y > 0 ? 2 : 3;
        u = normalScaled.x;
        v = normalScaled.z;
    }
    else
    {
        faceNo = normalScaled.z > 0 ? 4 : 5;
        u = normalScaled.x;
        v = normalScaled.y;
    }

    int indexU = floor(((u + 1)/2) * CONTACT_CUBEMAP_FACE_NB_SUBDIVISIONS);
    int indexV = floor(((v + 1)/2) * CONTACT_CUBEMAP_FACE_NB_SUBDIVISIONS);
    if (indexU == CONTACT_CUBEMAP_FACE_NB_SUBDIVISIONS) indexU--;
    if (indexV == CONTACT_CUBEMAP_FACE_NB_SUBDIVISIONS) indexV--;

    const int nbSubDivInFace = CONTACT_CUBEMAP_FACE_NB_SUBDIVISIONS * CONTACT_CUBEMAP_FACE_NB_SUBDIVISIONS;
    return faceNo * 200 + indexU * nbSubDivInFace + indexV;
}

// Update the contact manifolds
void rpContactManifoldSet::update()
{

    for (int i=mNbManifolds-1; i>=0; i--)
    {

        // Update the contact manifold
        mManifolds[i]->update(mShape1->getBody()->getTransform() * mShape1->getLocalToBodyTransform(),
                              mShape2->getBody()->getTransform() * mShape2->getLocalToBodyTransform());

        // Remove the contact manifold if has no contact points anymore
        if (mManifolds[i]->getNbContactPoints() == 0)
        {
            removeManifold(i);
        }
    }
}





// Clear the contact manifold set
void rpContactManifoldSet::clear()
{
    // Destroy all the contact manifolds
    for (int i=mNbManifolds-1; i>=0; i--)
    {
        removeManifold(i);
    }

    assert(mNbManifolds == 0);
}

// Create a new contact manifold and add it to the set
void rpContactManifoldSet::createManifold(short int normalDirectionId)
{
    assert(mNbManifolds < mNbMaxManifolds);

    mManifolds[mNbManifolds] = new rpContactManifold(mShape1, mShape2 , normalDirectionId);
    mNbManifolds++;
}

// Remove a contact manifold from the set
void rpContactManifoldSet::removeManifold(int index)
{

    assert(mNbManifolds > 0);
    assert(index >= 0 && index < mNbManifolds);

    // Delete the new contact
    //mManifolds[index]->~ContactManifold();
    // mMemoryAllocator.release(mManifolds[index], sizeof(ContactManifold));

    // Delete the new contact
    delete mManifolds[index];

    for (int i=index; (i+1) < mNbManifolds; i++)
    {
        mManifolds[i] = mManifolds[i+1];
    }

    mNbManifolds--;
}



// Return the total number of contact points in the set of manifolds
SIMD_INLINE int rpContactManifoldSet::getTotalNbContactPoints() const
{
    int nbPoints = 0;
    for (int i=0; i<mNbManifolds; i++)
    {
        nbPoints += mManifolds[i]->getNbContactPoints();
    }
    return nbPoints;
}



} /* namespace real_physics */




