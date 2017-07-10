/*
 * rpContactManifoldSet.h
 *
 *  Created on: 28 нояб. 2016 г.
 *      Author: wera
 */

#ifndef SOURCE_ENGIE_COLLISION_CONTACTMANIFLOD_RPCONTACTMANIFOLDSET_H_
#define SOURCE_ENGIE_COLLISION_CONTACTMANIFLOD_RPCONTACTMANIFOLDSET_H_

#include "rpContactManifold.h"

namespace real_physics
{


class rpContactManifold;

// Constants
const int MAX_MANIFOLDS_IN_CONTACT_MANIFOLD_SET = 3;   // Maximum number of contact manifolds in the set
const int CONTACT_CUBEMAP_FACE_NB_SUBDIVISIONS = 3;    // N Number for the N x N subdivisions of the cubemap

// Class ContactManifoldSet
/**
 * This class represents a set of one or several contact manifolds. Typically a
 * convex/convex collision will have a set with a single manifold and a convex-concave
 * collision can have more than one manifolds. Note that a contact manifold can
 * contains several contact points.
 */
class rpContactManifoldSet
{

    private:




        // -------------------- Attributes -------------------- //

        /// Maximum number of contact manifolds in the set
        int mNbMaxManifolds;

        /// Current number of contact manifolds in the set
        int mNbManifolds;

        /// Pointer to the first proxy shape of the contact
        rpProxyShape* mShape1;

        /// Pointer to the second proxy shape of the contact
        rpProxyShape* mShape2;

        /// Reference to the memory allocator
       // MemoryAllocator& mMemoryAllocator;

        /// Contact manifolds of the set
        rpContactManifold* mManifolds[MAX_MANIFOLDS_IN_CONTACT_MANIFOLD_SET];

        // -------------------- Methods -------------------- //

        /// Create a new contact manifold and add it to the set
        void createManifold(short normalDirectionId);

        /// Remove a contact manifold from the set
        void removeManifold(int index);

        // Return the index of the contact manifold with a similar average normal.
        int selectManifoldWithSimilarNormal(short int normalDirectionId) const;

        // Map the normal vector into a cubemap face bucket (a face contains 4x4 buckets)
        // Each face of the cube is divided into 4x4 buckets. This method maps the
        // normal vector into of the of the bucket and returns a unique Id for the bucket
        short int computeCubemapNormalId(const Vector3& normal) const;

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        rpContactManifoldSet(rpProxyShape* shape1, rpProxyShape* shape2,
                           int nbMaxManifolds);

        /// Destructor
        ~rpContactManifoldSet();


//        void setPairIndex( uint index1 , uint index2 )
//        {
//        	for (int i = 0; i < mNbManifolds; ++i)
//        	{
//			    mManifolds[i]->mIdIndex1 = index1;
//			    mManifolds[i]->mIdIndex2 = index2;
//			}
//        }


        void setExtermalPenetration( scalar penetration )
        {
            for (int i = 0; i < mNbManifolds; ++i)
            {
                mManifolds[i]->mExtremalPenetration = penetration;
            }
        }


        std::vector<rpContactPoint*> getAll() const
        {
              std::vector<rpContactPoint*> contacts;
              return contacts;
        }

        void setAllContacts( std::vector<rpContactPoint*> &mContacts );


        /// Return the first proxy shape
        rpProxyShape* getShape1() const;

        /// Return the second proxy shape
        rpProxyShape* getShape2() const;

        /// Add a contact point to the manifold set
        void addContactPoint(rpContactPoint* contact);

        /// Update the contact manifolds
        void update();

        /// Clear the contact manifold set
        void clear();

        /// Return the number of manifolds in the set
        int getNbContactManifolds() const;

        /// Return a given contact manifold
        rpContactManifold* getContactManifold(int index) const;

        /// Return the total number of contact points in the set of manifolds
        int getTotalNbContactPoints() const;

        // -------------------- Friendships -------------------- //

        friend class rpCollisionManager;
};



// Return the first proxy shape
SIMD_INLINE rpProxyShape* rpContactManifoldSet::getShape1() const
{
    return mShape1;
}

// Return the second proxy shape
SIMD_INLINE rpProxyShape* rpContactManifoldSet::getShape2() const
{
    return mShape2;
}

// Return the number of manifolds in the set
SIMD_INLINE int rpContactManifoldSet::getNbContactManifolds() const
{
    return mNbManifolds;
}


// Return a given contact manifold
SIMD_INLINE rpContactManifold* rpContactManifoldSet::getContactManifold(int index) const
{
    assert(index >= 0 && index < mNbManifolds);
    return mManifolds[index];
}



} /* namespace real_physics */

#endif /* SOURCE_ENGIE_COLLISION_CONTACTMANIFLOD_RPCONTACTMANIFOLDSET_H_ */
