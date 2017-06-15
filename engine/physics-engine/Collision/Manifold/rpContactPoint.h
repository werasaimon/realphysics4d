/*
 * rpContactPoint.h
 *
 *  Created on: 28 нояб. 2016 г.
 *      Author: wera
 */

#ifndef SOURCE_ENGIE_COLLISION_CONTACTMANIFLOD_RPCONTACTPOINT_H_
#define SOURCE_ENGIE_COLLISION_CONTACTMANIFLOD_RPCONTACTPOINT_H_

#include "../../LinearMaths/mathematics.h"

namespace real_physics
{


// Structure ContactPointInfo
/**
 * This structure contains informations about a collision contact
 * computed during the narrow-phase collision detection. Those
 * informations are used to compute the contact set for a contact
 * between two bodies.
 */
struct rpContactPointInfo
{

    private:

        // -------------------- Methods -------------------- //

    public:

        // -------------------- Attributes -------------------- //


        /// Normalized normal vector of the collision contact in world space
        Vector3 normal;

        /// Penetration depth of the contact
        scalar penetrationDepth;

        /// Contact point of body 1 in local space of body 1
        Vector3 localPoint1;

        /// Contact point of body 2 in local space of body 2
        Vector3 localPoint2;

        // -------------------- Methods -------------------- //

        rpContactPointInfo(void){}

        /// Constructor
        rpContactPointInfo(const Vector3& normal, scalar penetrationDepth,
                         const Vector3& localPoint1, const Vector3& localPoint2)
            : normal(normal),
			  penetrationDepth(penetrationDepth),
              localPoint1(localPoint1),
              localPoint2(localPoint2)
        {

        }
};





// Class ContactPoint
/**
 * This class represents a collision contact point between two
 * bodies in the physics engine.
 */
class rpContactPoint
{

    private :

        // -------------------- Attributes -------------------- //


        /// Normalized normal vector of the contact (from body1 toward body2) in world space
        Vector3 mNormal;

        /// Penetration depth
        scalar mPenetrationDepth;

        /// Contact point on body 1 in local space of body 1
        Vector3 mLocalPointOnBody1;

        /// Contact point on body 2 in local space of body 2
        Vector3 mLocalPointOnBody2;

        /// Contact point on body 1 in world space
        Vector3 mWorldPointOnBody1;

        /// Contact point on body 2 in world space
        Vector3 mWorldPointOnBody2;

        /// True if the contact is a resting contact (exists for more than one time step)
        bool mIsRestingContact;

        /// Two orthogonal vectors that span the tangential friction plane
        Vector3 mFrictionVectors[2];

        /// Cached penetration impulse
        scalar mPenetrationImpulse;

        /// Cached first friction impulse
        scalar mFrictionImpulse1;

        /// Cached second friction impulse
        scalar mFrictionImpulse2;

        /// Cached rolling resistance impulse
        Vector3 mRollingResistanceImpulse;

        // -------------------- Methods -------------------- //

        // Private copy-constructor
        rpContactPoint(const rpContactPoint& contact);

        // Private assignment operator
        rpContactPoint& operator=(const rpContactPoint& contact);

    public :

        // -------------------- Methods -------------------- //

       // rpContactPoint(){}
        /// Constructor
         rpContactPoint(rpContactPointInfo& contactInfo);

        /// Destructor
        ~rpContactPoint();



        /// Return the normal vector of the contact
        Vector3 getNormal() const;

        /// Set the penetration depth of the contact
        void setPenetrationDepth(scalar penetrationDepth);

        /// Return the contact local point on body 1
        Vector3 getLocalPointOnBody1() const;

        /// Return the contact local point on body 2
        Vector3 getLocalPointOnBody2() const;

        /// Return the contact world point on body 1
        Vector3 getWorldPointOnBody1() const;

        /// Return the contact world point on body 2
        Vector3 getWorldPointOnBody2() const;

        /// Return the cached penetration impulse
        scalar getPenetrationImpulse() const;

        /// Return the cached first friction impulse
        scalar getFrictionImpulse1() const;

        /// Return the cached second friction impulse
        scalar getFrictionImpulse2() const;

        /// Return the cached rolling resistance impulse
        Vector3 getRollingResistanceImpulse() const;

        /// Set the cached penetration impulse
        void setPenetrationImpulse(scalar impulse);

        /// Set the first cached friction impulse
        void setFrictionImpulse1(scalar impulse);

        /// Set the second cached friction impulse
        void setFrictionImpulse2(scalar impulse);

        /// Set the cached rolling resistance impulse
        void setRollingResistanceImpulse(const Vector3& impulse);

        /// Set the contact world point on body 1
        void setWorldPointOnBody1(const Vector3& worldPoint);

        /// Set the contact world point on body 2
        void setWorldPointOnBody2(const Vector3& worldPoint);

        /// Return true if the contact is a resting contact
        bool getIsRestingContact() const;

        /// Set the mIsRestingContact variable
        void setIsRestingContact(bool isRestingContact);

        /// Get the first friction vector
        Vector3 getFrictionVector1() const;

        /// Set the first friction vector
        void setFrictionVector1(const Vector3& frictionVector1);

        /// Get the second friction vector
        Vector3 getFrictionVector2() const;

        /// Set the second friction vector
        void setFrictionVector2(const Vector3& frictionVector2);

        /// Return the penetration depth
        scalar getPenetrationDepth() const;

        /// Return the number of bytes used by the contact point
        size_t getSizeInBytes() const;


        //-------------------- Friendships --------------------//

        friend class rpSequentialImpulseObjectSolver;


};



// Return the normal vector of the contact
SIMD_INLINE Vector3 rpContactPoint::getNormal() const
{
    return mNormal;
}

// Set the penetration depth of the contact
SIMD_INLINE void rpContactPoint::setPenetrationDepth(scalar penetrationDepth)
{
    this->mPenetrationDepth = penetrationDepth;
}

// Return the contact point on body 1
SIMD_INLINE Vector3 rpContactPoint::getLocalPointOnBody1() const
{
    return mLocalPointOnBody1;
}

// Return the contact point on body 2
SIMD_INLINE Vector3 rpContactPoint::getLocalPointOnBody2() const
{
    return mLocalPointOnBody2;
}

// Return the contact world point on body 1
SIMD_INLINE Vector3 rpContactPoint::getWorldPointOnBody1() const
{
    return mWorldPointOnBody1;
}

// Return the contact world point on body 2
SIMD_INLINE Vector3 rpContactPoint::getWorldPointOnBody2() const
{
    return mWorldPointOnBody2;
}

// Return the cached penetration impulse
SIMD_INLINE scalar rpContactPoint::getPenetrationImpulse() const
{
    return mPenetrationImpulse;
}

// Return the cached first friction impulse
SIMD_INLINE scalar rpContactPoint::getFrictionImpulse1() const
{
    return mFrictionImpulse1;
}

// Return the cached second friction impulse
SIMD_INLINE scalar rpContactPoint::getFrictionImpulse2() const
{
    return mFrictionImpulse2;
}

// Return the cached rolling resistance impulse
SIMD_INLINE Vector3 rpContactPoint::getRollingResistanceImpulse() const
{
    return mRollingResistanceImpulse;
}

// Set the cached penetration impulse
SIMD_INLINE void rpContactPoint::setPenetrationImpulse(scalar impulse)
{
    mPenetrationImpulse = impulse;
}

// Set the first cached friction impulse
SIMD_INLINE void rpContactPoint::setFrictionImpulse1(scalar impulse)
{
    mFrictionImpulse1 = impulse;
}

// Set the second cached friction impulse
SIMD_INLINE void rpContactPoint::setFrictionImpulse2(scalar impulse)
{
    mFrictionImpulse2 = impulse;
}

// Set the cached rolling resistance impulse
SIMD_INLINE void rpContactPoint::setRollingResistanceImpulse(const Vector3& impulse)
{
    mRollingResistanceImpulse = impulse;
}

// Set the contact world point on body 1
SIMD_INLINE void rpContactPoint::setWorldPointOnBody1(const Vector3& worldPoint)
{
    mWorldPointOnBody1 = worldPoint;
}

// Set the contact world point on body 2
SIMD_INLINE void rpContactPoint::setWorldPointOnBody2(const Vector3& worldPoint)
{
    mWorldPointOnBody2 = worldPoint;
}

// Return true if the contact is a resting contact
SIMD_INLINE bool rpContactPoint::getIsRestingContact() const
{
    return mIsRestingContact;
}

// Set the mIsRestingContact variable
SIMD_INLINE void rpContactPoint::setIsRestingContact(bool isRestingContact)
{
    mIsRestingContact = isRestingContact;
}

// Get the first friction vector
SIMD_INLINE Vector3 rpContactPoint::getFrictionVector1() const
{
    return mFrictionVectors[0];
}

// Set the first friction vector
SIMD_INLINE void rpContactPoint::setFrictionVector1(const Vector3& frictionVector1)
{
    mFrictionVectors[0] = frictionVector1;
}

// Get the second friction vector
SIMD_INLINE Vector3 rpContactPoint::getFrictionVector2() const
{
    return mFrictionVectors[1];
}

// Set the second friction vector
SIMD_INLINE void rpContactPoint::setFrictionVector2(const Vector3& frictionVector2)
{
    mFrictionVectors[1] = frictionVector2;
}

// Return the penetration depth of the contact
SIMD_INLINE scalar rpContactPoint::getPenetrationDepth() const
{
    return mPenetrationDepth;
}

// Return the number of bytes used by the contact point
SIMD_INLINE size_t rpContactPoint::getSizeInBytes() const
{
    return sizeof(rpContactPoint);
}


} /* namespace real_physics */

#endif /* SOURCE_ENGIE_COLLISION_CONTACTMANIFLOD_RPCONTACTPOINT_H_ */
