/*
 * Body.h
 *
 *  Created on: 25 нояб. 2016 г.
 *      Author: wera
 */

#ifndef SOURCE_ENGIE_COLLISION_BODY_RPBODY_H_
#define SOURCE_ENGIE_COLLISION_BODY_RPBODY_H_


// Libraries

#include "../../Memory/memory.h"



namespace real_physics
{



typedef long unsigned int luint;
typedef luint bodyindex;
typedef std::pair<bodyindex, bodyindex> bodyindexpair;



// TODO : Make this class abstract
// Class Body
/**
 * This class to represent a body of the physics engine. You should not
 * instantiante this class but instantiate the CollisionBody or RigidBody
 * classes instead.
 */
class rpBody
{

    protected :

        // -------------------- Attributes -------------------- //

        /// ID of the body
        bodyindex mID;

        /// True if the body has already been added in an island (for sleeping technique)
        bool mIsAlreadyInIsland;

        /// True if the body is allowed to go to sleep for better efficiency
        bool mIsAllowedToSleep;

        /// True if the body is active.
        /// An inactive body does not participate in collision detection,
        /// is not simulated and will not be hit in a ray casting query.
        /// A body is active by default. If you set this
        /// value to "false", all the proxy shapes of this body will be
        /// removed from the broad-phase. If you set this value to "true",
        /// all the proxy shapes will be added to the broad-phase. A joint
        /// connected to an inactive body will also be inactive.
        bool mIsActive;

        /// True if the body is sleeping (for sleeping technique)
        bool mIsSleeping;

        /// Elapsed time since the body velocity was bellow the sleep velocity
        scalar mSleepTime;

        /// Pointer that can be used to attach user data to the body
        void* mUserData;



        //-------------------- Constructor -------------------- //

        /// Private copy-constructor
        rpBody(const rpBody& body);

        /// Private assignment operator
        rpBody& operator=(const rpBody& body);

    public:

        //----------------------- Methods ----------------------//

        /// Constructor
        rpBody(bodyindex id);

        /// Destructor
        virtual ~rpBody();

        /// Return the ID of the body
        bodyindex getID() const;

        /// Return whether or not the body is allowed to sleep
        bool isAllowedToSleep() const;

        /// Set whether or not the body is allowed to go to sleep
        void setIsAllowedToSleep(bool isAllowedToSleep);

        /// Set the variable to know whether or not the body is sleeping
        virtual void setIsSleeping(bool isSleeping);

        /// Return whether or not the body is sleeping
        bool isSleeping() const;

        /// Return true if the body is active
        bool isActive() const;

        /// Set whether or not the body is active
        virtual void setIsActive(bool isActive);

        /// Return a pointer to the user data attached to this body
        void* getUserData() const;

        /// Attach user data to this body
        void setUserData(void* userData);

        /// Smaller than operator
        bool operator<(const rpBody& body2) const;

        /// Larger than operator
        bool operator>(const rpBody& body2) const;

        /// Equal operator
        bool operator==(const rpBody& body2) const;

        /// Not equal operator
        bool operator!=(const rpBody& body2) const;

        // -------------------- Friendship -------------------- //

        friend class DynamicsWorld;
};

// Return the id of the body
/**
 * @return The ID of the body
 */
SIMD_INLINE bodyindex rpBody::getID() const
{
    return mID;
}

// Return whether or not the rpBody is allowed to sleep
/**
 * @return True if the rpBody is allowed to sleep and false otherwise
 */
SIMD_INLINE bool rpBody::isAllowedToSleep() const
{
    return mIsAllowedToSleep;
}

// Set whether or not the rpBody is allowed to go to sleep
/**
 * @param isAllowedToSleep True if the rpBody is allowed to sleep
 */
SIMD_INLINE void rpBody::setIsAllowedToSleep(bool isAllowedToSleep)
{
    mIsAllowedToSleep = isAllowedToSleep;
    if (!mIsAllowedToSleep) setIsSleeping(false);
}

// Return whether or not the rpBody is sleeping
/**
 * @return True if the rpBody is currently sleeping and false otherwise
 */
SIMD_INLINE bool rpBody::isSleeping() const
{
    return mIsSleeping;
}

// Return true if the rpBody is active
/**
 * @return True if the rpBody currently active and false otherwise
 */
SIMD_INLINE bool rpBody::isActive() const
{
    return mIsActive;
}

// Set whether or not the rpBody is active
/**
 * @param isActive True if you want to activate the rpBody
 */
SIMD_INLINE void rpBody::setIsActive(bool isActive)
{
    mIsActive = isActive;
}

// Set the variable to know whether or not the rpBody is sleeping
SIMD_INLINE void rpBody::setIsSleeping(bool isSleeping)
{
    if (isSleeping)
    {
        mSleepTime = scalar(0.0);
    }
    else
    {
        if (mIsSleeping)
        {
            mSleepTime = scalar(0.0);
        }
    }

    mIsSleeping = isSleeping;
}

// Return a pointer to the user data attached to this rpBody
/**
 * @return A pointer to the user data you have attached to the rpBody
 */
SIMD_INLINE void* rpBody::getUserData() const
{
    return mUserData;
}

// Attach user data to this rpBody
/**
 * @param userData A pointer to the user data you want to attach to the rpBody
 */
SIMD_INLINE void rpBody::setUserData(void* userData)
{
    mUserData = userData;
}

// Smaller than operator
SIMD_INLINE bool rpBody::operator<(const rpBody& body2) const
{
    return (mID < body2.mID);
}

// Larger than operator
SIMD_INLINE bool rpBody::operator>(const rpBody& body2) const
{
    return (mID > body2.mID);
}

// Equal operator
SIMD_INLINE bool rpBody::operator==(const rpBody& body2) const
{
    return (mID == body2.mID);
}

// Not equal operator
SIMD_INLINE bool rpBody::operator!=(const rpBody& body2) const
{
    return (mID != body2.mID);
}


} /* namespace real_physics */

#endif /* SOURCE_ENGIE_COLLISION_BODY_RPBODY_H_ */
