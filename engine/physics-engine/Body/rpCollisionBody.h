/*
 * CollisionBody.h
 *
 *  Created on: 25 нояб. 2016 г.
 *      Author: wera
 */

#ifndef SOURCE_ENGIE_COLLISION_BODY_RPCOLLISIONBODY_H_
#define SOURCE_ENGIE_COLLISION_BODY_RPCOLLISIONBODY_H_

#include "../LinearMaths/mathematics.h"
#include "../LinearMaths/rpQuaternion.h"
#include "../Memory/memory.h"
#include "rpBody.h"



namespace real_physics
{
	struct RaycastInfo;
	class  rpAABB;
	class  rpCollisionShape;
} /* namespace real_physics */

namespace real_physics
{

// Class declarations
class   rpProxyShape;
class   rpCollisionWorld;
class   rpCollisionManager;
class   rpContactManifold;

typedef rpListElement<rpContactManifold> ContactManifoldListElement;

/// Enumeration for the type of a body
/// STATIC : A static body has infinite mass, zero velocity but the position can be
///          changed manually. A static body does not collide with other static or kinematic bodies.
/// KINEMATIC : A kinematic body has infinite mass, the velocity can be changed manually and its
///             position is computed by the physics engine. A kinematic body does not collide with
///             other static or kinematic bodies.
/// DYNAMIC : A dynamic body has non-zero mass, non-zero velocity determined by forces and its
///           position is determined by the physics engine. A dynamic body can collide with other
///           dynamic, static or kinematic bodies.
enum BodyType {STATIC, KINEMATIC, DYNAMIC};

// Class CollisionBody
/**
 * This class represents a body that is able to collide with others
 * bodies. This class inherits from the Body class.
 */
class rpCollisionBody : public rpBody
{

    protected:


        //-------------------- Attributes --------------------//

        /// Type of body (static, kinematic or dynamic)
        BodyType mType;


        /// World relativity body
        LorentzContraction      mRelativityMotion;

        /// Position and orientation of the body
        Transform               mTransform;

        /// First element of the linked list of proxy collision shapes of this body
        rpProxyShape*           mProxyCollisionShapes;

        /// Number of collision shapes
        uint                    mNbCollisionShapes;

        /// Collision detection object
        rpCollisionManager     *mCollisionDetection;


        /// First element of the linked list of contact manifolds involving this body
        ContactManifoldListElement* mContactManifoldsList = NULL;


        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        rpCollisionBody(const rpCollisionBody& body);

        /// Private assignment operator
        rpCollisionBody& operator=(const rpCollisionBody& body);

        /// Reset the contact manifold lists
        void resetContactManifoldsList();

        /// Remove all the collision shapes
        void removeAllCollisionShapes();

        /// Update the broad-phase state for this body (because it has moved for instance)
        virtual void updateBroadPhaseState() const;


        /// Update the broad-phase state of a proxy collision shape of the body
        void updateProxyShapeInBroadPhase(rpProxyShape* proxyShape, const Vector3& displacement , bool forceReinsert = false) const;


        /// Ask the broad-phase to test again the collision shapes of the body for collision
        /// (as if the body has moved).
        void askForBroadPhaseCollisionCheck() const;

        /// Reset the mIsAlreadyInIsland variable of the body and contact manifolds
        int resetIsAlreadyInIslandAndCountManifolds();

    public:


        /// Tensor covariant component velocity metrices
        void updateDisplacementRelativityVelocity( const Vector3& Velocity )
        {
            mRelativityMotion.updateDisplacementBoost( Velocity );
        }


        // -------------------- Methods -------------------- //

        /// Constructor
        rpCollisionBody(const Transform& transform, real_physics::rpCollisionManager *CollideWorld, bodyindex id );

        /// Destructor
        virtual ~rpCollisionBody();

        /// Return the type of the body
        BodyType getType() const;

        /// Set the type of the body
        virtual void setType(BodyType type);

        /// Set whether or not the body is active
        virtual void setIsActive(bool isActive);

        /// Return the current position and orientation
        const Transform& getTransform() const;

        /// Set the current position and orientation
        virtual void setTransform(const Transform& transform);

        /// Add a collision shape to the body.
        virtual rpProxyShape* addCollisionShape(rpCollisionShape* collisionShape, scalar massa ,
                                                 const Transform& transform = Transform::identity());

        /// Remove a collision shape from the body
        virtual void removeCollisionShapee(const rpProxyShape* proxyShape);

        /// Return the first element of the linked list of contact manifolds involving this body
        const ContactManifoldListElement* getContactManifoldsList() const;

        /// Return true if a point is inside the collision body
        bool testPointInside(const Vector3& worldPoint) const;

        /// Raycast method with feedback information
        bool raycast(const Ray& ray, RaycastInfo& raycastInfo);

        /// Compute and return the AABB of the body by merging all proxy shapes AABBs
        rpAABB getAABB() const;

        /// Return the linked list of proxy shapes of that body
        rpProxyShape* getProxyShapesList();

        /// Return the linked list of proxy shapes of that body
        const rpProxyShape* getProxyShapesList() const;

        /// Return the world-space coordinates of a point given the local-space coordinates of the body
        Vector3 getWorldPoint(const Vector3& localPoint) const;

        /// Return the world-space vector of a vector given in local-space coordinates of the body
        Vector3 getWorldVector(const Vector3& localVector) const;

        /// Return the body local-space coordinates of a point given in the world-space coordinates
        Vector3 getLocalPoint(const Vector3& worldPoint) const;

        /// Return the body local-space coordinates of a vector given in the world-space coordinates
        Vector3 getLocalVector(const Vector3& worldVector) const;

        /// Return the relativity body world space
		LorentzContraction getRelativityMotion() const;


        //-------------------- Friendship --------------------//
        friend class rpCollisionWorld;
        friend class rpDynamicsWorld;
        friend class rpCollisionManager;
        friend class rpBroadPhaseAlgorithm;
        friend class rpConvexMeshShape;
        friend class rpProxyShape;
        friend class rpPhysicsBody;
        friend class rpRigidPhysicsBody;
};

// Return the type of the body
/**
 * @return the type of the body (STATIC, KINEMATIC, DYNAMIC)
 */
SIMD_INLINE BodyType rpCollisionBody::getType() const
{
    return mType;
}

// Set the type of the body
/// The type of the body can either STATIC, KINEMATIC or DYNAMIC as described bellow:
/// STATIC : A static body has infinite mass, zero velocity but the position can be
///          changed manually. A static body does not collide with other static or kinematic bodies.
/// KINEMATIC : A kinematic body has infinite mass, the velocity can be changed manually and its
///             position is computed by the physics engine. A kinematic body does not collide with
///             other static or kinematic bodies.
/// DYNAMIC : A dynamic body has non-zero mass, non-zero velocity determined by forces and its
///           position is determined by the physics engine. A dynamic body can collide with other
///           dynamic, static or kinematic bodies.
/**
 * @param type The type of the body (STATIC, KINEMATIC, DYNAMIC)
 */
SIMD_INLINE void rpCollisionBody::setType(BodyType type)
{
    mType = type;

    if (mType == STATIC)
    {

        // Update the broad-phase state of the body
        updateBroadPhaseState();
    }
}

// Return the current position and orientation
/**
 * @return The current transformation of the body that transforms the local-space
 *         of the body into world-space
 */
SIMD_INLINE const Transform& rpCollisionBody::getTransform() const
{
    return mTransform;
}

// Set the current position and orientation
/**
 * @param transform The transformation of the body that transforms the local-space
 *                  of the body into world-space
 */
SIMD_INLINE void rpCollisionBody::setTransform(const Transform& transform)
{
    // Update the transform of the body
	mTransform = transform;

    // Update the broad-phase state of the body
    updateBroadPhaseState();
}

// Return the first element of the linked list of contact manifolds involving this body
/**
 * @return A pointer to the first element of the linked-list with the contact
 *         manifolds of this body
 */
SIMD_INLINE const ContactManifoldListElement* rpCollisionBody::getContactManifoldsList() const
{
    return mContactManifoldsList;
}

// Return the linked list of proxy shapes of that body
/**
* @return The pointer of the first proxy shape of the linked-list of all the
*         proxy shapes of the body
*/
SIMD_INLINE rpProxyShape* rpCollisionBody::getProxyShapesList()
{
    return mProxyCollisionShapes;
}

// Return the linked list of proxy shapes of that body
/**
* @return The pointer of the first proxy shape of the linked-list of all the
*         proxy shapes of the body
*/
SIMD_INLINE const rpProxyShape* rpCollisionBody::getProxyShapesList() const
{
    return mProxyCollisionShapes;
}

// Return the world-space coordinates of a point given the local-space coordinates of the body
/**
* @param localPoint A point in the local-space coordinates of the body
* @return The point in world-space coordinates
*/
SIMD_INLINE Vector3 rpCollisionBody::getWorldPoint(const Vector3& localPoint) const
{
    return mTransform * localPoint;
}

// Return the world-space vector of a vector given in local-space coordinates of the body
/**
* @param localVector A vector in the local-space coordinates of the body
* @return The vector in world-space coordinates
*/
SIMD_INLINE Vector3 rpCollisionBody::getWorldVector(const Vector3& localVector) const
{
    return mTransform.getOrientation() * localVector;
}

// Return the body local-space coordinates of a point given in the world-space coordinates
/**
* @param worldPoint A point in world-space coordinates
* @return The point in the local-space coordinates of the body
*/
SIMD_INLINE Vector3 rpCollisionBody::getLocalPoint(const Vector3& worldPoint) const
{
    return mTransform.getInverse() * worldPoint;
}



// Return the body local-space coordinates of a vector given in the world-space coordinates
/**
* @param worldVector A vector in world-space coordinates
* @return The vector in the local-space coordinates of the body
*/
SIMD_INLINE Vector3 rpCollisionBody::getLocalVector(const Vector3& worldVector) const
{
    return mTransform.getOrientation().getInverse() * worldVector;
}


// Return the type of the body
/**
 * @return the type of the loretz boost matrix
 */
SIMD_INLINE  LorentzContraction rpCollisionBody::getRelativityMotion() const
{
	return mRelativityMotion;
}


} /* namespace real_physics */

#endif /* SOURCE_ENGIE_COLLISION_BODY_RPCOLLISIONBODY_H_ */
