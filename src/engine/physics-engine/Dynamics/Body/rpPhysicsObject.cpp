/*
 * rprpPhysicsObject.cpp
 *
 *  Created on: 14 дек. 2016 г.
 *      Author: wera
 */

#include "rpPhysicsObject.h"

#include <stddef.h>
#include <cassert>

#include "../../Collision/rpCollisionDetection.h"
#include "../../Collision/rpProxyShape.h"
#include "../../Collision/Shapes/rpAABB.h"
#include "../../Collision/Shapes/rpCollisionShape.h"

namespace real_physics
{


rpPhysicsObject::rpPhysicsObject(const Transform& transform, rpCollisionDetection* CollideWorld, bodyindex id)
:rpCollisionBody(transform, CollideWorld, id)
{

}

rpPhysicsObject::~rpPhysicsObject()
{
	// Remove all the proxy collision shapes of the body
	removeAllCollisionShapes();

}



rpProxyShape* rpPhysicsObject::addCollisionShape(rpCollisionShape* collisionShape , scalar mass , const Transform& transform )
{

    assert(mass > scalar(0.0));

    // Create a new proxy collision shape to attach the collision shape to the body
    rpProxyShape* proxyShape = new rpProxyShape(this, collisionShape, transform , mass);

    // Add it to the list of proxy collision shapes of the body
    if (mProxyCollisionShapes == NULL)
    {
        mProxyCollisionShapes = proxyShape;
    }
    else
    {
        proxyShape->mNext = mProxyCollisionShapes;
        mProxyCollisionShapes = proxyShape;
    }

    // Compute the world-space AABB of the new collision shape
    rpAABB aabb;
    collisionShape->computeAABB(aabb, mTransform , transform);

    // Notify the collision detection about this new collision shape
    mCollisionDetection->addProxyCollisionShape(proxyShape, aabb);

    mNbCollisionShapes++;

    // Recompute the center of mass, total mass and inertia tensor of the body with the new
    // collision shape
    recomputeMassInformation();

    // Return a pointer to the proxy collision shape
    return proxyShape;
}



SIMD_INLINE void rpPhysicsObject::removeCollisionShape(const rpProxyShape* proxyShape)
{
	   // Remove the collision shape
        removeCollisionShapee(proxyShape);

	    // Recompute the total mass, center of mass and inertia tensor
	    recomputeMassInformation();
}


SIMD_INLINE void rpPhysicsObject::updateBroadPhaseState() const
{

	    //DynamicsWorld& world = static_cast<DynamicsWorld&>(mWorld);
	 	const Vector3 displacement =  Vector3::ZERO;//world.mTimeStep * mLinearVelocity;

	    // For all the proxy collision shapes of the body
	    for (rpProxyShape* shape = mProxyCollisionShapes; shape != NULL; shape = shape->mNext)
	    {
	        // Recompute the world-space AABB of the collision shape
	        rpAABB aabb;
	        shape->getCollisionShape()->computeAABB(aabb, mTransform , shape->getLocalToBodyTransform() , mRelativityMotion.getLorentzMatrix());


	        // Update the broad-phase state for the proxy collision shape
	        mCollisionDetection->updateProxyCollisionShape(shape, aabb, displacement);
	    }

}



} /* namespace real_physics */


