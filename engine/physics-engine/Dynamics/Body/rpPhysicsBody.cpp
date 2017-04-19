/*
 * rpRigidPhysicsBody.cpp
 *
 *  Created on: 14 дек. 2016 г.
 *      Author: wera
 */

#include "../../Dynamics/Body/rpPhysicsBody.h"

namespace real_physics
{


rpPhysicsBody::rpPhysicsBody(const Transform& transform, rpCollisionDetection* CollideWorld, bodyindex id)
:rpPhysicsObject(transform, CollideWorld, id)
{

}

rpPhysicsBody::~rpPhysicsBody()
{
	// Remove all the proxy collision shapes of the body
	removeAllCollisionShapes();
}


} /* namespace real_physics */

