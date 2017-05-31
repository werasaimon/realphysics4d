/*
 * rpRigidPhysicsBody.cpp
 *
 *  Created on: 14 дек. 2016 г.
 *      Author: wera
 */

#include "../../Dynamics/Body/rpPhysicsBody.h"
#include "../../Dynamics/Joint/rpJoint.h"

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

void rpPhysicsBody::removeJointFromJointsList(const rpJoint *joint)
{

    assert(joint != NULL);
    assert(mJointsList != NULL);

    // Remove the joint from the linked list of the joints of the first body
    if (mJointsList->joint == joint)
    {   // If the first element is the one to remove
         rpJointListElement* elementToRemove = mJointsList;
         mJointsList = elementToRemove->next;
         delete elementToRemove;
    }
    else
    {  // If the element to remove is not the first one in the list
        rpJointListElement* currentElement = mJointsList;
        while (currentElement->next != NULL)
        {
            if (currentElement->next->joint == joint)
            {
                rpJointListElement* elementToRemove = currentElement->next;
                currentElement->next = elementToRemove->next;
                delete elementToRemove;
                break;
            }
            currentElement = currentElement->next;
        }
    }

}


} /* namespace real_physics */

