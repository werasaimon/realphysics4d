/*
 * rpRigidPhysicsBody.cpp
 *
 *  Created on: 14 дек. 2016 г.
 *      Author: wera
 */

#include "rpPhysicsBody.h"
#include "../Dynamics/Joint/rpJoint.h"

namespace real_physics
{


rpPhysicsBody::rpPhysicsBody(const Transform& transform, rpCollisionManager *CollideWorld, bodyindex id)
:rpPhysicsObject(transform, CollideWorld, id) ,  mJointsList(NULL)
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
    if (mJointsList->getPointer() == joint)
    {   // If the first element is the one to remove
         JointListElement* elementToRemove = mJointsList;
         mJointsList = elementToRemove->getNext();
         delete elementToRemove;
    }
    else
    {  // If the element to remove is not the first one in the list
        JointListElement* currentElement = mJointsList;
        while (currentElement->getNext() != NULL)
        {
            if (currentElement->getNext()->getPointer() == joint)
            {
                JointListElement* elementToRemove = currentElement->getNext();
                currentElement/*->next */ = elementToRemove->getNext();
                delete elementToRemove;
                break;
            }
            currentElement = currentElement->getNext();
        }
    }

}


} /* namespace real_physics */

