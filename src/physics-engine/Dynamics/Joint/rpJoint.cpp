/*
 * rpJoint.cpp
 *
 *  Created on: 8 янв. 2017 г.
 *      Author: wera
 */

#include "rpJoint.h"

namespace real_physics
{


// Constructor
rpJoint::rpJoint(const rpJointInfo& jointInfo)
    :mBody1(jointInfo.body1), mBody2(jointInfo.body2),
			mType(jointInfo.type),
            mPositionCorrectionTechnique(jointInfo.positionCorrectionTechnique),
            mIsCollisionEnabled(jointInfo.isCollisionEnabled),
			mIsAlreadyInIsland(false)
{

    assert(mBody1 != NULL);
    assert(mBody2 != NULL);
}

// Destructor
rpJoint::~rpJoint()
{

}


/// set stoffnes constraint-joint
void rpJoint::setSoftness(const scalar &value)
{
    softness = value;
}





} /* namespace real_physics */
