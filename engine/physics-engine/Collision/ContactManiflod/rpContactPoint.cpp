/*
 * rpContactPoint.cpp
 *
 *  Created on: 28 нояб. 2016 г.
 *      Author: wera
 */

#include "rpContactPoint.h"

namespace real_physics
{

// Constructor
rpContactPoint::rpContactPoint(rpContactPointInfo& contactInfo)
: mNormal(contactInfo.normal),
  mPenetrationDepth(contactInfo.penetrationDepth),
  mLocalPointOnBody1(contactInfo.localPoint1),
  mLocalPointOnBody2(contactInfo.localPoint2),
  mWorldPointOnBody1((contactInfo.localPoint1)),
  mWorldPointOnBody2( contactInfo.localPoint2),
  mIsRestingContact(false)
{

//	mFrictionVectors[0] = Vector3(0, 0, 0);
//	mFrictionVectors[1] = Vector3(0, 0, 0);

//    mLocalPointOnBody1 = Vector3::ZERO;
//    mLocalPointOnBody2 = Vector3::ZERO;

//    mWorldPointOnBody1 = Vector3::ZERO;
//    mWorldPointOnBody2 = Vector3::ZERO;

    //assert(mPenetrationDepth > 0.0);
}

// Destructor
rpContactPoint::~rpContactPoint()
{

}

} /* namespace real_physics */
