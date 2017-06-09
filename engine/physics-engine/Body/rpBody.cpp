/*
 * Body.cpp
 *
 *  Created on: 25 нояб. 2016 г.
 *      Author: wera
 */

#include "rpBody.h"

namespace real_physics
{



// Constructor
/**
 * @param id ID of the new body
 */
rpBody::rpBody(bodyindex id)
     : mID(id),
	   mIsAlreadyInIsland(false), mIsAllowedToSleep(true), mIsActive(true),
       mIsSleeping(false), mSleepTime(0),
	   mUserData(NULL)
{

}

// Destructor
rpBody::~rpBody()
{

}



} /* namespace real_physics */
