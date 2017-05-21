/*
 * UltimatePhysics.cpp
 *
 *  Created on: 6 мар. 2017 г.
 *      Author: wera
 */

#include "../element-engine/UltimatePhysics.h"


UltimatePhysics::UltimatePhysics(real_physics::rpCollisionBody* _physBody)
: mPhysicsDriver(_physBody)
{
	assert(mPhysicsDriver);
}


UltimatePhysics::~UltimatePhysics() {
	// TODO Auto-generated destructor stub
}

