/*
 * rpPhysicsMaterial.cpp
 *
 *  Created on: 15 дек. 2016 г.
 *      Author: wera
 */

#include "../Material/rpPhysicsMaterial.h"
#include "../../config.h"


namespace real_physics
{

// Constructor
rpPhysicsMaterial::rpPhysicsMaterial()
         : mFrictionCoefficient(DEFAULT_FRICTION_COEFFICIENT),
           mRollingResistance(DEFAULT_ROLLING_RESISTANCE),
           mBounciness(DEFAULT_BOUNCINESS)
{

}

// Copy-constructor
rpPhysicsMaterial::rpPhysicsMaterial(const rpPhysicsMaterial& material)
         : mFrictionCoefficient(material.mFrictionCoefficient),
           mRollingResistance(material.mRollingResistance),
		   mBounciness(material.mBounciness)
{

}

// Destructor
rpPhysicsMaterial::~rpPhysicsMaterial()
{

}


} /* namespace real_physics */
