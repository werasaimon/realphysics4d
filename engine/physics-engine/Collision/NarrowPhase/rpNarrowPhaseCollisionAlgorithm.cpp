/*
 * rpNarrowPhaseAlgorithm.cpp
 *
 *  Created on: 24 нояб. 2016 г.
 *      Author: wera
 */

#include "rpNarrowPhaseCollisionAlgorithm.h"

#include <stddef.h>



namespace real_physics
{



// Constructor
NarrowPhaseCollisionAlgorithm::NarrowPhaseCollisionAlgorithm()
: mCurrentOverlappingPair(NULL)
{

}

// Destructor
NarrowPhaseCollisionAlgorithm::~NarrowPhaseCollisionAlgorithm()
{

}

//// Initalize the algorithm
//void NarrowPhaseAlgorithm::init(CollisionDetection* collisionDetection, MemoryAllocator* memoryAllocator) {
//    mCollisionDetection = collisionDetection;
//    mMemoryAllocator = memoryAllocator;
//}

} /* namespace real_physics */
