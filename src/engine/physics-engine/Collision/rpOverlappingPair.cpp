/*
 * rpOverlappingPair.cpp
 *
 *  Created on: 23 нояб. 2016 г.
 *      Author: wera
 */

#include "rpOverlappingPair.h"

#include "ContactManiflod/rpContactManifoldSet.h"

namespace real_physics
{


// Constructor
rpOverlappingPair::rpOverlappingPair(rpProxyShape* shape1, rpProxyShape* shape2,  int nbMaxContactManifolds)
: mShape1(shape1) ,mShape2(shape2) ,
  mCachedSeparatingAxis(1.0, 1.0, 1.0) ,
  mContactManifoldSet(shape1, shape2, nbMaxContactManifolds)
{

}

// Destructor
rpOverlappingPair::~rpOverlappingPair()
{
	mContactManifoldSet.clear();
}


//SIMD_INLINE void real_physics::rpOverlappingPair::addContact(ContactPoint* contact)
//{
//	 mContactManifoldSet->addContactPoint(contact);
//}
//
//
//SIMD_INLINE void real_physics::rpOverlappingPair::update()
//{
//	 mContactManifoldSet->update();
//}



//SIMD_INLINE void real_physics::rpOverlappingPair::clearContactPoints()
//{
//	mContactManifoldSet->clear();
//}



} /* namespace real_physics */
