/*
 * rpProxyShape.cpp
 *
 *  Created on: 16 нояб. 2016 г.
 *      Author: wera
 */

#include "rpProxyShape.h"





#include <iostream>
using namespace std;

namespace real_physics
{


//#define PIKACHU_MIN_EPS 0.001

  // Constructor
  /**
   * @param body Pointer to the parent body
   * @param shape Pointer to the collision shape
   * @param transform Transformation from collision shape local-space to body local-space
   * @param mass Mass of the collision shape (in kilograms)
   */
  rpProxyShape::rpProxyShape( rpCollisionBody* body , rpCollisionShape* shape,
                              const Transform& transform, scalar mass)
   : mBody(body) , mCollisionShape(shape) ,
	 mLocalToBodyTransform(transform), mMass(mass),
     mNext(NULL),
	 mBroadPhaseID(-1),
	 mCachedCollisionData(NULL),
	 mUserData(NULL),
     mCollisionCategoryBits(0x0001),
	 mCollideWithMaskBits(0xFFFF)
  {

  }

  // Destructor
  rpProxyShape::~rpProxyShape()
  {

    // Release the cached collision data memory
      if (mCachedCollisionData != NULL)
      {
	         free(mCachedCollisionData);
      }
  }

  // Return true if a point is inside the collision shape
  /**
   * @param worldPoint Point to test in world-space coordinates
   * @return True if the point is inside the collision shape
   */
  bool rpProxyShape::testPointInside(const Vector3& worldPoint)
  {
      const Transform localToWorld = mLocalToBodyTransform;
      const Vector3 localPoint = localToWorld.getInverse() * worldPoint;
      return mCollisionShape->testPointInside(localPoint, this);
  }

  // Raycast method with feedback information
  /**
   * @param ray Ray to use for the raycasting
   * @param[out] raycastInfo Result of the raycasting that is valid only if the
   *             methods returned true
   * @return True if the ray hit the collision shape
   */
  bool rpProxyShape::raycast(const Ray& ray, RaycastInfo& raycastInfo)
  {

      // If the corresponding body is not active, it cannot be hit by rays
      if (!mBody->isActive()) return false;

      // Convert the ray into the local-space of the collision shape
      const Transform localToWorldTransform = getWorldTransform();
      const Transform worldToLocalTransform = localToWorldTransform.getInverse();

      Ray rayLocal(worldToLocalTransform * ray.point1,
                   worldToLocalTransform * ray.point2,
                   ray.maxFraction);

      bool isHit = mCollisionShape->raycast(rayLocal, raycastInfo, this);

      // Convert the raycast info into world-space
      raycastInfo.worldPoint  = localToWorldTransform * raycastInfo.worldPoint;
      raycastInfo.worldNormal = localToWorldTransform.getOrientation() * raycastInfo.worldNormal;
      raycastInfo.worldNormal.normalize();

      return isHit;
  }







} /* namespace real_physics */
