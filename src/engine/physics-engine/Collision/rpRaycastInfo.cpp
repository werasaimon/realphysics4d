/*
 * rpRaycastInfo.cpp
 *
 *  Created on: 16 нояб. 2016 г.
 *      Author: wera
 */


 // Libraries
#include "rpRaycastInfo.h"
#include "rpProxyShape.h"

namespace real_physics
{


  // Ray cast test against a proxy shape
  scalar RaycastTest::raycastAgainstShape(rpProxyShape* shape, const Ray& ray)
  {

      // Ray casting test against the collision shape
      RaycastInfo raycastInfo;
      bool isHit = shape->raycast(ray, raycastInfo);

      // If the ray hit the collision shape
      if (isHit)
      {
          // Report the hit to the user and return the
          // user hit fraction value
          return userCallback->notifyRaycastHit(raycastInfo);
      }

      return ray.maxFraction;
  }

} /* namespace real_physics */
