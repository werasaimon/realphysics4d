/*
 * rpRaycastInfo.h
 *
 *  Created on: 16 нояб. 2016 г.
 *      Author: wera
 */

#ifndef COLLISION_RPRAYCASTINFO_H_
#define COLLISION_RPRAYCASTINFO_H_


#include "../LinearMaths/mathematics.h"

namespace real_physics
{

  // Declarations
  class rpCollisionBody;
  class rpProxyShape;
  class rpCollisionShape;

  // Structure RaycastInfo
  /**
   * This structure contains the information about a raycast hit.
   */
  struct RaycastInfo
  {

      private:

          // -------------------- Methods -------------------- //

          /// Private copy constructor
          RaycastInfo(const RaycastInfo& raycastInfo);

          /// Private assignment operator
          RaycastInfo& operator=(const RaycastInfo& raycastInfo);

      public:

          // -------------------- Attributes -------------------- //

          /// Hit point in world-space coordinates
          Vector3 worldPoint;

          /// Surface normal at hit point in world-space coordinates
          Vector3 worldNormal;

          /// Fraction distance of the hit point between point1 and point2 of the ray
          /// The hit point "p" is such that p = point1 + hitFraction * (point2 - point1)
          scalar hitFraction;

          /// Mesh subpart index that has been hit (only used for triangles mesh and -1 otherwise)
          int meshSubpart;

          /// Hit triangle index (only used for triangles mesh and -1 otherwise)
          int triangleIndex;

          /// Pointer to the hit collision body
          rpCollisionBody* body;

          /// Pointer to the hit proxy collision shape
          rpProxyShape* proxyShape;

          // -------------------- Methods -------------------- //

          /// Constructor
          RaycastInfo()
          : meshSubpart(-1), triangleIndex(-1) , proxyShape(NULL)
          {

          }

          /// Destructor
          ~RaycastInfo()
          {

          }
  };


  // Class RaycastCallback
  /**
   * This class can be used to register a callback for ray casting queries.
   * You should implement your own class inherited from this one and implement
   * the notifyRaycastHit() method. This method will be called for each ProxyShape
   * that is hit by the ray.
   */
  class RaycastCallback
  {

      public:

          // -------------------- Methods -------------------- //

          /// Destructor
          virtual ~RaycastCallback()
          {

          }

          /// This method will be called for each ProxyShape that is hit by the
          /// ray. You cannot make any assumptions about the order of the
          /// calls. You should use the return value to control the continuation
          /// of the ray. The returned value is the next maxFraction value to use.
          /// If you return a fraction of 0.0, it means that the raycast should
          /// terminate. If you return a fraction of 1.0, it indicates that the
          /// ray is not clipped and the ray cast should continue as if no hit
          /// occurred. If you return the fraction in the parameter (hitFraction
          /// value in the RaycastInfo object), the current ray will be clipped
          /// to this fraction in the next queries. If you return -1.0, it will
          /// ignore this ProxyShape and continue the ray cast.
          /**
           * @param raycastInfo Information about the raycast hit
           * @return Value that controls the continuation of the ray after a hit
           */
          virtual scalar notifyRaycastHit(const RaycastInfo& raycastInfo)=0;

  };

  /// Structure RaycastTest
  struct RaycastTest
  {

      public:

          /// User callback class
          RaycastCallback* userCallback;

          /// Constructor
          RaycastTest(RaycastCallback* callback)
          {
              userCallback = callback;
          }

          /// Ray cast test against a proxy shape
          scalar raycastAgainstShape(rpProxyShape* shape, const Ray& ray);
  };

} /* namespace real_physics */

#endif /* COLLISION_RPRAYCASTINFO_H_ */
