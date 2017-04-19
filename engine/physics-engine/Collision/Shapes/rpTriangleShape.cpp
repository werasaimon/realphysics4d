/*
 * rpTriangleShape.cpp
 *
 *  Created on: 16 нояб. 2016 г.
 *      Author: wera
 */


 // Libraries
#include "rpTriangleShape.h"
#include "../rpProxyShape.h"
#include <cassert>


namespace real_physics
{




  // Constructor
  /**
   * @param point1 First point of the triangle
   * @param point2 Second point of the triangle
   * @param point3 Third point of the triangle
   * @param margin The collision margin (in meters) around the collision shape
   */
  rpTriangleShape::rpTriangleShape(const Vector3& point1, const Vector3& point2, const Vector3& point3, scalar margin)
   : rpConvexShape(TRIANGLE, margin)
  {
      mPoints[0] = point1;
      mPoints[1] = point2;
      mPoints[2] = point3;
      mRaycastTestType = FRONT;
  }

  // Destructor
  rpTriangleShape::~rpTriangleShape()
  {

  }

  // Raycast method with feedback information
  /// This method use the line vs triangle raycasting technique described in
  /// Real-time Collision Detection by Christer Ericson.
  bool rpTriangleShape::raycast(const Ray& ray, RaycastInfo& raycastInfo, rpProxyShape* proxyShape) const
  {

      //PROFILE("rpTriangleShape::raycast()");

      const Vector3 pq = ray.point2 - ray.point1;
      const Vector3 pa = mPoints[0] - ray.point1;
      const Vector3 pb = mPoints[1] - ray.point1;
      const Vector3 pc = mPoints[2] - ray.point1;

      // Test if the line PQ is inside the eges BC, CA and AB. We use the triple
      // product for this test.
      const Vector3 m = pq.cross(pc);
      scalar u = pb.dot(m);
      if (mRaycastTestType == FRONT)
      {
	  if (u < scalar(0.0)) return false;
      }
      else if (mRaycastTestType == BACK)
      {
	  if (u > scalar(0.0)) return false;
      }

      scalar v = -pa.dot(m);
      if (mRaycastTestType == FRONT)
       {
	  if (v < scalar(0.0)) return false;
      }
      else if (mRaycastTestType == BACK)
      {
	  if (v > scalar(0.0)) return false;
      }
      else if (mRaycastTestType == FRONT_AND_BACK)
      {
	  if (!sameSign(u, v)) return false;
      }

      scalar w = pa.dot(pq.cross(pb));
      if (mRaycastTestType == FRONT)
      {
	  if (w < scalar(0.0)) return false;
      }
      else if (mRaycastTestType == BACK)
      {
	  if (w > scalar(0.0)) return false;
      }
      else if (mRaycastTestType == FRONT_AND_BACK)
      {
	  if (!sameSign(u, w)) return false;
      }

      // If the line PQ is in the triangle plane (case where u=v=w=0)
      if (approxEqual(u, 0) && approxEqual(v, 0) && approxEqual(w, 0)) return false;

      // Compute the barycentric coordinates (u, v, w) to determine the
      // intersection point R, R = u * a + v * b + w * c
      scalar denom = scalar(1.0) / (u + v + w);
      u *= denom;
      v *= denom;
      w *= denom;

      // Compute the local hit point using the barycentric coordinates
      const Vector3 localHitPoint = u * mPoints[0] + v * mPoints[1] + w * mPoints[2];
      const scalar hitFraction = (localHitPoint - ray.point1).length() / pq.length();

      if (hitFraction < scalar(0.0) || hitFraction > ray.maxFraction) return false;

      Vector3 localHitNormal = (mPoints[1] - mPoints[0]).cross(mPoints[2] - mPoints[0]);
      if (localHitNormal.dot(pq) > scalar(0.0)) localHitNormal = -localHitNormal;

      raycastInfo.body = proxyShape->getBody();
      raycastInfo.proxyShape = proxyShape;
      raycastInfo.worldPoint = localHitPoint;
      raycastInfo.hitFraction = hitFraction;
      raycastInfo.worldNormal = localHitNormal;

      return true;
  }
} /* namespace real_physics */
