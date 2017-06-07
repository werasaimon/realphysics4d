/*
 * rpAABB.cpp
 *
 *  Created on: 16 нояб. 2016 г.
 *      Author: wera
 */


  // Libraries
  #include <cassert>
  #include "rpAABB.h"



namespace real_physics
{


  using namespace std;

  // Constructor
  rpAABB::rpAABB()
  {

  }

  // Constructor
  rpAABB::rpAABB(const Vector3& minCoordinates, const Vector3& maxCoordinates)
       :mMinCoordinates(minCoordinates), mMaxCoordinates(maxCoordinates)
  {

  }

  // Copy-constructor
  rpAABB::rpAABB(const rpAABB& aabb)
       : mMinCoordinates(aabb.mMinCoordinates), mMaxCoordinates(aabb.mMaxCoordinates)
  {

  }

  // Destructor
  rpAABB::~rpAABB()
  {

  }

  // Merge the rpAABB in parameter with the current one
  void rpAABB::mergeWithAABB (const rpAABB& aabb)
  {

      mMinCoordinates.x = Min(mMinCoordinates.x, aabb.mMinCoordinates.x);
      mMinCoordinates.y = Min(mMinCoordinates.y, aabb.mMinCoordinates.y);
      mMinCoordinates.z = Min(mMinCoordinates.z, aabb.mMinCoordinates.z);

      mMaxCoordinates.x = Max(mMaxCoordinates.x, aabb.mMaxCoordinates.x);
      mMaxCoordinates.y = Max(mMaxCoordinates.y, aabb.mMaxCoordinates.y);
      mMaxCoordinates.z = Max(mMaxCoordinates.z, aabb.mMaxCoordinates.z);
  }

  // Replace the current AABB with a new AABB that is the union of two AABBs in parameters
  void rpAABB::mergeTwoAABBs(const rpAABB& aabb1, const rpAABB& aabb2)
  {
      mMinCoordinates.x = Min(aabb1.mMinCoordinates.x, aabb2.mMinCoordinates.x);
      mMinCoordinates.y = Min(aabb1.mMinCoordinates.y, aabb2.mMinCoordinates.y);
      mMinCoordinates.z = Min(aabb1.mMinCoordinates.z, aabb2.mMinCoordinates.z);

      mMaxCoordinates.x = Max(aabb1.mMaxCoordinates.x, aabb2.mMaxCoordinates.x);
      mMaxCoordinates.y = Max(aabb1.mMaxCoordinates.y, aabb2.mMaxCoordinates.y);
      mMaxCoordinates.z = Max(aabb1.mMaxCoordinates.z, aabb2.mMaxCoordinates.z);
  }

  // Return true if the current AABB contains the AABB given in parameter
  bool rpAABB::contains(const rpAABB& aabb) const
  {

      bool isInside = true;
      isInside = isInside && mMinCoordinates.x <= aabb.mMinCoordinates.x;
      isInside = isInside && mMinCoordinates.y <= aabb.mMinCoordinates.y;
      isInside = isInside && mMinCoordinates.z <= aabb.mMinCoordinates.z;

      isInside = isInside && mMaxCoordinates.x >= aabb.mMaxCoordinates.x;
      isInside = isInside && mMaxCoordinates.y >= aabb.mMaxCoordinates.y;
      isInside = isInside && mMaxCoordinates.z >= aabb.mMaxCoordinates.z;
      return isInside;
  }

  // Create and return an AABB for a triangle
  rpAABB rpAABB::createAABBForTriangle(const Vector3* trianglePoints)
  {

      Vector3 minCoords(trianglePoints[0].x, trianglePoints[0].y, trianglePoints[0].z);
      Vector3 maxCoords(trianglePoints[0].x, trianglePoints[0].y, trianglePoints[0].z);

      if (trianglePoints[1].x < minCoords.x) minCoords.x = trianglePoints[1].x;
      if (trianglePoints[1].y < minCoords.y) minCoords.y = trianglePoints[1].y;
      if (trianglePoints[1].z < minCoords.z) minCoords.z = trianglePoints[1].z;

      if (trianglePoints[2].x < minCoords.x) minCoords.x = trianglePoints[2].x;
      if (trianglePoints[2].y < minCoords.y) minCoords.y = trianglePoints[2].y;
      if (trianglePoints[2].z < minCoords.z) minCoords.z = trianglePoints[2].z;

      if (trianglePoints[1].x > maxCoords.x) maxCoords.x = trianglePoints[1].x;
      if (trianglePoints[1].y > maxCoords.y) maxCoords.y = trianglePoints[1].y;
      if (trianglePoints[1].z > maxCoords.z) maxCoords.z = trianglePoints[1].z;

      if (trianglePoints[2].x > maxCoords.x) maxCoords.x = trianglePoints[2].x;
      if (trianglePoints[2].y > maxCoords.y) maxCoords.y = trianglePoints[2].y;
      if (trianglePoints[2].z > maxCoords.z) maxCoords.z = trianglePoints[2].z;

      return rpAABB(minCoords, maxCoords);
  }

  // Return true if the ray intersects the AABB
  /// This method use the line vs AABB raycasting technique described in
  /// Real-time Collision Detection by Christer Ericson.
  bool rpAABB::testRayIntersect(const Ray& ray) const
  {

      const Vector3 point2 = ray.point1 + ray.maxFraction * (ray.point2 - ray.point1);
      const Vector3 e = mMaxCoordinates - mMinCoordinates;
      const Vector3 d = point2 - ray.point1;
      const Vector3 m = ray.point1 + point2 - mMinCoordinates - mMaxCoordinates;

      // Test if the AABB face normals are separating axis
      scalar adx = Abs(d.x);
      if (Abs(m.x) > e.x + adx) return false;
      scalar ady = Abs(d.y);
      if (Abs(m.y) > e.y + ady) return false;
      scalar adz = Abs(d.z);
      if (Abs(m.z) > e.z + adz) return false;

      // Add in an epsilon term to counteract arithmetic errors when segment is
      // (near) parallel to a coordinate axis (see text for detail)
      const scalar epsilon = 0.00001;
      adx += epsilon;
      ady += epsilon;
      adz += epsilon;

      // Test if the cross products between face normals and ray direction are
      // separating axis
      if (Abs(m.y * d.z - m.z * d.y) > e.y * adz + e.z * ady) return false;
      if (Abs(m.z * d.x - m.x * d.z) > e.x * adz + e.z * adx) return false;
      if (Abs(m.x * d.y - m.y * d.x) > e.x * ady + e.y * adx) return false;

      // No separating axis has been found
      return true;
  }

} /* namespace real_physics */
