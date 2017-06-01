/*
 * rpAABB.h
 *
 *  Created on: 16 нояб. 2016 г.
 *      Author: wera
 */

#ifndef COLLISION_SHAPES_RPAABB_H_
#define COLLISION_SHAPES_RPAABB_H_

// Libraries
#include "../../LinearMaths/mathematics.h"

namespace real_physics
{

  // Class AABB
  /**
   * This class represents a bounding volume of type "Axis Aligned
   * Bounding Box". It's a box where all the edges are always aligned
   * with the world coordinate system. The AABB is defined by the
   * minimum and maximum world coordinates of the three axis.
   */
  class rpAABB
  {

      private :

          // -------------------- Attributes -------------------- //

          /// Minimum world coordinates of the rpAABB on the x,y and z axis
          Vector3 mMinCoordinates;

          /// Maximum world coordinates of the rpAABB on the x,y and z axis
          Vector3 mMaxCoordinates;

      public :

          // -------------------- Methods -------------------- //

          /// Constructor
          rpAABB();

          /// Constructor
          rpAABB(const Vector3& minCoordinates, const Vector3& maxCoordinates);

          /// Copy-constructor
          rpAABB(const rpAABB& aabb);

          /// Destructor
          ~rpAABB();

          /// Return the center point
          Vector3 getCenter() const;

          /// Return the minimum coordinates of the rpAABB
          const Vector3& getMin() const;

          /// Set the minimum coordinates of the rpAABB
          void setMin(const Vector3& min);

          /// Return the maximum coordinates of the rpAABB
          const Vector3& getMax() const;

          /// Set the maximum coordinates of the rpAABB
          void setMax(const Vector3& max);

          /// Return the size of the rpAABB in the three dimension x, y and z
          Vector3 getExtent() const;

          /// Inflate each side of the rpAABB by a given size
          void inflate(scalar dx, scalar dy, scalar dz);

          /// Return true if the current rpAABB is overlapping with the rpAABB in argument
          bool testCollision(const rpAABB& aabb) const;

          /// Return the volume of the rpAABB
          scalar getVolume() const;

          /// Merge the rpAABB in parameter with the current one
          void mergeWithAABB(const rpAABB& aabb);

          /// Replace the current rpAABB with a new rpAABB that is the union of two rpAABBs in parameters
          void mergeTwoAABBs(const rpAABB& aabb1, const rpAABB& aabb2);

          /// Return true if the current rpAABB contains the rpAABB given in parameter
          bool contains(const rpAABB& aabb) const;

          /// Return true if a point is inside the rpAABB
          bool contains(const Vector3& point) const;

          /// Return true if the rpAABB of a triangle intersects the rpAABB
          bool testCollisionTriangleAABB(const Vector3* trianglePoints) const;

          /// Return true if the ray intersects the rpAABB
          bool testRayIntersect(const Ray& ray) const;

          /// Create and return an rpAABB for a triangle
          static rpAABB createAABBForTriangle(const Vector3* trianglePoints);

          /// Assignment operator
          rpAABB& operator=(const rpAABB& aabb);

          // -------------------- Friendship -------------------- //

          friend class rpDynamicAABBTree;
  };

  // Return the center point of the rpAABB in world coordinates
  SIMD_INLINE Vector3 rpAABB::getCenter() const
  {
      return (mMinCoordinates + mMaxCoordinates) * scalar(0.5);
  }

  // Return the minimum coordinates of the rpAABB
  SIMD_INLINE const Vector3& rpAABB::getMin() const
  {
      return mMinCoordinates;
  }

  // Set the minimum coordinates of the rpAABB
  SIMD_INLINE void rpAABB::setMin(const Vector3& min)
  {
      mMinCoordinates = min;
  }

  // Return the maximum coordinates of the rpAABB
  SIMD_INLINE const Vector3& rpAABB::getMax() const
  {
      return mMaxCoordinates;
  }

  // Set the maximum coordinates of the rpAABB
  SIMD_INLINE void rpAABB::setMax(const Vector3& max)
  {
      mMaxCoordinates = max;
  }

  // Return the size of the rpAABB in the three dimension x, y and z
  SIMD_INLINE Vector3 rpAABB::getExtent() const
  {
    return  mMaxCoordinates - mMinCoordinates;
  }

  // Inflate each side of the rpAABB by a given size
  SIMD_INLINE void rpAABB::inflate(scalar dx, scalar dy, scalar dz)
  {
      mMaxCoordinates += Vector3(dx, dy, dz);
      mMinCoordinates -= Vector3(dx, dy, dz);
  }

  // Return true if the current rpAABB is overlapping with the rpAABB in argument.
  /// Two rpAABBs overlap if they overlap in the three x, y and z axis at the same time
  SIMD_INLINE bool rpAABB::testCollision(const rpAABB& aabb) const
  {
      if (mMaxCoordinates.x < aabb.mMinCoordinates.x ||
          aabb.mMaxCoordinates.x < mMinCoordinates.x) return false;
      if (mMaxCoordinates.y < aabb.mMinCoordinates.y ||
          aabb.mMaxCoordinates.y < mMinCoordinates.y) return false;
      if (mMaxCoordinates.z < aabb.mMinCoordinates.z||
          aabb.mMaxCoordinates.z < mMinCoordinates.z) return false;
      return true;
  }

  // Return the volume of the AABB
  SIMD_INLINE scalar rpAABB::getVolume() const
  {
      const Vector3 diff = mMaxCoordinates - mMinCoordinates;
      return (diff.x * diff.y * diff.z);
  }

  // Return true if the rpAABB of a triangle intersects the rpAABB
  SIMD_INLINE bool rpAABB::testCollisionTriangleAABB(const Vector3* trianglePoints) const
  {

      if (min3(trianglePoints[0].x, trianglePoints[1].x, trianglePoints[2].x) > mMaxCoordinates.x) return false;
      if (min3(trianglePoints[0].y, trianglePoints[1].y, trianglePoints[2].y) > mMaxCoordinates.y) return false;
      if (min3(trianglePoints[0].z, trianglePoints[1].z, trianglePoints[2].z) > mMaxCoordinates.z) return false;

      if (max3(trianglePoints[0].x, trianglePoints[1].x, trianglePoints[2].x) < mMinCoordinates.x) return false;
      if (max3(trianglePoints[0].y, trianglePoints[1].y, trianglePoints[2].y) < mMinCoordinates.y) return false;
      if (max3(trianglePoints[0].z, trianglePoints[1].z, trianglePoints[2].z) < mMinCoordinates.z) return false;

      return true;
  }

  // Return true if a point is inside the rpAABB
  SIMD_INLINE bool rpAABB::contains(const Vector3& point) const
  {

      return (point.x >= mMinCoordinates.x - MACHINE_EPSILON && point.x <= mMaxCoordinates.x + MACHINE_EPSILON &&
              point.y >= mMinCoordinates.y - MACHINE_EPSILON && point.y <= mMaxCoordinates.y + MACHINE_EPSILON &&
              point.z >= mMinCoordinates.z - MACHINE_EPSILON && point.z <= mMaxCoordinates.z + MACHINE_EPSILON);
  }



  // Assignment operator
  SIMD_INLINE rpAABB& rpAABB::operator=(const rpAABB& aabb)
  {
      if (this != &aabb)
      {
          mMinCoordinates = aabb.mMinCoordinates;
          mMaxCoordinates = aabb.mMaxCoordinates;
      }
      return *this;
  }
} /* namespace real_physics */

#endif /* COLLISION_SHAPES_RPAABB_H_ */
