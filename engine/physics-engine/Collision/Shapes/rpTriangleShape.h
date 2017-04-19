/*
 * rpTriangleShape.h
 *
 *  Created on: 16 нояб. 2016 г.
 *      Author: wera
 */

#ifndef COLLISION_SHAPES_RPTRIANGLESHAPE_H_
#define COLLISION_SHAPES_RPTRIANGLESHAPE_H_


// Libraries
#include "../../LinearMaths/mathematics.h"
#include "rpConvexShape.h"

namespace real_physics
{

  /// Raycast test side for the triangle
  enum TriangleRaycastSide
  {

      /// Raycast against front triangle
      FRONT,

      /// Raycast against back triangle
      BACK,

      /// Raycast against front and back triangle
      FRONT_AND_BACK
  };




  static Matrix3x3 CalcInertionTriangle(const Vector3 P1, const Vector3 P2, const Vector3 P3 , scalar massa)
  {

      // Объем - смешанное произведение / 6
          float m =
          (
            P1[0]*P2[1]*P3[2] + P1[1]*P2[2]*P3[0] + P1[2]*P2[0]*P3[1] -
            P1[0]*P2[2]*P3[1] - P1[1]*P2[0]*P3[2] - P1[2]*P2[1]*P3[0]
          )/6.0f;

          massa += m;


          // Координаты вершин относительно центра масс треугольника
          Vector3 CenterOfMass = (P1 + P2 + P3) / 3.0f;

          Vector3 p1 = P1 - CenterOfMass;
          Vector3 p2 = P2 - CenterOfMass;
          Vector3 p3 = P3 - CenterOfMass;


  }


  // Class TriangleShape
  /**
   * This class represents a triangle collision shape that is centered
   * at the origin and defined three points.
   */
  class rpTriangleShape : public rpConvexShape
  {

      protected:

          // -------------------- Attribute -------------------- //

          /// Three points of the triangle
          Vector3 mPoints[3];

          /// Raycast test type for the triangle (front, back, front-back)
          TriangleRaycastSide mRaycastTestType;

          // -------------------- Methods -------------------- //

          /// Private copy-constructor
          rpTriangleShape(const rpTriangleShape& shape);

          /// Private assignment operator
          rpTriangleShape& operator=(const rpTriangleShape& shape);

          /// Return a local support point in a given direction without the object margin
          virtual Vector3 getLocalSupportPointWithoutMargin(const Vector3& direction,
                                                            void** cachedCollisionData) const;

          /// Return true if a point is inside the collision shape
          virtual bool testPointInside(const Vector3& localPoint, rpProxyShape* proxyShape) const;

          /// Raycast method with feedback information
          virtual bool raycast(const Ray& ray, RaycastInfo& raycastInfo, rpProxyShape* proxyShape) const;

          /// Return the number of bytes used by the collision shape
          virtual size_t getSizeInBytes() const;

      public:


          // -------------------- Methods -------------------- //

          /// Constructor
          rpTriangleShape(const Vector3& point1, const Vector3& point2, const Vector3& point3,
                          scalar margin = OBJECT_MARGIN);

          /// Destructor
          virtual ~rpTriangleShape();

          /// Return the local bounds of the shape in x, y and z directions.
          virtual void getLocalBounds(Vector3& min, Vector3& max) const;

          /// Set the local scaling vector of the collision shape
          virtual void setLocalScaling(const Vector3& scaling);

          /// Return the local inertia tensor of the collision shape
          virtual void computeLocalInertiaTensor(Matrix3x3& tensor, scalar mass) const;

          /// Update the AABB of a body using its collision shape
          virtual void computeAABB(rpAABB& aabb, const Transform& transform) const;

          /// Return the raycast test type (front, back, front-back)
          TriangleRaycastSide getRaycastTestType() const;

          // Set the raycast test type (front, back, front-back)
          void setRaycastTestType(TriangleRaycastSide testType);

          /// Return the coordinates of a given vertex of the triangle
          Vector3 getVertex(int index) const;

          // ---------- Friendship ---------- //

          friend class ConcaveMeshRaycastCallback;
          friend class TriangleOverlapCallback;
  };

  // Return the number of bytes used by the collision shape
  SIMD_INLINE size_t rpTriangleShape::getSizeInBytes() const
  {
      return sizeof(rpTriangleShape);
  }

  // Return a local support point in a given direction without the object margin
  SIMD_INLINE Vector3 rpTriangleShape::getLocalSupportPointWithoutMargin(const Vector3& direction,
                                                                void** cachedCollisionData) const
  {
      Vector3 dotProducts(direction.dot(mPoints[0]),
    		              direction.dot(mPoints[1]),
						  direction.dot(mPoints[2]));

      return mPoints[dotProducts.getMaxAxis()];
  }

  // Return the local bounds of the shape in x, y and z directions.
  // This method is used to compute the AABB of the box
  /**
   * @param min The minimum bounds of the shape in local-space coordinates
   * @param max The maximum bounds of the shape in local-space coordinates
   */
  SIMD_INLINE void rpTriangleShape::getLocalBounds(Vector3& min, Vector3& max) const
  {

      const Vector3 xAxis(mPoints[0].x, mPoints[1].x, mPoints[2].x);
      const Vector3 yAxis(mPoints[0].y, mPoints[1].y, mPoints[2].y);
      const Vector3 zAxis(mPoints[0].z, mPoints[1].z, mPoints[2].z);

      min.setAllValues(xAxis.getMinValue(), yAxis.getMinValue(), zAxis.getMinValue());
      max.setAllValues(xAxis.getMaxValue(), yAxis.getMaxValue(), zAxis.getMaxValue());

      min -= Vector3(mMargin, mMargin, mMargin);
      max += Vector3(mMargin, mMargin, mMargin);
  }

  // Set the local scaling vector of the collision shape
  SIMD_INLINE void rpTriangleShape::setLocalScaling(const Vector3& scaling)
  {

      mPoints[0] = (mPoints[0] / mScaling) * scaling;
      mPoints[1] = (mPoints[1] / mScaling) * scaling;
      mPoints[2] = (mPoints[2] / mScaling) * scaling;

      rpCollisionShape::setLocalScaling(scaling);
  }

  // Return the local inertia tensor of the triangle shape
  /**
   * @param[out] tensor The 3x3 inertia tensor matrix of the shape in local-space
   *                    coordinates
   * @param mass Mass to use to compute the inertia tensor of the collision shape
   */
  SIMD_INLINE void rpTriangleShape::computeLocalInertiaTensor(Matrix3x3& tensor, scalar mass) const
  {
      tensor.setToZero();
	  //tensor.setToIdentity();
	  //tensor *= scalar(mass * 3.0);
	  //tensor = CalcInertionTriangle(mPoints[0], mPoints[1], mPoints[2], mass);
  }

  // Update the AABB of a body using its collision shape
  /**
   * @param[out] aabb The axis-aligned bounding box (AABB) of the collision shape
   *                  computed in world-space coordinates
   * @param transform Transform used to compute the AABB of the collision shape
   */
  SIMD_INLINE void rpTriangleShape::computeAABB(rpAABB& aabb, const Transform& transform) const
  {

      const Vector3 worldPoint1 = transform * mPoints[0];
      const Vector3 worldPoint2 = transform * mPoints[1];
      const Vector3 worldPoint3 = transform * mPoints[2];



      const Vector3 xAxis(worldPoint1.x, worldPoint2.x, worldPoint3.x);
      const Vector3 yAxis(worldPoint1.y, worldPoint2.y, worldPoint3.y);
      const Vector3 zAxis(worldPoint1.z, worldPoint2.z, worldPoint3.z);

      aabb.setMin(Vector3(xAxis.getMinValue(), yAxis.getMinValue(), zAxis.getMinValue()));
      aabb.setMax(Vector3(xAxis.getMaxValue(), yAxis.getMaxValue(), zAxis.getMaxValue()));

  }

  // Return true if a point is inside the collision shape
  SIMD_INLINE bool rpTriangleShape::testPointInside(const Vector3& localPoint, rpProxyShape* proxyShape) const
  {
      return false;
  }

  // Return the raycast test type (front, back, front-back)
  SIMD_INLINE TriangleRaycastSide rpTriangleShape::getRaycastTestType() const
  {
      return mRaycastTestType;
  }

  // Set the raycast test type (front, back, front-back)
  /**
   * @param testType Raycast test type for the triangle (front, back, front-back)
   */
  SIMD_INLINE void rpTriangleShape::setRaycastTestType(TriangleRaycastSide testType)
  {
      mRaycastTestType = testType;
  }

  // Return the coordinates of a given vertex of the triangle
  /**
   * @param index Index (0 to 2) of a vertex of the triangle
   */
  SIMD_INLINE Vector3 rpTriangleShape::getVertex(int index) const
  {
      assert(index >= 0 && index < 3);
      return mPoints[index];
  }


} /* namespace real_physics */

#endif /* COLLISION_SHAPES_RPTRIANGLESHAPE_H_ */
