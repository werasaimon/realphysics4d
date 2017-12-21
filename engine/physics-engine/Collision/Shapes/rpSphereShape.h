/*
 * rpSphereShape.h
 *
 *  Created on: 18 нояб. 2016 г.
 *      Author: wera
 */

#ifndef SOURCE_ENGIE_COLLISION_SHAPES_RPSPHERESHAPE_H_
#define SOURCE_ENGIE_COLLISION_SHAPES_RPSPHERESHAPE_H_

#include "rpConvexShape.h"

namespace real_physics
{


// Class SphereShape
/**
 * This class represents a sphere collision shape that is centered
 * at the origin and defined by its radius. This collision shape does not
 * have an explicit object margin distance. The margin is implicitly the
 * radius of the sphere. Therefore, no need to specify an object margin
 * for a sphere shape.
 */
class rpSphereShape : public rpConvexShape
{

    protected :


        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        rpSphereShape(const rpSphereShape& shape );

        /// Private assignment operator
        rpSphereShape& operator=(const rpSphereShape& shape);

        /// Return a local support point in a given direction without the object margin
        virtual Vector3 getLocalSupportPointWithoutMargin(const Vector3& direction,
                                                          void** cachedCollisionData) const;

        /// Return true if a point is inside the collision shape
        virtual bool testPointInside(const Vector3& localPoint, rpProxyShape* proxyShape) const;

        /// Raycast method with feedback information
        virtual bool raycast(const Ray& ray, RaycastInfo& raycastInfo, rpProxyShape* proxyShape) const;

        /// Return the number of bytes used by the collision shape
        virtual size_t getSizeInBytes() const;

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        rpSphereShape(scalar radius);

        /// Destructor
        virtual ~rpSphereShape();

        /// Return the radius of the sphere
        scalar getRadius() const;

        /// Set the scaling vector of the collision shape
        virtual void setLocalScaling(const Vector3& scaling);

        /// Return the local bounds of the shape in x, y and z directions.
        virtual void getLocalBounds(Vector3& min, Vector3& max) const;

        /// Return the local inertia tensor of the collision shape
        virtual void computeLocalInertiaTensor(Matrix3x3& tensor, scalar mass) const;

        /// Update the AABB of a body using its collision shape
        virtual void computeAABB(rpAABB& aabb, const Transform& transform) const;
};

// Get the radius of the sphere
/**
 * @return Radius of the sphere (in meters)
 */
SIMD_INLINE scalar rpSphereShape::getRadius() const
{
    return mMargin;
}

// Set the scaling vector of the collision shape
SIMD_INLINE void rpSphereShape::setLocalScaling(const Vector3& scaling)
{
    mMargin = (mMargin / mScaling.x) * scaling.x;
    rpCollisionShape::setLocalScaling(scaling);
}

// Return the number of bytes used by the collision shape
SIMD_INLINE size_t rpSphereShape::getSizeInBytes() const
{
    return sizeof(rpSphereShape);
}

// Return a local support point in a given direction without the object margin
SIMD_INLINE Vector3 rpSphereShape::getLocalSupportPointWithoutMargin(const Vector3& direction,
                                                              void** cachedCollisionData) const
{

//    // Return the center of the sphere (the radius is taken into account in the object margin)
//    return  direction.getUnit() * mRadius;

    // Return the center of the sphere (the radius is taken into account in the object margin)
    return Vector3(0.0, 0.0, 0.0);
}

// Return the local bounds of the shape in x, y and z directions.
// This method is used to compute the AABB of the box
/**
 * @param min The minimum bounds of the shape in local-space coordinates
 * @param max The maximum bounds of the shape in local-space coordinates
 */
SIMD_INLINE void rpSphereShape::getLocalBounds(Vector3& min, Vector3& max) const
{

    // Maximum bounds
    max.x = mMargin;
    max.y = mMargin;
    max.z = mMargin;

    // Minimum bounds
    min.x = -mMargin;
    min.y = -mMargin;
    min.z = -mMargin;
}

// Return the local inertia tensor of the sphere
/**
 * @param[out] tensor The 3x3 inertia tensor matrix of the shape in local-space
 *                    coordinates
 * @param mass Mass to use to compute the inertia tensor of the collision shape
 */
SIMD_INLINE void rpSphereShape::computeLocalInertiaTensor(Matrix3x3& tensor, scalar mass) const
{
    scalar diag = scalar(0.4) * mass * mMargin * mMargin;
    tensor.setAllValues(diag, 0.0, 0.0,
                        0.0, diag, 0.0,
                        0.0, 0.0, diag);
}

// Update the AABB of a body using its collision shape
/**
 * @param[out] aabb The axis-aligned bounding box (AABB) of the collision shape
 *                  computed in world-space coordinates
 * @param transform Transform used to compute the AABB of the collision shape
 */
SIMD_INLINE void rpSphereShape::computeAABB(rpAABB& aabb, const Transform& transform) const
{

    // Get the local extents in x,y and z direction
    Vector3 extents(mMargin, mMargin, mMargin);

    // Update the AABB with the new minimum and maximum coordinates
    aabb.setMin(transform.getPosition4().getPos() - extents);
    aabb.setMax(transform.getPosition4().getPos() + extents);
}

// Return true if a point is inside the collision shape
SIMD_INLINE bool rpSphereShape::testPointInside(const Vector3& localPoint, rpProxyShape* proxyShape) const
{
    return (localPoint.lengthSquare() < mMargin * mMargin);
}
} /* namespace real_physics */

#endif /* SOURCE_ENGIE_COLLISION_SHAPES_RPSPHERESHAPE_H_ */
