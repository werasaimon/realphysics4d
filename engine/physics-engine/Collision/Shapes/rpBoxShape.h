/*
 * rpBoxShape.h
 *
 *  Created on: 17 нояб. 2016 г.
 *      Author: wera
 */

#ifndef SOURCE_ENGIE_COLLISION_SHAPES_RPBOXSHAPE_H_
#define SOURCE_ENGIE_COLLISION_SHAPES_RPBOXSHAPE_H_

#include "rpConvexShape.h"

namespace real_physics
{





// Class BoxShape
/**
 * This class represents a 3D box shape. Those axis are unit length.
 * The three extents are half-widths of the box along the three
 * axis x, y, z local axis. The "transform" of the corresponding
 * rigid body will give an orientation and a position to the box. This
 * collision shape uses an extra margin distance around it for collision
 * detection purpose. The default margin is 4cm (if your units are meters,
 * which is recommended). In case, you want to simulate small objects
 * (smaller than the margin distance), you might want to reduce the margin by
 * specifying your own margin distance using the "margin" parameter in the
 * constructor of the box shape. Otherwise, it is recommended to use the
 * default margin distance by not using the "margin" parameter in the constructor.
 */
	class rpBoxShape : public rpConvexShape
	{

	protected :

		// -------------------- Attributes -------------------- //

		/// Extent sizes of the box in the x, y and z direction
		Vector3 mExtent;

		// -------------------- Methods -------------------- //

		/// Private copy-constructor
		rpBoxShape(const rpBoxShape& shape);

		/// Private assignment operator
		rpBoxShape& operator=(const rpBoxShape& shape);

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
        rpBoxShape(const Vector3& extent );


		/// Constructor
        rpBoxShape(const Vector3& extent, scalar margin );

		/// Destructor
		virtual ~rpBoxShape();

		/// Return the extents of the box
		Vector3 getExtent() const;

		/// Set the scaling vector of the collision shape
		virtual void setLocalScaling(const Vector3& scaling);

		/// Return the local bounds of the shape in x, y and z directions
		virtual void getLocalBounds(Vector3& min, Vector3& max) const;

		/// Return the local inertia tensor of the collision shape
		virtual void computeLocalInertiaTensor(Matrix3x3& tensor, scalar mass) const;
	};

	// Return the extents of the box
	/**
	 * @return The vector with the three extents of the box shape (in meters)
	 */
	SIMD_INLINE Vector3 rpBoxShape::getExtent() const
	{
		return mExtent + Vector3(mMargin, mMargin, mMargin);
	}

	// Set the scaling vector of the collision shape
	SIMD_INLINE void rpBoxShape::setLocalScaling(const Vector3& scaling)
	{

		mExtent = (mExtent / mScaling) * scaling;

		rpCollisionShape::setLocalScaling(scaling);
	}

	// Return the local bounds of the shape in x, y and z directions
	/// This method is used to compute the AABB of the box
	/**
	 * @param min The minimum bounds of the shape in local-space coordinates
	 * @param max The maximum bounds of the shape in local-space coordinates
	 */
	SIMD_INLINE void rpBoxShape::getLocalBounds(Vector3& min, Vector3& max) const
	{

		// Maximum bounds
		max = mExtent + Vector3(mMargin, mMargin, mMargin);

		// Minimum bounds
		min = -max;
	}

	// Return the number of bytes used by the collision shape
	SIMD_INLINE size_t rpBoxShape::getSizeInBytes() const
	{
		return sizeof(rpBoxShape);
	}

	// Return a local support point in a given direction without the objec margin
	SIMD_INLINE Vector3 rpBoxShape::getLocalSupportPointWithoutMargin(const Vector3& direction,
			                                                   void** cachedCollisionData) const
	{

		return Vector3(direction.x < 0.0 ? -mExtent.x : mExtent.x,
				       direction.y < 0.0 ? -mExtent.y : mExtent.y,
					   direction.z < 0.0 ? -mExtent.z : mExtent.z);

	}

	// Return true if a point is inside the collision shape
	SIMD_INLINE bool rpBoxShape::testPointInside(const Vector3& localPoint, rpProxyShape* proxyShape) const
	{
		return (localPoint.x < mExtent[0] && localPoint.x > -mExtent[0] &&
				localPoint.y < mExtent[1] && localPoint.y > -mExtent[1] &&
				localPoint.z < mExtent[2] && localPoint.z > -mExtent[2]);
	}





} /* namespace real_physics */

#endif /* SOURCE_ENGIE_COLLISION_SHAPES_RPBOXSHAPE_H_ */
