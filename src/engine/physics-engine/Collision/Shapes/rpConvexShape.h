/*
 * rpConvexShape.h
 *
 *  Created on: 16 нояб. 2016 г.
 *      Author: wera
 */

#ifndef COLLISION_SHAPES_RPCONVEXSHAPE_H_
#define COLLISION_SHAPES_RPCONVEXSHAPE_H_

#include "rpCollisionShape.h"

namespace real_physics
{

	/// Object margin for collision detection in meters (for the GJK-EPA Algorithm)
    const scalar OBJECT_MARGIN = scalar(0.001);

  // Class ConvexShape
  /**
   * This abstract class represents a convex collision shape associated with a
   * body that is used during the narrow-phase collision detection.
   */
  class rpConvexShape : public rpCollisionShape
  {

      protected :

          // -------------------- Attributes -------------------- //

          /// Margin used for the GJK collision detection algorithm
          scalar mMargin;

          // -------------------- Methods -------------------- //

          /// Private copy-constructor
          rpConvexShape(const rpConvexShape& shape);

          /// Private assignment operator
          rpConvexShape& operator=(const rpConvexShape& shape);


      public:

          // Return a local support point in a given direction with the object margin
          Vector3 getLocalSupportPointWithMargin(const Vector3& direction,
                                                 void** cachedCollisionData) const;

          /// Return a local support point in a given direction without the object margin
          virtual Vector3 getLocalSupportPointWithoutMargin(const Vector3& direction,
                                                            void** cachedCollisionData) const=0;

          /// Return true if a point is inside the collision shape
          virtual bool testPointInside(const Vector3& worldPoint, rpProxyShape* proxyShape) const=0;

      public :



          virtual void Draw()
          {

          }


          // -------------------- Methods -------------------- //

          /// Constructor
          rpConvexShape(CollisionShapeType type, scalar margin);

          /// Destructor
          virtual ~rpConvexShape();

          /// Return the current object margin
          scalar getMargin() const;

          /// Return true if the collision shape is convex, false if it is concave
          virtual bool isConvex() const;

          // -------------------- Friendship -------------------- //

          friend class GJKAlgorithm;
          friend class EPAAlgorithm;

          friend class  rpCollisionBody;
  };

  /// Return true if the collision shape is convex, false if it is concave
  SIMD_INLINE bool rpConvexShape::isConvex() const
  {
      return true;
  }

  // Return the current collision shape margin
  /**
   * @return The margin (in meters) around the collision shape
   */
  SIMD_INLINE scalar rpConvexShape::getMargin() const
  {
      return mMargin;
  }



} /* namespace real_physics */

#endif /* COLLISION_SHAPES_RPCONVEXSHAPE_H_ */
