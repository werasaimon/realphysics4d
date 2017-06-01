/*
 * rpConvexShape.cpp
 *
 *  Created on: 16 нояб. 2016 г.
 *      Author: wera
 */

#include "rpConvexShape.h"



#include <iostream>
using namespace std;

namespace real_physics
{


  // Constructor
  rpConvexShape::rpConvexShape(CollisionShapeType type, scalar margin)
  : rpCollisionShape(type), mMargin(margin)
  {

  }

  // Destructor
  rpConvexShape::~rpConvexShape()
  {

  }

  // Return a local support point in a given direction with the object margin
  Vector3 rpConvexShape::getLocalSupportPointWithMargin(const Vector3& direction,
                                                        void** cachedCollisionData) const
  {

//	 std::cout<< mMargin <<std::endl;

      // Get the support point without margin
      Vector3 supportPoint = getLocalSupportPointWithoutMargin(direction, cachedCollisionData);

      if (mMargin != scalar(0.0))
      {
          // Add the margin to the support point
          Vector3 unitVec(0.0, -1.0, 0.0);
          if (direction.lengthSquare() > MACHINE_EPSILON * MACHINE_EPSILON)
          {
              unitVec = direction.getUnit();
          }
          supportPoint += unitVec * mMargin;
      }

      return supportPoint;
  }


} /* namespace real_physics */
