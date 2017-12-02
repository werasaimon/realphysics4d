/*
 * rpCollisionShape.cpp
 *
 *  Created on: 16 нояб. 2016 г.
 *      Author: wera
 */

#include "rpCollisionShape.h"
#include "../../LinearMaths/mathematics.h"

#include <GL/freeglut.h>

namespace real_physics
{


  // Constructor
  rpCollisionShape::rpCollisionShape(CollisionShapeType type)
  : mType(type),
	mScaling(1.0, 1.0, 1.0)
  {
	  mNbMaxPeturberationIteration = 10;
	  mEpsilonPeturberation = 0.001;
  }

  // Destructor
  rpCollisionShape::~rpCollisionShape()
  {

  }




  // Compute the world-space AABB of the collision shape given a transform
  /**
   * @param[out] aabb The axis-aligned bounding box (AABB) of the collision shape
   *                  computed in world-space coordinates
   * @param transform Transform used to compute the AABB of the collision shape
   */
  void rpCollisionShape::computeAABB(rpAABB& aabb, const Transform &transform0 , const Transform &transform1  ) const
  {

      Transform transform = transform0 * transform1;


      // Get the local bounds in x,y and z direction
      Vector3 minBounds;
      Vector3 maxBounds;
      getLocalBounds(minBounds, maxBounds);


      /// Scale size AABB local
      minBounds *= 1.04;
      maxBounds *= 1.04;


      // Rotate the local bounds according to the orientation of the body
      Matrix3x3 worldAxis =  transform.getBasis().getAbsoluteMatrix();
      Vector3 worldMinBounds(worldAxis.getRow(0).dot(minBounds),
                             worldAxis.getRow(1).dot(minBounds),
                             worldAxis.getRow(2).dot(minBounds));
      Vector3 worldMaxBounds(worldAxis.getRow(0).dot(maxBounds),
                             worldAxis.getRow(1).dot(maxBounds),
                             worldAxis.getRow(2).dot(maxBounds));



//      // Compute the minimum and maximum coordinates of the rotated extents
//       Vector3 minCoordinates = transform.getPosition() + worldMinBounds;
//       Vector3 maxCoordinates = transform.getPosition() + worldMaxBounds;



      Vector3 position = transform.getPosition();
      glPushMatrix();
      Vector3 halfSize = (worldMinBounds - worldMaxBounds);
      glTranslatef(position.x, position.y, position.z);
      glScalef(halfSize.x, halfSize.y, halfSize.z);
      glutWireCube(0.5);
      glPopMatrix();




      Vector3 minCoordinates = transform.getPosition() + worldMinBounds;
      Vector3 maxCoordinates = transform.getPosition() + worldMaxBounds;


      // Update the AABB with the new minimum and maximum coordinates
      aabb.setMin(minCoordinates);
      aabb.setMax(maxCoordinates);
  }



} /* namespace real_physics */
