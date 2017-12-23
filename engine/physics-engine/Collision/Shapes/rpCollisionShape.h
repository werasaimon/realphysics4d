/*
 * rpCollisionShape.h
 *
 *  Created on: 16 нояб. 2016 г.
 *      Author: wera
 */

#ifndef COLLISION_SHAPES_RPCOLLISIONSHAPE_H_
#define COLLISION_SHAPES_RPCOLLISIONSHAPE_H_

#include <iostream>
#include <cstdlib>

// Libraries
#include <cassert>
#include <typeinfo>

#include "../../LinearMaths/mathematics.h"
#include "../rpRaycastInfo.h"
#include "rpAABB.h"



namespace real_physics
{


  /// Type of the collision shape
  enum CollisionShapeType {TRIANGLE,
	                       BOX,
						   SPHERE,
						   CONE,
						   CYLINDER,
                           CAPSULE, CONVEX_MESH ,
						   CONVEX_HULL_MESH ,
						   CONCAVE_MESH,
						   HEIGHTFIELD};







  //Extern declarations
  class rpProxyShape;


  // Class CollisionShape
  /**
   * This abstract class represents the collision shape associated with a
   * body that is used during the narrow-phase collision detection.
   */
  class rpCollisionShape
  {

      protected :

	  // -------------------- Attributes -------------------- //

          /// Type of the collision shape
          CollisionShapeType mType;

          /// Scaling vector of the collision shape
          Vector3 mScaling;



          /// Max iterration peturbiration
          int    mNbMaxPeturberationIteration;
          /// Eppsiolon in peturbiration
          scalar mEpsilonPeturberation;


          // -------------------- Methods -------------------- //

          /// Private copy-constructor
          rpCollisionShape(const rpCollisionShape& shape);

          /// Private assignment operator
          rpCollisionShape& operator=(const rpCollisionShape& shape);

          /// Return true if a point is inside the collision shape
          virtual bool testPointInside(const Vector3& worldPoint, rpProxyShape* proxyShape) const {}

          /// Raycast method with feedback information
          virtual bool raycast(const Ray& ray, RaycastInfo& raycastInfo, rpProxyShape* proxyShape) const {}

          /// Return the number of bytes used by the collision shape
          virtual size_t getSizeInBytes() const {}


          /// Return a local support point in a given direction with the object margin
          virtual Vector3 getLocalSupportPointWithMargin(const Vector3& direction, void** cachedCollisionData = NULL ) const {}



      public:


          /// Return a local support point in a given direction with the object margin
          const Vector3 getLocalSupportPointWithMarginn(const Vector3& direction ) const
          {
              return getLocalSupportPointWithMargin( direction , NULL) * mScaling;
          }



//          virtual  Vector3* getAxisPeturberationPoints( const Vector3& xAxis , const Transform& worldTransform , int &_NbPoints) const
//          {
//
//        	  Matrix3x3 InverseRotate = worldTransform.getBasis().getTranspose();
//        	  Vector3 axis = InverseRotate * xAxis;
//
//        	  Vector3 n0, n1;
//        	  Vector3::btPlaneSpace1(axis , n0 , n1 );
//
//        	  int    NbCount = mNbMaxPeturberationIteration + 1;
//        	  scalar epsilon = mEpsilonPeturberation;//0.077f;
//
//        	  Vector3 startPoint;
//        	  Vector3 OldPoint;
//
//        	  //Vector3* resultVertices = new Vector3[NbCount];
//
//        	  Vector3* result = NULL;
//
//        	  int &num_element = _NbPoints;
//        	  for( int i = 0; i < NbCount; i++ )
//        	  {
//        		  scalar ang = (2.0f * Pi() / scalar(NbCount)) * scalar(i);
//
//        		  Vector3 auxAxis = (axis + n0 * Cos(ang) * epsilon
//        				                  + n1 * Sin(ang) * epsilon);
//
//
//        		  /***************************************************************/
//
//        		  Vector3 sp = getLocalSupportPointWithMarginn( auxAxis );
//        		  Vector3 spVertex = worldTransform * sp;
//
//
//        		  /*-----------------------------------------------------*/
//        		  if ((spVertex - OldPoint).length() > 0.02 || i == 0)
//        		  {
//        			  if ((spVertex - startPoint).length() < 0.02 && num_element ) break;
//
//        			  //axVertices[num_element++] = spVertex;
//        			   //resultVertices = new Vector3;
//        			   //resultVertices[num_element++] = spVertex;
//
//
//        			  /***************************************/
//        			   num_element++;
//        			   Vector3* array = (Vector3*) realloc(result , num_element * sizeof(Vector3));
//        			   if (array != NULL)
//        			   {
//        			      result = array;
//        			      result[num_element - 1] = spVertex;                      // добавить к массиву только что введённое число
//        			   }
//        			   else
//        			   {
//        				   free(result);
//        			   }
//
//        			   /***************************************/
//
//
//
//        			  OldPoint = spVertex;
//        			  if (num_element == 1) startPoint = spVertex;
//        		  }
//        		  /*-----------------------------------------------------*/
//        	  }
//
//        	  //return resultVertices;
//        	  return result;
//          }



          // -------------------- Methods -------------------- //

          /// Constructor
          rpCollisionShape(CollisionShapeType type);

          /// Destructor
          virtual ~rpCollisionShape();

          /// Return the type of the collision shapes
          CollisionShapeType getType() const;

          /// Return true if the collision shape is convex, false if it is concave
          virtual bool isConvex() const {}

          /// Return the local bounds of the shape in x, y and z directions
          virtual void getLocalBounds(Vector3& min, Vector3& max) const {}

          /// Return the scaling vector of the collision shape
          Vector3 getScaling() const;

          /// Set the local scaling vector of the collision shape
          virtual void setLocalScaling(const Vector3& scaling);

          /// Return the local inertia tensor of the collision shapes
          virtual void computeLocalInertiaTensor(Matrix3x3& tensor, scalar mass) const {}

          /// Compute the world-space AABB of the collision shape given a transform
          virtual void computeAABB(rpAABB& aabb, const Transform& transform0 , const Transform& transform1 ) const;

          /// Return true if the collision shape type is a convex shape
          static bool isConvex(CollisionShapeType shapeType);

          /// Return the maximum number of contact manifolds in an overlapping pair given two shape types
          static int computeNbMaxContactManifolds(CollisionShapeType shapeType1,
                                                  CollisionShapeType shapeType2);

          // -------------------- Friendship -------------------- //

          friend class rpProxyShape;
          friend class rpCollisionWorld;
          friend class rpCollisionBody;
  };


  // Return the type of the collision shape
  /**
   * @return The type of the collision shape (box, sphere, cylinder, ...)
   */
  SIMD_INLINE CollisionShapeType rpCollisionShape::getType() const
  {
      return mType;
  }

  // Return true if the collision shape type is a convex shape
  SIMD_INLINE bool rpCollisionShape::isConvex(CollisionShapeType shapeType)
  {
      return shapeType != CONCAVE_MESH && shapeType != HEIGHTFIELD;
  }

  // Return the scaling vector of the collision shape
  SIMD_INLINE Vector3 rpCollisionShape::getScaling() const
  {
      return mScaling;
  }

  // Set the scaling vector of the collision shape
  SIMD_INLINE void rpCollisionShape::setLocalScaling(const Vector3& scaling)
  {
      mScaling = scaling;
  }

  // Return the maximum number of contact manifolds allowed in an overlapping
  // pair wit the given two collision shape types
  SIMD_INLINE int rpCollisionShape::computeNbMaxContactManifolds(CollisionShapeType shapeType1,
                                                                 CollisionShapeType shapeType2)
  {
      // If both shapes are convex
      if (isConvex(shapeType1) && isConvex(shapeType2))
      {
          return NB_MAX_CONTACT_MANIFOLDS_CONVEX_SHAPE;
      }   // If there is at least one concave shape
      else
      {
          return NB_MAX_CONTACT_MANIFOLDS_CONCAVE_SHAPE;
      }
  }

} /* namespace real_physics */

#endif /* COLLISION_SHAPES_RPCOLLISIONSHAPE_H_ */
