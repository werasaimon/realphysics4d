/*
 * rpProxyShape.h
 *
 *  Created on: 16 нояб. 2016 г.
 *      Author: wera
 */

#ifndef COLLISION_RPPROXYSHAPE_H_
#define COLLISION_RPPROXYSHAPE_H_


#include "Shapes/rpCollisionShape.h"
#include "../Body/rpCollisionBody.h"

namespace real_physics
{


  // Class ProxyShape
  /**
   * The CollisionShape instances are supposed to be unique for memory optimization. For instance,
   * consider two rigid bodies with the same sphere collision shape. In this situation, we will have
   * a unique instance of SphereShape but we need to differentiate between the two instances during
   * the collision detection. They do not have the same position in the world and they do not
   * belong to the same rigid body. The ProxyShape class is used for that purpose by attaching a
   * rigid body with one of its collision shape. A body can have multiple proxy shapes (one for
   * each collision shape attached to the body).
   */
  class rpProxyShape
  {

      protected:

          // -------------------- Attributes -------------------- //

          /// Pointer to the parent body
          rpCollisionBody*  mBody;

          /// Internal collision shape
          rpCollisionShape* mCollisionShape;

          /// Local-space to parent body-space transform (does not change over time)
          Transform         mLocalToBodyTransform;

          /// Mass (in kilogramms) of the corresponding collision shape
          scalar            mMass;

          /// Pointer to the next proxy shape of the body (linked list)
          rpProxyShape*     mNext;

          /// Broad-phase ID (node ID in the dynamic AABB tree)
          int               mBroadPhaseID;

          /// Cached collision data
          void*             mCachedCollisionData;

          /// Pointer to user data
          void*             mUserData;




          /// Bits used to define the collision category of this shape.
          /// You can set a single bit to one to define a category value for this
          /// shape. This value is one (0x0001) by default. This variable can be used
          /// together with the mCollideWithMaskBits variable so that given
          /// categories of shapes collide with each other and do not collide with
          /// other categories.
          unsigned short mCollisionCategoryBits;

          /// Bits mask used to state which collision categories this shape can
          /// collide with. This value is 0xFFFF by default. It means that this
          /// proxy shape will collide with every collision categories by default.
          unsigned short mCollideWithMaskBits;




          // -------------------- Methods -------------------- //

          /// Private copy-constructor
          rpProxyShape(const rpProxyShape& proxyShape);

          /// Private assignment operator
          rpProxyShape& operator=(const rpProxyShape& proxyShape);




      public:



          Vector3 getRelativisticTransformLorentzBoost( const Vector3& point ) const
          {
        	  Vector3 temp = (getLocalToBodyTransform() * point);
        	  temp = mBody->mRelativityMotion.getLorentzMatrix() * (mBody->mTransform.getBasis() * temp);
        	  return (temp + mBody->mTransform.getPosition());
          }


          Vector3 supportWorldTransformed(const Vector3 &direction ) const
          {
        	  Matrix3x3 m = getWorldTransform().getBasis().getTranspose();

        	  Vector3    antiRotDirect = m * direction;
        	  Vector3         spVertex = mCollisionShape->getLocalSupportPointWithMarginn(antiRotDirect);
        	  return getRelativisticTransformLorentzBoost(  /*(getWorldTransform() */ spVertex );
          }



          void getIntervalWorld(const Vector3& xAxis, scalar& min, scalar& max) const
          {
        	  Vector3 s_p0 = supportWorldTransformed( xAxis );
        	  Vector3 s_p1 = supportWorldTransformed(-xAxis );
        	  min = s_p1.dot(xAxis);
        	  max = s_p0.dot(xAxis);
          }



          /*****************************************************************
           * Method taken from the site :
           * http://www.gamedev.ru/code/articles/convex_collisions
           *****************************************************************/
          Vector3* getAxisPeturberationPoints( const Vector3& xAxis , uint &_NbPoints) const
          {

        	  _NbPoints = 0;

        	  const scalar OFF_SET_COLLISION_CONTACT = scalar(0.02);

        	  Matrix3x3 InverseRotate = getWorldTransform().getBasis().getTranspose();
        	  Vector3 axis = InverseRotate * xAxis;

        	  Vector3 n0, n1;
        	  Vector3::btPlaneSpace1(axis , n0 , n1 );

        	  uint   NbCount = mCollisionShape->mNbMaxPeturberationIteration + 1;
        	  scalar epsilon = mCollisionShape->mEpsilonPeturberation;//0.077f;

        	  Vector3 startPoint;
        	  Vector3 OldPoint;



              Vector3* result = NULL;
              uint &num_element = _NbPoints;
        	  for( uint i = 0; i < NbCount; i++ )
        	  {
                  scalar ang = (2.0f * Pi() / scalar(float(NbCount))) * scalar(float(i));

                  Vector3 auxAxis = (axis + n0 * Cos(ang) * epsilon
                                          + n1 * Sin(ang) * epsilon);


        		  /***************************************************************/

        		  Vector3 sp = mCollisionShape->getLocalSupportPointWithMarginn( auxAxis );
        		  Vector3 spVertex =  getRelativisticTransformLorentzBoost( /* (getWorldTransform() */ sp);


        		  /*-----------------------------------------------------*/
        		  if ((spVertex - OldPoint).length2() > OFF_SET_COLLISION_CONTACT || i == 0)
        		  {
        			  if ((spVertex - startPoint).length2() < OFF_SET_COLLISION_CONTACT && num_element > 0 ) break;


                      /***************************************/
                      num_element++;
                      Vector3* array = (Vector3*) realloc(result , num_element * sizeof(Vector3));
                      if (array != NULL)
                      {
                          result = array;
                          result[num_element - 1] = spVertex;
                      }
                      else
                      {
                          free(result);
                      }

        			  /***************************************/

        			  OldPoint = spVertex;
        			  if (num_element == 1) startPoint = spVertex;
        		  }
        		  /*-----------------------------------------------------*/
        	  }

              return result;
          }


          // -------------------- Methods -------------------- //

          /// Constructor
          rpProxyShape(rpCollisionBody* body , rpCollisionShape* shape ,
        		       const Transform& transform, scalar mass=0 );

          /// Destructor
          ~rpProxyShape();

          /// Return the collision shape
          const rpCollisionShape* getCollisionShape() const;

          /// Return the parent body
          rpCollisionBody* getBody() const;

          /// Return the mass of the collision shape
          scalar getMass() const;

          /// Return a pointer to the user data attached to this body
          void* getUserData() const;

          /// Attach user data to this body
          void setUserData(void* userData);


          /// Return the local to parent body transform
          const Transform& getLocalToBodyTransform() const;

          /// Set the local to parent body transform
          void setLocalToBodyTransform(const Transform& transform);

          /// Return the local to world transform
          const Transform getWorldTransform() const;


          /// Return true if a point is inside the collision shape
          bool testPointInside(const Vector3& worldPoint);

          /// Raycast method with feedback information
          bool raycast(const Ray& ray, RaycastInfo& raycastInfo);


          /// Return the collision bits mask
          unsigned short getCollideWithMaskBits() const;

          /// Set the collision bits mask
          void setCollideWithMaskBits(unsigned short collideWithMaskBits);

          /// Return the collision category bits
          unsigned short getCollisionCategoryBits() const;

          /// Set the collision category bits
          void setCollisionCategoryBits(unsigned short collisionCategoryBits);

          /// Return the next proxy shape in the linked list of proxy shapes
                rpProxyShape* getNext();

          /// Return the next proxy shape in the linked list of proxy shapes
          const rpProxyShape* getNext() const;

          /// Return the pointer to the cached collision data
          void** getCachedCollisionData();

          /// Return the local scaling vector of the collision shape
          Vector3 getLocalScaling() const;

          /// Set the local scaling vector of the collision shape
          virtual void setLocalScaling(const Vector3& scaling);






          // -------------------- Friendship -------------------- //
          friend class rpPhysicsObject;
          friend class rpRigidPhysicsBody;


          friend class rpOverlappingPair;
          friend class rpCollisionBody;
          friend class rpBroadPhaseAlgorithm;
          friend class rpDynamicAABBTree;
          friend class rpCollisionManager;
          friend class rpCollisionWorld;
          friend class rpDynamicsWorld;
          friend class rpConvexMeshShape;


          friend class EPAAlgorithm;
          friend class GJKAlgorithm;

  };

  // Return the pointer to the cached collision data
  SIMD_INLINE void** rpProxyShape::getCachedCollisionData()
  {
      return &mCachedCollisionData;
  }

  // Return the collision shape
  /**
   * @return Pointer to the internal collision shape
   */
  SIMD_INLINE const rpCollisionShape* rpProxyShape::getCollisionShape() const
  {
      return mCollisionShape;
  }



  // Return the parent body
  /**
   * @return Pointer to the parent body
   */
  SIMD_INLINE rpCollisionBody* rpProxyShape::getBody() const
  {
      return mBody;
  }



  // Return the mass of the collision shape
  /**
   * @return Mass of the collision shape (in kilograms)
   */
  SIMD_INLINE scalar rpProxyShape::getMass() const
  {
      return mMass;
  }

  // Return a pointer to the user data attached to this body
  /**
   * @return A pointer to the user data stored into the proxy shape
   */
  SIMD_INLINE void* rpProxyShape::getUserData() const
  {
      return mUserData;
  }

  // Attach user data to this body
  /**
   * @param userData Pointer to the user data you want to store within the proxy shape
   */
  SIMD_INLINE void rpProxyShape::setUserData(void* userData)
  {
      mUserData = userData;
  }

  // Return the local to parent body transform
  /**
   * @return The transformation that transforms the local-space of the collision shape
   *         to the local-space of the parent body
   */
  SIMD_INLINE const Transform& rpProxyShape::getLocalToBodyTransform() const
  {
      return mLocalToBodyTransform;
  }


  // Set the local to parent body transform
  SIMD_INLINE void rpProxyShape::setLocalToBodyTransform(const Transform& transform)
  {
      mLocalToBodyTransform = transform;
  }



  // Return the local to world transform
  /**
   * @return The transformation that transforms the local-space of the collision
   *         shape to the world-space
   */
  SIMD_INLINE const Transform rpProxyShape::getWorldTransform() const
  {
	  return mBody->mTransform * mLocalToBodyTransform;
  }



  // Return the next proxy shape in the linked list of proxy shapes
  /**
   * @return Pointer to the next proxy shape in the linked list of proxy shapes
   */
  SIMD_INLINE rpProxyShape* rpProxyShape::getNext()
  {
      return mNext;
  }

  // Return the next proxy shape in the linked list of proxy shapes
  /**
   * @return Pointer to the next proxy shape in the linked list of proxy shapes
   */
  SIMD_INLINE const rpProxyShape* rpProxyShape::getNext() const
  {
      return mNext;
  }

  // Return the collision category bits
  /**
   * @return The collision category bits mask of the proxy shape
   */
  SIMD_INLINE unsigned short rpProxyShape::getCollisionCategoryBits() const
  {
      return mCollisionCategoryBits;
  }

  // Set the collision category bits
  /**
   * @param collisionCategoryBits The collision category bits mask of the proxy shape
   */
  SIMD_INLINE void rpProxyShape::setCollisionCategoryBits(unsigned short collisionCategoryBits)
  {
      mCollisionCategoryBits = collisionCategoryBits;
  }

  // Return the collision bits mask
  /**
   * @return The bits mask that specifies with which collision category this shape will collide
   */
  SIMD_INLINE unsigned short rpProxyShape::getCollideWithMaskBits() const
  {
      return mCollideWithMaskBits;
  }

  // Set the collision bits mask
  /**
   * @param collideWithMaskBits The bits mask that specifies with which collision category this shape will collide
   */
  SIMD_INLINE void rpProxyShape::setCollideWithMaskBits(unsigned short collideWithMaskBits)
  {
      mCollideWithMaskBits = collideWithMaskBits;
  }

  // Return the local scaling vector of the collision shape
  /**
   * @return The local scaling vector
   */
  SIMD_INLINE Vector3 rpProxyShape::getLocalScaling() const
  {
      return mCollisionShape->getScaling();
  }



  // Set the local scaling vector of the collision shape
  /**
   * @param scaling The new local scaling vector
   */
  SIMD_INLINE void rpProxyShape::setLocalScaling(const Vector3& scaling)
  {

      // Set the local scaling of the collision shape
      mCollisionShape->setLocalScaling(scaling);
  }

} /* namespace real_physics */

#endif /* COLLISION_RPPROXYSHAPE_H_ */
