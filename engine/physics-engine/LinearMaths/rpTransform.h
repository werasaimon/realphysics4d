/*
 * rpTransform.h
 *
 *  Created on: 15 нояб. 2016 г.
 *      Author: wera
 */

#ifndef SOURCE_REAL_PHYSICS_LINEARMATHS_RPTRANSFORM_H_
#define SOURCE_REAL_PHYSICS_LINEARMATHS_RPTRANSFORM_H_



// Libraries
#include "rpMatrix3x3.h"
#include "rpMinkowskiVector4.h"
#include "rpVector3D.h"
#include "rpQuaternion.h"


namespace real_physics
{




template<class T> class  rpTransform
{


private:

  // -------------------- Attributes -------------------- //

  rpMatrix3x3<T>   mScale;
  rpVector3D<T>    mPosition;
  rpQuaternion<T>  mOrientation;



public:




  // Constructor
  rpTransform()
   : mPosition(rpVector3D<T>(0.0, 0.0, 0.0)), mOrientation(rpQuaternion<T>::identity()) , mScale(rpMatrix3x3<T>::identity())
  {

  }


  // Constructor
  rpTransform(const rpVector3D<T>& position, const rpMatrix3x3<T> orientation = rpMatrix3x3<T>::identity() , const rpMatrix3x3<T>& _scale = rpMatrix3x3<T>::identity())
  : mPosition(position) , mOrientation(rpQuaternion<T>(orientation)) , mScale(_scale)
  {

  }


  // Constructor
  rpTransform(const rpVector3D<T>& position, const rpQuaternion<T>& orientation = rpQuaternion<T>::identity() , const rpMatrix3x3<T>& _scale = rpMatrix3x3<T>::identity())
  : mPosition(position), mOrientation(orientation) , mScale(_scale)
  {

  }

  // Copy-constructor
  rpTransform(const rpTransform<T>& transform)
  : mPosition(transform.mPosition), mOrientation(transform.mOrientation) , mScale(transform.mScale)
  {

  }

  // Destructor
  ~rpTransform()
  {

  }




  /// Return the origin of the transform
  const rpVector3D<T>& getPosition() const;

  /// Set the origin of the transform
  void setPosition(const rpVector3D<T>& position);



  /// Return the orientation quaternion
  const rpQuaternion<T>& getOrientation() const;

  /// Set the rotation quaternion
  void setOrientation(const rpQuaternion<T>& orientation);



  /// Return the scale of the transform
  rpMatrix3x3<T> getScale() const;

  /// Return the scale of the transform
  void setScale(const rpMatrix3x3<T> &scale);



  /// Set the transform to the identity transform
  void setToIdentity();





  /// Set the transform from an OpenGL transform matrix
  void setFromOpenGL(T* openglMatrix);

  /// Get the OpenGL matrix of the transform
  void getOpenGLMatrix(T* openglMatrix) const;





  /// Return the inverse of the transform
  rpTransform<T> getInverse() const;



  /// Return the inverse of the Matrix
  rpMatrix3x3<T> getBasis() const;


  /**
  /// Return an interpolated transform
  static rpTransform<T> interpolateTransforms(const rpTransform<T>& oldTransform,
                                              const rpTransform<T>& newTransform,
                                              T interpolationFactor);
  /**/

  /// Return the identity transform
  static rpTransform<T> identity();



  //-----------------------------------------------------------------------------//

  /// Lorentz demission distance world
  void setCreateLorentzBoost( const rpVector3D<T> &dir , const T& v )
  {
      mScale = rpMatrix3x3<T>::LoretzBoostScale( dir , v ).getInverse();
  }

  //-----------------------------------------------------------------------------//

  /// Return the transformed vector
  rpVector3D<T> operator*(const rpVector3D<T>& vector) const;

  /// Operator of multiplication of a transform with another one
  rpTransform<T> operator*(const rpTransform<T>& transform2) const;

  /// Return true if the two transforms are equal
  bool operator==(const rpTransform<T>& transform2) const;

  /// Return true if the two transforms are different
  bool operator!=(const rpTransform<T>& transform2) const;

  /// Assignment operator
  rpTransform<T>& operator=(const rpTransform<T>& transform);



};

template<class T>
SIMD_INLINE const rpVector3D<T>& real_physics::rpTransform<T>::getPosition() const
{
  return mPosition;
}

template<class T>
SIMD_INLINE void real_physics::rpTransform<T>::setPosition(const rpVector3D<T>& position)
{
  mPosition = position;
}

template<class T>
SIMD_INLINE const rpQuaternion<T>& real_physics::rpTransform<T>::getOrientation() const
{
  return mOrientation;
}

template<class T>
SIMD_INLINE void real_physics::rpTransform<T>::setOrientation(const rpQuaternion<T>& orientation)
{
  mOrientation = orientation;
}


template<class T>
SIMD_INLINE rpMatrix3x3<T> rpTransform<T>::getScale() const
{
  return mScale;
}

template<class T>
SIMD_INLINE void rpTransform<T>::setScale(const rpMatrix3x3<T> &scale)
{
  mScale = scale;
}

template<class T>
SIMD_INLINE void real_physics::rpTransform<T>::setToIdentity()
{
  mScale       = rpMatrix3x3<T>::identity();
  mPosition    = rpVector3D<T>::ZERO;
  mOrientation = rpQuaternion<T>::identity();
}

template<class T>
SIMD_INLINE void real_physics::rpTransform<T>::setFromOpenGL(T* openglMatrix)
{
  rpMatrix3x3<T> matrix(openglMatrix[0], openglMatrix[4], openglMatrix[8],
                        openglMatrix[1], openglMatrix[5], openglMatrix[9],
                        openglMatrix[2], openglMatrix[6], openglMatrix[10]);
  mOrientation = rpQuaternion<T>(matrix);
  mPosition.setAllValues(openglMatrix[12],
                         openglMatrix[13],
                         openglMatrix[14]);
}





/**
template<class T>
SIMD_INLINE rpTransform<T> real_physics::rpTransform<T>::interpolateTransforms( const rpTransform<T>& oldTransform ,
                                                                              const rpTransform<T>& newTransform ,
                                                                              T interpolationFactor)
{

  rpVector3D<T> interPosition =  oldTransform.mPosition * (T(1.0) - interpolationFactor) +
          newTransform.mPosition * interpolationFactor;

  rpQuaternion<T> interOrientation = rpQuaternion<T>::slerp(oldTransform.mOrientation,
                                                            newTransform.mOrientation,
                                                            interpolationFactor);

  return rpTransform<T>(interPosition, interOrientation);
}
/**/


template<class T>
SIMD_INLINE rpTransform<T> real_physics::rpTransform<T>::identity()
{
  return rpTransform<T>(rpVector3D<T>(0, 0, 0), rpQuaternion<T>::identity());
}



template<class T>
SIMD_INLINE rpVector3D<T> real_physics::rpTransform<T>::operator *(const rpVector3D<T>& vector) const
{
  return  (getBasis() * vector) + mPosition;
}

template<class T>
SIMD_INLINE rpTransform<T> real_physics::rpTransform<T>::operator *(const rpTransform<T>& transform2) const
{
  return rpTransform<T>(mPosition + (getBasis() * transform2.mPosition) , mOrientation * transform2.mOrientation , mScale * transform2.mScale);
}

template<class T>
SIMD_INLINE bool real_physics::rpTransform<T>::operator ==(const rpTransform<T>& transform2) const
{
  return (mPosition == transform2.mPosition) && (mOrientation == transform2.mOrientation) && (mScale == transform2.mScale);
}

template<class T>
SIMD_INLINE bool real_physics::rpTransform<T>::operator !=(const rpTransform<T>& transform2) const
{
  return !(*this == transform2);
}


template<class T>
SIMD_INLINE  rpMatrix3x3<T> rpTransform<T>::getBasis() const
{
  return mScale * mOrientation.getMatrix();
}



template<class T>
SIMD_INLINE rpTransform<T>& real_physics::rpTransform<T>::operator =(const rpTransform<T>& transform)
{
  if (&transform != this)
  {
      mPosition    = transform.mPosition;
      mOrientation = transform.mOrientation;
      mScale       = transform.mScale;
  }
  return *this;
}



template<class T>
SIMD_INLINE rpTransform<T> real_physics::rpTransform<T>::getInverse() const
{
  const rpQuaternion<T> &invQuaternion = mOrientation.getInverse();
  const rpMatrix3x3<T>  &invMatrix     = invQuaternion.getMatrix();
  const rpMatrix3x3<T>  &invScale      = mScale.getInverse();
  return rpTransform<T>( (invScale * invMatrix * (-mPosition))  , invQuaternion , invScale );
}



template<class T>
SIMD_INLINE void real_physics::rpTransform<T>::getOpenGLMatrix( T* openglMatrix ) const
{

  const rpMatrix3x3<T>& matrix = getBasis();
  openglMatrix[0]  = matrix[0][0]; openglMatrix[1]  = matrix[1][0];
  openglMatrix[2]  = matrix[2][0]; openglMatrix[3]  = 0.0;
  openglMatrix[4]  = matrix[0][1]; openglMatrix[5]  = matrix[1][1];
  openglMatrix[6]  = matrix[2][1]; openglMatrix[7]  = 0.0;
  openglMatrix[8]  = matrix[0][2]; openglMatrix[9]  = matrix[1][2];
  openglMatrix[10] = matrix[2][2]; openglMatrix[11] = 0.0;
  openglMatrix[12] = mPosition.x;  openglMatrix[13] = mPosition.y;
  openglMatrix[14] = mPosition.z;  openglMatrix[15] = 1.0;

}





} /* namespace real_physics */



#endif /* SOURCE_REAL_PHYSICS_LINEARMATHS_RPTRANSFORM_H_ */
