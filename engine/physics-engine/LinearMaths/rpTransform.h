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

 // T                mTime
 // rpVector3D<T>    mPosition;


  rpMinkowskiVector4<T> mPosition4;
  rpQuaternion<T>       mOrientation;
  rpMatrix3x3<T>        mScale;

public:




  // Constructor
  rpTransform()
   : mPosition4(0.0,0.0,0.0,1.0) , mOrientation(rpQuaternion<T>::identity()) , mScale(rpMatrix3x3<T>::identity())
  {

  }


  // Constructor
  rpTransform(const rpMinkowskiVector4<T>& position, const rpMatrix3x3<T> orientation = rpMatrix3x3<T>::identity() , const rpMatrix3x3<T>& _scale = rpMatrix3x3<T>::identity())
  :  mPosition4(position) ,  mOrientation(rpQuaternion<T>(orientation)) , mScale(_scale)
  {

  }

  // Constructor
  rpTransform(const rpMinkowskiVector4<T>& position, const rpQuaternion<T>& orientation = rpQuaternion<T>::identity() , const rpMatrix3x3<T>& _scale = rpMatrix3x3<T>::identity())
  : mPosition4(position) , mOrientation(orientation) , mScale(_scale)
  {

  }

  // Constructor
  rpTransform(const rpVector3D<T>& position, const rpMatrix3x3<T> orientation = rpMatrix3x3<T>::identity() , const rpMatrix3x3<T>& _scale = rpMatrix3x3<T>::identity() ,T  _time = 1.0)
  : mPosition4(position,_time) , mOrientation(rpQuaternion<T>(orientation)) , mScale(_scale)
  {

  }


  // Constructor
  rpTransform(const rpVector3D<T>& position, const rpQuaternion<T>& orientation = rpQuaternion<T>::identity() , const rpMatrix3x3<T>& _scale = rpMatrix3x3<T>::identity() , T _time = 1.0)
  : mPosition4(position,_time) , mOrientation(orientation) , mScale(_scale)
  {

  }

  // Copy-constructor
  rpTransform(const rpTransform<T>& transform)
  : mPosition4(transform.mPosition4) , mOrientation(transform.mOrientation) , mScale(transform.mScale)
  {

  }

  // Destructor
  ~rpTransform()
  {

  }


  const rpMinkowskiVector4<T> getPosition4() const;

  void setPosition4(const rpMinkowskiVector4<T> &position4);



//  /// Return the origin of the transform
//  const rpVector3D<T>& getPosition() const;

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




  /// Return time step
  T getTime() const;

 /// Set the time step
  void setTime(const T &time);




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





  /// Return the identity transform
  static rpTransform<T> identity();



  //-----------------------------------------------------------------------------//

  /// Lorentz demission distance in the world
  void BuildLorentzBoost( const rpVector3D<T> &dir , const T& v , T *_gamma = NULL )
  {
      mScale = rpMatrix3x3<T>::getLorentzBoost( dir , v , _gamma);
  }


  /// Lorentz demission distance world
  void BuildLorentzBoostTesting(  const rpVector3D<T> &dir    , const T& v  ,
                                  const rpVector3D<T> &w_axis , const T& wv  )
  {

      /// Loretz factor gamma ^ -1
      T gamma_w = sqrt( 1.0 - (wv*wv) / (_c*_c));
      T gamma_v = sqrt( 1.0 -   (v*v) / (_c*_c));


      /// Scaling direction
      rpMatrix3x3<T> M_dir  = rpMatrix3x3<T>::getLorentzBoost( dir , v , &gamma_v);

      if( w_axis.length2() > 0.001)
      {
          /// Ortogonal axis of the angular velocity axis
          rpVector3D<T>  Up;
          rpVector3D<T>  Left;
          rpVector3D<T>::btPlaneSpace1( w_axis , Up , Left );

          /// Scaling Loretz group operators
          rpMatrix3x3<T> M_up   = rpMatrix3x3<T>::getLorentzBoost( Up.getUnit()   , wv , &gamma_w);
          rpMatrix3x3<T> M_left = rpMatrix3x3<T>::getLorentzBoost( Left.getUnit() , wv , &gamma_w);

          /// group Li SO(0,3)
          mScale = M_dir * M_up * M_left;
      }
      else
      {
          /// group Li SU(0,1)
          mScale = M_dir;
      }

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
SIMD_INLINE const rpMinkowskiVector4<T> rpTransform<T>::getPosition4() const
{
    return mPosition4;
}

template<class T>
SIMD_INLINE void rpTransform<T>::setPosition4(const rpMinkowskiVector4<T> &position4)
{
    mPosition4 = position4;
}






//template<class T>
//SIMD_INLINE const rpVector3D<T>& rpTransform<T>::getPosition() const
//{
//  return mPosition;
//}

template<class T>
SIMD_INLINE void rpTransform<T>::setPosition(const rpVector3D<T>& position)
{
  mPosition4 = rpMinkowskiVector4<T>( position , mPosition4.t );
}





template<class T>
SIMD_INLINE const rpQuaternion<T>& rpTransform<T>::getOrientation() const
{
  return mOrientation;
}

template<class T>
SIMD_INLINE void rpTransform<T>::setOrientation(const rpQuaternion<T>& orientation)
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
SIMD_INLINE T rpTransform<T>::getTime() const
{
    return mPosition4.t;
}

template<class T>
SIMD_INLINE void rpTransform<T>::setTime(const T &time)
{
   mPosition4.t = time;
}


template<class T>
SIMD_INLINE void rpTransform<T>::setToIdentity()
{
  mScale       = rpMatrix3x3<T>::identity();
  mOrientation = rpQuaternion<T>::identity();
  mPosition4   = rpMinkowskiVector4<T>( rpVector3D<T>::ZERO , T(1.0) );
}

template<class T>
SIMD_INLINE void rpTransform<T>::setFromOpenGL(T* openglMatrix)
{
  rpMatrix3x3<T> matrix(openglMatrix[0], openglMatrix[4], openglMatrix[8],
                        openglMatrix[1], openglMatrix[5], openglMatrix[9],
                        openglMatrix[2], openglMatrix[6], openglMatrix[10]);
  mOrientation = rpQuaternion<T>(matrix);

 rpVector3D<T> pos( openglMatrix[12],
                    openglMatrix[13],
                    openglMatrix[14]);

  mPosition4 = rpMinkowskiVector4<T>( pos , mPosition4.t );
}





template<class T>
SIMD_INLINE rpTransform<T> rpTransform<T>::identity()
{
  return rpTransform<T>(rpVector3D<T>(0, 0, 0), rpQuaternion<T>::identity());
}



template<class T>
SIMD_INLINE rpVector3D<T> rpTransform<T>::operator * (const rpVector3D<T>& vector) const
{
  return  (getBasis() * vector) + mPosition4.getPos();
}

template<class T>
SIMD_INLINE rpTransform<T> rpTransform<T>::operator * (const rpTransform<T>& transform2) const
{
  return rpTransform<T>(mPosition4.getPos() + (getBasis() * transform2.mPosition4.getPos()) , mOrientation * transform2.mOrientation , mScale * transform2.mScale , mPosition4.t * transform2.mPosition4.t);
}

template<class T>
SIMD_INLINE bool rpTransform<T>::operator ==(const rpTransform<T>& transform2) const
{
  return  (mOrientation == transform2.mOrientation) &&
          (mScale       == transform2.mScale)       &&
          (mPosition4   == transform2.mPosition4);
}

template<class T>
SIMD_INLINE bool rpTransform<T>::operator !=(const rpTransform<T>& transform2) const
{
  return !(*this == transform2);
}


template<class T>
SIMD_INLINE  rpMatrix3x3<T> rpTransform<T>::getBasis() const
{
  return mScale * mOrientation.getMatrix();
}



template<class T>
SIMD_INLINE rpTransform<T>& rpTransform<T>::operator =(const rpTransform<T>& transform)
{
  if (&transform != this)
  {
      mPosition4   = transform.mPosition4;
      mOrientation = transform.mOrientation;
      mScale       = transform.mScale;
  }
  return *this;
}



template<class T>
SIMD_INLINE rpTransform<T> rpTransform<T>::getInverse() const
{
  const rpQuaternion<T> &invQuaternion = mOrientation.getInverse();
  const rpMatrix3x3<T>  &invMatrix     = invQuaternion.getMatrix();
  const rpMatrix3x3<T>  &invScale      = mScale.getInverse();
  return rpTransform<T>( (invScale * invMatrix * (-mPosition4.getPos()))  , invQuaternion , invScale , T(1.0) / mPosition4.t);
}



template<class T>
SIMD_INLINE void rpTransform<T>::getOpenGLMatrix( T* openglMatrix ) const
{

  const rpMatrix3x3<T>& matrix = getBasis();
  openglMatrix[0]  = matrix[0][0]; openglMatrix[1]  = matrix[1][0];
  openglMatrix[2]  = matrix[2][0]; openglMatrix[3]  = 0.0;
  openglMatrix[4]  = matrix[0][1]; openglMatrix[5]  = matrix[1][1];
  openglMatrix[6]  = matrix[2][1]; openglMatrix[7]  = 0.0;
  openglMatrix[8]  = matrix[0][2]; openglMatrix[9]  = matrix[1][2];
  openglMatrix[10] = matrix[2][2]; openglMatrix[11] = 0.0;

  openglMatrix[12] = mPosition4.x;
  openglMatrix[13] = mPosition4.y;
  openglMatrix[14] = mPosition4.z;
  openglMatrix[15] = 1.0;

}





} /* namespace real_physics */



#endif /* SOURCE_REAL_PHYSICS_LINEARMATHS_RPTRANSFORM_H_ */
