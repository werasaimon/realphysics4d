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

    rpVector3D<T>   mPosition;
    rpQuaternion<T> mOrientation;



  public:




    // Constructor
    rpTransform()
    : mPosition(rpVector3D<T>(0.0, 0.0, 0.0)), mOrientation(rpQuaternion<T>::identity())
    {

    }


    // Constructor
    rpTransform(const rpVector3D<T>& position, const rpMatrix3x3<T> orientation)
    : mPosition(position) , mOrientation(rpQuaternion<T>(orientation))
    {

    }


    // Constructor
    rpTransform(const rpVector3D<T>& position, const rpQuaternion<T>& orientation)
    : mPosition(position), mOrientation(orientation)
    {

    }

    // Copy-constructor
    rpTransform(const rpTransform<T>& transform)
    : mPosition(transform.mPosition), mOrientation(transform.mOrientation)
    {

    }

    // Destructor
    ~rpTransform()
    {

    }


    //-------------------------------------------------------------------------//


    /**@brief Return the inverse of this transform times the other transform
      * @param t The other transform
      * return this.inverse() * the other */
    rpTransform<T> inverseTimes(const rpTransform<T>& t) const;

    //-------------------------------------------------------------------------//



    /// Return the origin of the transform
    const rpVector3D<T>& getPosition() const;

    /// Set the origin of the transform
    void setPosition(const rpVector3D<T>& position);



    /// Return the orientation quaternion
    const rpQuaternion<T>& getOrientation() const;

    /// Set the rotation quaternion
    void setOrientation(const rpQuaternion<T>& orientation);


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


    /// Return an interpolated transform
    static rpTransform<T> interpolateTransforms(const rpTransform<T>& oldTransform,
						                        const rpTransform<T>& newTransform,
					                        	T interpolationFactor);

    /// Return the identity transform
    static rpTransform<T> identity();


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
  SIMD_INLINE void real_physics::rpTransform<T>::setToIdentity()
  {
    mPosition    = rpVector3D<T>(0.0, 0.0, 0.0);
    mOrientation = rpQuaternion<T>::identity();
  }

  template<class T>
  SIMD_INLINE void real_physics::rpTransform<T>::setFromOpenGL(T* openglMatrix)
  {
    rpMatrix3x3<T> matrix(openglMatrix[0], openglMatrix[4], openglMatrix[8],
						  openglMatrix[1], openglMatrix[5], openglMatrix[9],
						  openglMatrix[2], openglMatrix[6], openglMatrix[10]);
    mOrientation = rpQuaternion<T>(matrix);
    mPosition.setAllValues(openglMatrix[12], openglMatrix[13], openglMatrix[14]);
  }






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



  template<class T>
  SIMD_INLINE rpTransform<T> real_physics::rpTransform<T>::identity()
  {
    return rpTransform<T>(rpVector3D<T>(0, 0, 0), rpQuaternion<T>::identity());
  }



  template<class T>
   SIMD_INLINE rpVector3D<T> real_physics::rpTransform<T>::operator *(const rpVector3D<T>& vector) const
   {
     return (mOrientation.getMatrix() * vector) + mPosition;
   }

   template<class T>
   SIMD_INLINE rpTransform<T> real_physics::rpTransform<T>::operator *(const rpTransform<T>& transform2) const
   {
     return rpTransform<T>(mPosition + mOrientation.getMatrix() * transform2.mPosition,
 			               mOrientation * transform2.mOrientation);
   }

  template<class T>
  SIMD_INLINE bool real_physics::rpTransform<T>::operator ==(const rpTransform<T>& transform2) const
  {
    return (mPosition == transform2.mPosition) && (mOrientation == transform2.mOrientation);
  }

  template<class T>
  SIMD_INLINE bool real_physics::rpTransform<T>::operator !=(const rpTransform<T>& transform2) const
  {
    return !(*this == transform2);
  }

template<class T>
SIMD_INLINE rpTransform<T> rpTransform<T>::inverseTimes(const rpTransform<T>& t) const
{

	rpMatrix3x3<T> mOrientationMatrix = mOrientation.getMatrix();
	rpMatrix3x3<T> Matrix = t.mOrientation.getMatrix();
	rpVector3D<T> v = t.getPosition() - mPosition;
    return rpTransform<T>( mOrientationMatrix * v  , mOrientationMatrix.transposeTimes(Matrix) );
}

template<class T>
SIMD_INLINE  rpMatrix3x3<T> rpTransform<T>::getBasis() const
{
	return mOrientation.getMatrix();
}



template<class T>
SIMD_INLINE rpTransform<T>& real_physics::rpTransform<T>::operator =(const rpTransform<T>& transform)
{
	if (&transform != this)
	{
		mPosition = transform.mPosition;
		mOrientation = transform.mOrientation;
	}
	return *this;
}



template<class T>
SIMD_INLINE rpTransform<T> real_physics::rpTransform<T>::getInverse() const
{
	const rpQuaternion<T>& invQuaternion = mOrientation.getInverse();
	rpMatrix3x3<T> invMatrix = invQuaternion.getMatrix();
	return rpTransform<T>(invMatrix * (-mPosition), invQuaternion);
}



template<class T>
SIMD_INLINE void real_physics::rpTransform<T>::getOpenGLMatrix( T* openglMatrix ) const
{

  const rpMatrix3x3<T>& matrix = mOrientation.getMatrix();// * rpMatrix3x3<T>::relativityBoost(vel/60.0);//.getTranspose();
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
