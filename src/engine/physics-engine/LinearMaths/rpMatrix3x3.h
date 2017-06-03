/*
 * rpMatrix3x3.h
 *
 *  Created on: 15 нояб. 2016 г.
 *      Author: wera
 */

#ifndef SOURCE_REAL_PHYSICS_LINEARMATHS_RPMATRIX3X3_H_
#define SOURCE_REAL_PHYSICS_LINEARMATHS_RPMATRIX3X3_H_


// Libraries
#include <cassert>
#include "rpVector3D.h"
#include "../config.h"


namespace real_physics
{

template<class T> class rpQuaternion;


template<class T> class rpMatrix3x3
{

   private:

     // -------------------- Attributes -------------------- //

     /// Rows of the matrix;
     rpVector3D<T> mRows[3];


    public:

  	enum tMatAxis
  	{
          eDir = 2, eUp = 1, eSide = 0,
            eX = 0,  eY = 1,    eZ = 2
  	};

      // Constructor of the class Matrix3x3
     rpMatrix3x3()
     {
    	  // Initialize all values in the matrix to zero
    	  setAllValues(0.0, 0.0, 0.0,
                	   0.0, 0.0, 0.0,
				       0.0, 0.0, 0.0);
     }

      // Constructor
      rpMatrix3x3(T value)
      {
    	  setAllValues(value, value, value,
    			       value, value, value,
				       value, value, value);
      }

      // Constructor with arguments
      rpMatrix3x3(T a1, T a2, T a3,
    		      T b1, T b2, T b3,
			      T c1, T c2, T c3)
      {
    	  // Initialize the matrix with the values
    	    	  setAllValues(a1, a2, a3,
    						   b1, b2, b3,
    						   c1, c2, c3);
      }


      // Constructor with arguments
      rpMatrix3x3( T data[3][3] )
      {
	      setAllValues(data[0][0], data[0][1], data[0][2],
				       data[1][0], data[1][1], data[1][2],
				       data[2][0], data[2][1], data[2][2]);
      }


      // Copy-constructor
      rpMatrix3x3(const rpMatrix3x3<T>& matrix)
      {
	      setAllValues(matrix.mRows[0][0], matrix.mRows[0][1], matrix.mRows[0][2],
				       matrix.mRows[1][0], matrix.mRows[1][1], matrix.mRows[1][2],
				       matrix.mRows[2][0], matrix.mRows[2][1], matrix.mRows[2][2]);
      }


      // Destructor
      virtual ~rpMatrix3x3()
      {

      }




      /// Assignment operator
      rpMatrix3x3<T>& operator = (const rpMatrix3x3<T>& matrix)
      {
    	    // Check for self-assignment
    	    if (&matrix != this)
    	    {
                setAllValues(matrix.mRows[0][0], matrix.mRows[0][1], matrix.mRows[0][2],
    	                     matrix.mRows[1][0], matrix.mRows[1][1], matrix.mRows[1][2],
    	                     matrix.mRows[2][0], matrix.mRows[2][1], matrix.mRows[2][2]);
    	    }
    	    return *this;
      }


//      /// Assignment operator
//      const rpMatrix3x3<T>& operator = (const rpMatrix3x3<T>& matrix);


      /// Set all the values in the matrix
      void setAllValues(T a1, T a2, T a3,
						T b1, T b2, T b3,
						T c1, T c2, T c3);


      //----------------------------------------------------------------//


      /// Set the matrix to zero
      void setToZero();

      /// Return a column
      rpVector3D<T> getColumn(int i) const;

      /// Return a row
      rpVector3D<T> getRow(int i) const;


      /// Get an axis direction from the Matrix3x3
      rpVector3D<T> getAxis(tMatAxis Axis) const
      {
          return getColumn(Axis);
      }




      rpVector3D<T> getEulerAngle();


      //---------------------------------------------------------------//

      rpMatrix3x3<T> transposeTimes(const rpMatrix3x3<T>& m) const;


      /// The rotation matrix from the
      /// building system to the inertial system has the form:
      rpMatrix3x3<T> rotateToInertialSystemR(const rpVector3D<T>& euler) const;
      rpMatrix3x3<T> TransitionMatrixR(const rpVector3D<T>& euler) const;



      //---------------------------------------------------------------//


      /// Return the transpose matrix
      rpMatrix3x3<T> getTranspose() const;

      /// Return the determinant of the matrix
      T getDeterminant() const;

      /// Return the trace of the matrix
      T getTrace() const;

      /// Return the inverse matrix
      rpMatrix3x3<T> getInverse() const;

      /// Return the matrix with absolute values
      rpMatrix3x3<T> getAbsoluteMatrix() const;



      T *getData();


      /// Set the matrix to the identity matrix
      void setToIdentity();

      /// Return the 3x3 identity matrix
      static rpMatrix3x3<T> identity();

      /// Return the 3x3 zero matrix
      static rpMatrix3x3<T> zero();



      /// Return a skew-symmetric matrix using a given vector that can be used
      /// to compute cross product with another vector using matrix multiplication
      static rpMatrix3x3<T> computeSkewSymmetricMatrixForCrossProduct(const rpVector3D<T>& vector);


      /// Return a  matrix using a given vector that can be used
      /// to compute dot product with another vector using matrix multiplication
      static rpMatrix3x3<T> MatrixTensorProduct( const rpVector3D<T>& vector1 , const rpVector3D<T>& vector2 );



      //---------------------------- Friend method -------------------------//

      /// Overloaded operator for addition
      friend rpMatrix3x3<T>  operator+(const rpMatrix3x3<T> & matrix1, const rpMatrix3x3<T> & matrix2)
      {
    	   return rpMatrix3x3<T>(matrix1.mRows[0][0] + matrix2.mRows[0][0],  matrix1.mRows[0][1] +
    	                         matrix2.mRows[0][1],  matrix1.mRows[0][2] + matrix2.mRows[0][2],
    	                         matrix1.mRows[1][0] + matrix2.mRows[1][0],  matrix1.mRows[1][1] +
    	                         matrix2.mRows[1][1],  matrix1.mRows[1][2] + matrix2.mRows[1][2],
    	                         matrix1.mRows[2][0] + matrix2.mRows[2][0],  matrix1.mRows[2][1] +
    	                         matrix2.mRows[2][1],  matrix1.mRows[2][2] + matrix2.mRows[2][2]);
      }

      /// Overloaded operator for substraction
      friend rpMatrix3x3<T>  operator-(const rpMatrix3x3<T> & matrix1, const rpMatrix3x3<T> & matrix2)
      {


    	  return rpMatrix3x3<T>(matrix1.mRows[0][0] - matrix2.mRows[0][0],  matrix1.mRows[0][1] -
    	                        matrix2.mRows[0][1],  matrix1.mRows[0][2] - matrix2.mRows[0][2],
    	                        matrix1.mRows[1][0] - matrix2.mRows[1][0],  matrix1.mRows[1][1] -
    	                        matrix2.mRows[1][1],  matrix1.mRows[1][2] - matrix2.mRows[1][2],
    	                        matrix1.mRows[2][0] - matrix2.mRows[2][0],  matrix1.mRows[2][1] -
    	                        matrix2.mRows[2][1],  matrix1.mRows[2][2] - matrix2.mRows[2][2]);
      }

      /// Overloaded operator for the negative of the matrix
      friend rpMatrix3x3<T>  operator-(const rpMatrix3x3<T> & matrix)
      {
    	  return rpMatrix3x3<T>(-matrix.mRows[0][0], -matrix.mRows[0][1], -matrix.mRows[0][2],
    	                        -matrix.mRows[1][0], -matrix.mRows[1][1], -matrix.mRows[1][2],
    	                        -matrix.mRows[2][0], -matrix.mRows[2][1], -matrix.mRows[2][2]);
      }

      /// Overloaded operator for multiplication with a number
      friend rpMatrix3x3<T>  operator*(T nb, const rpMatrix3x3<T>& matrix)
      {
          return rpMatrix3x3<T>(matrix.mRows[0][0] * nb, matrix.mRows[0][1] * nb, matrix.mRows[0][2] * nb,
    	                        matrix.mRows[1][0] * nb, matrix.mRows[1][1] * nb, matrix.mRows[1][2] * nb,
    	                        matrix.mRows[2][0] * nb, matrix.mRows[2][1] * nb, matrix.mRows[2][2] * nb);
      }

      /// Overloaded operator for multiplication with a matrix
      friend rpMatrix3x3<T>  operator*(const rpMatrix3x3<T>& matrix, T nb)
      {
    	  return nb * matrix;
      }



      /// Overloaded operator for matrix multiplication
      friend rpMatrix3x3<T>  operator * (const rpMatrix3x3<T>& matrix1, const rpMatrix3x3<T>& matrix2)
      {

    	  rpMatrix3x3<T> result;
    	  result.setAllValues(    matrix1.mRows[0][0]*  matrix2.mRows[0][0] + matrix1.mRows[0][1] *
								  matrix2.mRows[1][0] + matrix1.mRows[0][2]*  matrix2.mRows[2][0],
								  matrix1.mRows[0][0]*  matrix2.mRows[0][1] + matrix1.mRows[0][1] *
								  matrix2.mRows[1][1] + matrix1.mRows[0][2]*  matrix2.mRows[2][1],
								  matrix1.mRows[0][0]*  matrix2.mRows[0][2] + matrix1.mRows[0][1] *
								  matrix2.mRows[1][2] + matrix1.mRows[0][2]*  matrix2.mRows[2][2],
								  matrix1.mRows[1][0]*  matrix2.mRows[0][0] + matrix1.mRows[1][1] *
								  matrix2.mRows[1][0] + matrix1.mRows[1][2]*  matrix2.mRows[2][0],
								  matrix1.mRows[1][0]*  matrix2.mRows[0][1] + matrix1.mRows[1][1] *
								  matrix2.mRows[1][1] + matrix1.mRows[1][2]*  matrix2.mRows[2][1],
								  matrix1.mRows[1][0]*  matrix2.mRows[0][2] + matrix1.mRows[1][1] *
								  matrix2.mRows[1][2] + matrix1.mRows[1][2]*  matrix2.mRows[2][2],
								  matrix1.mRows[2][0]*  matrix2.mRows[0][0] + matrix1.mRows[2][1] *
								  matrix2.mRows[1][0] + matrix1.mRows[2][2]*  matrix2.mRows[2][0],
								  matrix1.mRows[2][0]*  matrix2.mRows[0][1] + matrix1.mRows[2][1] *
								  matrix2.mRows[1][1] + matrix1.mRows[2][2]*  matrix2.mRows[2][1],
								  matrix1.mRows[2][0]*  matrix2.mRows[0][2] + matrix1.mRows[2][1] *
								  matrix2.mRows[1][2] + matrix1.mRows[2][2]*  matrix2.mRows[2][2]);

    	  return result;
      }


      /// Overloaded operator for matrix multiplication cross
      friend rpMatrix3x3<T>  operator^(const rpMatrix3x3<T>& matrix1, const rpMatrix3x3<T>& matrix2)
      {
    	  return matrix1 * matrix2.getTranspose();
      }






      /// Overloaded operator for multiplication with a vector
      friend rpVector3D<T> operator*(const rpMatrix3x3<T>& matrix, const rpVector3D<T>& vector)
      {

    	  return rpVector3D<T>(matrix.mRows[0][0]*vector.x +
    			               matrix.mRows[0][1]*vector.y +
    	                       matrix.mRows[0][2]*vector.z,
    	                       matrix.mRows[1][0]*vector.x +
							   matrix.mRows[1][1]*vector.y +
    	                       matrix.mRows[1][2]*vector.z,
    	                       matrix.mRows[2][0]*vector.x +
							   matrix.mRows[2][1]*vector.y +
    	                       matrix.mRows[2][2]*vector.z);
      }



      //--------------------------------------------------------------------//


      /// Overloaded operator for equality condition
      bool operator == (const rpMatrix3x3<T>& matrix) const;

      /// Overloaded operator for the is different condition
      bool operator != (const rpMatrix3x3<T>& matrix) const;




      /// Overloaded operator for addition with assignment
      rpMatrix3x3<T>& operator+=(const rpMatrix3x3<T>& matrix);

      /// Overloaded operator for substraction with assignment
      rpMatrix3x3<T>& operator-=(const rpMatrix3x3<T>& matrix);

      /// Overloaded operator for multiplication with a number with assignment
      rpMatrix3x3<T>& operator*=(T nb);




      /// Overloaded operator to read element of the matrix.
      const rpVector3D<T>& operator[](int row) const;

      /// Overloaded operator to read/write element of the matrix.
      rpVector3D<T>& operator[](int row);

};

template<class T>
SIMD_INLINE void real_physics::rpMatrix3x3<T>::setAllValues(T a1, T a2, T a3,
															T b1, T b2, T b3,
															T c1, T c2, T c3)
{
  mRows[0][0] = a1; mRows[0][1] = a2; mRows[0][2] = a3;
  mRows[1][0] = b1; mRows[1][1] = b2; mRows[1][2] = b3;
  mRows[2][0] = c1; mRows[2][1] = c2; mRows[2][2] = c3;

}

template<class T>
SIMD_INLINE void real_physics::rpMatrix3x3<T>::setToZero()
{
  mRows[0].setToZero();
  mRows[1].setToZero();
  mRows[2].setToZero();
}

template<class T>
SIMD_INLINE rpVector3D<T> real_physics::rpMatrix3x3<T>::getColumn(int i) const
{
  assert(i>= 0 && i<3);
  return rpVector3D<T> (mRows[0][i], mRows[1][i], mRows[2][i]);
}

template<class T>
SIMD_INLINE rpVector3D<T> real_physics::rpMatrix3x3<T>::getRow(int i) const
{
  assert(i>= 0 && i<3);
  return mRows[i];
}



template<class T>
SIMD_INLINE T real_physics::rpMatrix3x3<T>::getDeterminant() const
{
  // Compute and return the determinant of the matrix
  return (mRows[0][0]*(mRows[1][1]*mRows[2][2]-mRows[2][1]*mRows[1][2]) -
          mRows[0][1]*(mRows[1][0]*mRows[2][2]-mRows[2][0]*mRows[1][2]) +
          mRows[0][2]*(mRows[1][0]*mRows[2][1]-mRows[2][0]*mRows[1][1]));
}

template<class T>
SIMD_INLINE T real_physics::rpMatrix3x3<T>::getTrace() const
{
  // Compute and return the trace
  return (mRows[0][0] + mRows[1][1] + mRows[2][2]);
}


template<class T>
SIMD_INLINE rpMatrix3x3<T> real_physics::rpMatrix3x3<T>::getAbsoluteMatrix() const
{
  return rpMatrix3x3<T>(fabs(mRows[0][0]), fabs(mRows[0][1]), fabs(mRows[0][2]),
			            fabs(mRows[1][0]), fabs(mRows[1][1]), fabs(mRows[1][2]),
			            fabs(mRows[2][0]), fabs(mRows[2][1]), fabs(mRows[2][2]));
}


template<class T>
SIMD_INLINE rpMatrix3x3<T> real_physics::rpMatrix3x3<T>::getTranspose() const
{
  // Return the transpose matrix
	return rpMatrix3x3<T>(mRows[0][0], mRows[1][0], mRows[2][0],
		                  mRows[0][1], mRows[1][1], mRows[2][1],
		                  mRows[0][2], mRows[1][2], mRows[2][2]);

}



template<class T>
SIMD_INLINE void real_physics::rpMatrix3x3<T>::setToIdentity()
{
  mRows[0][0] = 1.0; mRows[0][1] = 0.0; mRows[0][2] = 0.0;
  mRows[1][0] = 0.0; mRows[1][1] = 1.0; mRows[1][2] = 0.0;
  mRows[2][0] = 0.0; mRows[2][1] = 0.0; mRows[2][2] = 1.0;
}

template<class T>
SIMD_INLINE rpMatrix3x3<T> real_physics::rpMatrix3x3<T>::identity()
{
  return rpMatrix3x3<T>(1.0, 0.0, 0.0,
			            0.0, 1.0, 0.0,
			            0.0, 0.0, 1.0);
}

template<class T>
SIMD_INLINE rpMatrix3x3<T> real_physics::rpMatrix3x3<T>::zero()
{
  return rpMatrix3x3<T>(0.0, 0.0, 0.0,
			            0.0, 0.0, 0.0,
			            0.0, 0.0, 0.0);
}


// Return a skew-symmetric matrix using a given vector that can be used
// to compute cross product with another vector using matrix multiplication
template<class T>
SIMD_INLINE rpMatrix3x3<T> real_physics::rpMatrix3x3<T>::computeSkewSymmetricMatrixForCrossProduct(const rpVector3D<T>& vector)
{
  return rpMatrix3x3<T>(0, -vector.z, vector.y,
			            vector.z, 0, -vector.x,
			           -vector.y, vector.x, 0);
}

template<class T>
SIMD_INLINE bool real_physics::rpMatrix3x3<T>::operator ==(const rpMatrix3x3<T>& matrix) const
{
  return (mRows[0][0] == matrix.mRows[0][0] && mRows[0][1] == matrix.mRows[0][1] &&
		  mRows[0][2] == matrix.mRows[0][2] &&
		  mRows[1][0] == matrix.mRows[1][0] && mRows[1][1] == matrix.mRows[1][1] &&
		  mRows[1][2] == matrix.mRows[1][2] &&
		  mRows[2][0] == matrix.mRows[2][0] && mRows[2][1] == matrix.mRows[2][1] &&
		  mRows[2][2] == matrix.mRows[2][2]);
}

template<class T>
SIMD_INLINE bool real_physics::rpMatrix3x3<T>::operator !=(const rpMatrix3x3<T>& matrix) const
{
  return !(*this == matrix);
}

template<class T>
SIMD_INLINE rpMatrix3x3<T>& real_physics::rpMatrix3x3<T>::operator +=(const rpMatrix3x3<T>& matrix)
{
  mRows[0][0] += matrix.mRows[0][0]; mRows[0][1] += matrix.mRows[0][1];
  mRows[0][2] += matrix.mRows[0][2]; mRows[1][0] += matrix.mRows[1][0];
  mRows[1][1] += matrix.mRows[1][1]; mRows[1][2] += matrix.mRows[1][2];
  mRows[2][0] += matrix.mRows[2][0]; mRows[2][1] += matrix.mRows[2][1];
  mRows[2][2] += matrix.mRows[2][2];
  return *this;
}

template<class T>
SIMD_INLINE rpMatrix3x3<T>& real_physics::rpMatrix3x3<T>::operator -=(const rpMatrix3x3<T>& matrix)
{
  mRows[0][0] -= matrix.mRows[0][0]; mRows[0][1] -= matrix.mRows[0][1];
  mRows[0][2] -= matrix.mRows[0][2]; mRows[1][0] -= matrix.mRows[1][0];
  mRows[1][1] -= matrix.mRows[1][1]; mRows[1][2] -= matrix.mRows[1][2];
  mRows[2][0] -= matrix.mRows[2][0]; mRows[2][1] -= matrix.mRows[2][1];
  mRows[2][2] -= matrix.mRows[2][2];
  return *this;
}

template<class T>
SIMD_INLINE rpMatrix3x3<T>& real_physics::rpMatrix3x3<T>::operator *=(T nb)
{
	mRows[0][0] *= nb; mRows[0][1] *= nb; mRows[0][2] *= nb;
    mRows[1][0] *= nb; mRows[1][1] *= nb; mRows[1][2] *= nb;
    mRows[2][0] *= nb; mRows[2][1] *= nb; mRows[2][2] *= nb;
  return *this;
}




template<class T>
SIMD_INLINE const rpVector3D<T>& real_physics::rpMatrix3x3<T>::operator [](int row) const
{
  return mRows[row];
}




template<class T>
SIMD_INLINE T *rpMatrix3x3<T>::getData()
{
    return &mRows[0][0];
}





template<class T>
SIMD_INLINE rpMatrix3x3<T> rpMatrix3x3<T>::transposeTimes( const rpMatrix3x3<T>& m ) const
{
	 return rpMatrix3x3<T>
	 (
			mRows[0].x * m[0].x + mRows[1].x * m[1].x + mRows[2].x * m[2].x,
			mRows[0].x * m[0].y + mRows[1].x * m[1].y + mRows[2].x * m[2].y,
			mRows[0].x * m[0].z + mRows[1].x * m[1].z + mRows[2].x * m[2].z,
			mRows[0].y * m[0].x + mRows[1].y * m[1].x + mRows[2].y * m[2].x,
			mRows[0].y * m[0].y + mRows[1].y * m[1].y + mRows[2].y * m[2].y,
			mRows[0].y * m[0].z + mRows[1].y * m[1].z + mRows[2].y * m[2].z,
			mRows[0].z * m[0].x + mRows[1].z * m[1].x + mRows[2].z * m[2].x,
			mRows[0].z * m[0].y + mRows[1].z * m[1].y + mRows[2].z * m[2].y,
			mRows[0].z * m[0].z + mRows[1].z * m[1].z + mRows[2].z * m[2].z
	);
}



template<class T>
SIMD_INLINE rpVector3D<T>& real_physics::rpMatrix3x3<T>::operator [](int row)
{
  return mRows[row];
}


template<class T>
SIMD_INLINE rpMatrix3x3<T> real_physics::rpMatrix3x3<T>::getInverse() const
{

    // Compute the determinant of the matrix
    T determinant = getDeterminant();

    // Check if the determinant is equal to zero
    // assert(Abs(determinant) > MACHINE_EPSILON);

    T invDeterminant = T(1.0) / determinant;

    rpMatrix3x3<T> tempMatrix((mRows[1][1]*mRows[2][2]-mRows[2][1]*mRows[1][2]),
                             -(mRows[0][1]*mRows[2][2]-mRows[2][1]*mRows[0][2]),
                              (mRows[0][1]*mRows[1][2]-mRows[0][2]*mRows[1][1]),
                             -(mRows[1][0]*mRows[2][2]-mRows[2][0]*mRows[1][2]),
                              (mRows[0][0]*mRows[2][2]-mRows[2][0]*mRows[0][2]),
                             -(mRows[0][0]*mRows[1][2]-mRows[1][0]*mRows[0][2]),
                              (mRows[1][0]*mRows[2][1]-mRows[2][0]*mRows[1][1]),
                             -(mRows[0][0]*mRows[2][1]-mRows[2][0]*mRows[0][1]),
                              (mRows[0][0]*mRows[1][1]-mRows[0][1]*mRows[1][0]));

    // Return the inverse matrix
    return (invDeterminant * tempMatrix);

}


template<class T>
rpMatrix3x3<T> rpMatrix3x3<T>::MatrixTensorProduct( const rpVector3D<T>&  v1, const rpVector3D<T>& v2 )
{
	return rpMatrix3x3<T>(v1.x * v2.x , v1.y * v2.x , v1.z * v2.x,
				          v1.x * v2.y , v1.y * v2.y , v1.z * v2.y,
						  v1.x * v2.z , v1.y * v2.z , v1.z * v2.z);
}


template<class T>
rpMatrix3x3<T> rpMatrix3x3<T>::rotateToInertialSystemR(const rpVector3D<T> &euler) const
{

    T m[9];

    T psi  = euler.y;
    T teta = euler.x;
    T phy  = euler.z;

    m[0] =  cos(psi) * cos(teta);
    m[1] =  sin(psi) * cos(teta);
    m[2] = -sin(teta);


    m[3] = cos(psi)*sin(teta)*sin(phy) - sin(psi)*cos(phy);
    m[4] = sin(psi)*sin(teta)*sin(phy) + cos(psi)*cos(phy);
    m[5] = sin(phy)*cos(teta);


    m[6] = cos(psi)*sin(teta)*cos(phy) + sin(psi)*sin(phy);
    m[7] = sin(psi)*sin(teta)*cos(phy) - cos(psi)*sin(phy);
    m[8] = cos(phy)*cos(teta);


    return rpMatrix3x3<T>( m[0] , m[3] , m[6] ,
                           m[1] , m[4] , m[7] ,
                           m[2] , m[5] , m[8] );
}


template<class T>
rpMatrix3x3<T> rpMatrix3x3<T>::TransitionMatrixR(const rpVector3D<T> &euler) const
{
    T m[9];

    T psi  = euler.y;
    T teta = euler.x;
    T phy  = euler.z;

    m[0] =  1;
    m[1] =  0;
    m[2] =  0;

    m[3] =  0;
    m[4] =  cos(phy);
    m[5] = -sin(phy);


    m[6] = -sin(teta);
    m[7] =  sin(phy) * cos(teta);
    m[8] =  cos(phy) * cos(teta);


    return rpMatrix3x3<T>( m[0] , m[3] , m[6] ,
                           m[1] , m[4] , m[7] ,
                           m[2] , m[5] , m[8] );

}



template<class T>
rpVector3D<T> rpMatrix3x3<T>::getEulerAngle()
{
    T angle_x;
    T angle_y;
    T angle_z;

    T RADIANS = 1.0;// 180.0 / PI;

    T D;
    angle_y = D = -asin( getData()[2]);         /* Вычисления угла вращения вокруг оси Y */
    T C         =  cos( angle_y );
    angle_y    *=  RADIANS;

    if ( fabs( C ) > 0.005 )                     /* "Шарнирный замок" (Gimball lock)? */
    {
        T rx      =  getData()[10] / C;          /* Если нет, то получаем угол вращения вокруг оси X */
        T ry      = -getData()[6]  / C;

        angle_x  = atan2( ry, rx ) * RADIANS;

        rx      =  getData()[0] / C;             /* Получаем угол вращения вокруг оси  Z */
        ry      = -getData()[1] / C;

        angle_z  = atan2( ry, rx ) * RADIANS;
    }
    else                                          /* Имеет место "Шарнирный замок" (Gimball lock) */
    {
        angle_x  = 0;                             /* Угол вращения вокруг оси X приравниваем к нулю */

        T rx      = getData()[5];                 /* И вычисляем угол вращения вокруг оси Z */
        T ry      = getData()[4];

        angle_z  = atan2( ry, rx ) * RADIANS;
    }


    /* Ранжируем найденные углы в диапазон от 0 до 360 градусов */
    //    angle_x = clamp( angle_x, 0.0, 360.0 );
    //    angle_y = clamp( angle_y, 0.0, 360.0 );
    //    angle_z = clamp( angle_z, 0.0, 360.0 );

    return rpVector3D<T>(angle_x , angle_y , angle_z);

}



} /* namespace real_physics */


#endif /* SOURCE_REAL_PHYSICS_LINEARMATHS_RPMATRIX3X3_H_ */
