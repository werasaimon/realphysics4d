/*
 * rpMatrix2x2.h
 *
 *  Created on: 15 нояб. 2016 г.
 *      Author: wera
 */

#ifndef SOURCE_REAL_PHYSICS_LINEARMATHS_RPMATRIX2X2_H_
#define SOURCE_REAL_PHYSICS_LINEARMATHS_RPMATRIX2X2_H_

// Libraries
#include <cassert>
#include "rpVector2D.h"


namespace real_physics
{

template<class T> class rpMatrix2x2
{


 private :

    // -------------------- Attributes -------------------- //


	/// Rows of the matrix;
	rpVector2D<T> mRows[2];


  public :

       // -------------------- Methods -------------------- //

	// Constructor of the class Matrix2x2
	rpMatrix2x2()
	{
	    // Initialize all values in the matrix to zero
	    setAllValues(0.0, 0.0, 0.0, 0.0);
	}

	// Constructor
	rpMatrix2x2( T value )
	{
	    setAllValues(value, value, value, value);
	}

	// Constructor with arguments
	rpMatrix2x2( T a1, T a2, T b1, T b2)
	{
	    // Initialize the matrix with the values
	    setAllValues(a1, a2, b1, b2);
	}

	// Destructor
	virtual ~rpMatrix2x2()
	{

	}

	// Copy-constructor
	rpMatrix2x2(const rpMatrix2x2<T>& matrix)
	{
	    setAllValues(matrix.mRows[0][0], matrix.mRows[0][1],
	                 matrix.mRows[1][0], matrix.mRows[1][1]);
	}


	//-----------------------------------------------------------//

	/// Assignment operator
	rpMatrix2x2<T>& operator = (const rpMatrix2x2<T>& matrix);

	/// Set all the values in the matrix
	void setAllValues(T a1, T a2, T b1, T b2);

	/// Set the matrix to zero
	void setToZero();


	//-----------------------------------------------------------//

    /// Return a column
	rpMatrix2x2<T>&  getColumn(int i) const;

	/// Return a row
	rpMatrix2x2<T>&  getRow(int i) const;



	 /// Return the transpose matrix
	rpMatrix2x2<T> getTranspose() const;

	/// Return the determinant of the matrix
	T getDeterminant() const;



	/// Return the trace of the matrix
	T getTrace() const;

	/// Return the inverse matrix
	rpMatrix2x2<T> getInverse() const;

	/// Return the matrix with absolute values
	rpMatrix2x2<T> getAbsoluteMatrix() const;

	/// Set the matrix to the identity matrix
	void setToIdentity();

	/// Return the 2x2 identity matrix
	static rpMatrix2x2<T>  identity();

 /// Return the 2x2 zero matrix
    static rpMatrix2x2<T>  zero();




    //------------------ friend method ---------------------------------//

    /// Overloaded operator for addition
    friend rpMatrix2x2<T> operator+(const rpMatrix2x2<T>& matrix1, const rpMatrix2x2<T>& matrix2)
    {
    	return rpMatrix2x2<T>(matrix1.mRows[0][0] + matrix2.mRows[0][0],
    			              matrix1.mRows[0][1] + matrix2.mRows[0][1],
				              matrix1.mRows[1][0] + matrix2.mRows[1][0],
				              matrix1.mRows[1][1] + matrix2.mRows[1][1]);

    }

    /// Overloaded operator for substraction
    friend rpMatrix2x2<T> operator-(const rpMatrix2x2<T>& matrix1, const rpMatrix2x2<T>& matrix2)
    {
    	 return rpMatrix2x2<T>(matrix1.mRows[0][0] - matrix2.mRows[0][0],
    	                       matrix1.mRows[0][1] - matrix2.mRows[0][1],
    	                       matrix1.mRows[1][0] - matrix2.mRows[1][0],
    	                       matrix1.mRows[1][1] - matrix2.mRows[1][1]);

    }

    /// Overloaded operator for the negative of the matrix
    friend rpMatrix2x2<T> operator-(const rpMatrix2x2<T>& matrix)
    {
    	 return rpMatrix2x2<T>(-matrix.mRows[0][0], -matrix.mRows[0][1],
    	                       -matrix.mRows[1][0], -matrix.mRows[1][1]);
    }

    /// Overloaded operator for multiplication with a number
    friend rpMatrix2x2<T> operator*( T nb, const rpMatrix2x2<T>& matrix)
    {
    	 return rpMatrix2x2<T>(matrix.mRows[0][0] * nb, matrix.mRows[0][1] * nb,
    	                       matrix.mRows[1][0] * nb, matrix.mRows[1][1] * nb);

    }

    /// Overloaded operator for multiplication with a matrix
    friend rpMatrix2x2<T> operator*(const rpMatrix2x2<T>& matrix, T nb)
    {
    	return nb * matrix;
    }

    /// Overloaded operator for matrix multiplication
    friend rpMatrix2x2<T> operator*(const rpMatrix2x2<T>& matrix1, const rpMatrix2x2<T>& matrix2)
    {
    	 return rpMatrix2x2<T>(matrix1.mRows[0][0] * matrix2.mRows[0][0] + matrix1.mRows[0][1] *
    	                       matrix2.mRows[1][0],
    	                       matrix1.mRows[0][0] * matrix2.mRows[0][1] + matrix1.mRows[0][1] *
    	                       matrix2.mRows[1][1],
    	                       matrix1.mRows[1][0] * matrix2.mRows[0][0] + matrix1.mRows[1][1] *
    	                       matrix2.mRows[1][0],
    	                       matrix1.mRows[1][0] * matrix2.mRows[0][1] + matrix1.mRows[1][1] *
    	                       matrix2.mRows[1][1]);
    }

    /// Overloaded operator for multiplication with a vector
    friend rpVector2D<T> operator*(const rpMatrix2x2<T>& matrix, const rpVector2D<T>& vector)
    {
    	  return rpVector2D<T>(matrix.mRows[0][0]*vector.x + matrix.mRows[0][1]*vector.y,
    	                       matrix.mRows[1][0]*vector.x + matrix.mRows[1][1]*vector.y);
    }

    //-------------------------------------------------------------------//


    /// Overloaded operator for equality condition
    bool operator==(const rpMatrix2x2<T>& matrix) const;

    /// Overloaded operator for the is different condition
    bool operator!= (const rpMatrix2x2<T>& matrix) const;

    /// Overloaded operator for addition with assignment
    rpMatrix2x2<T>& operator+=(const rpMatrix2x2<T>& matrix);

    /// Overloaded operator for substraction with assignment
    rpMatrix2x2<T>& operator-=(const rpMatrix2x2<T>& matrix);




    /// Overloaded operator for multiplication with a number with assignment
    rpMatrix2x2<T>& operator*=(T nb);

    /// Overloaded operator to read element of the matrix.
    const rpVector2D<T>& operator[](int row) const;

    /// Overloaded operator to read/write element of the matrix.
    rpVector2D<T>& operator[](int row);

};


template<class T>
SIMD_INLINE void real_physics::rpMatrix2x2<T>::setAllValues(T a1, T a2, T b1, T b2)
{
	 mRows[0][0] = a1; mRows[0][1] = a2;
	 mRows[1][0] = b1; mRows[1][1] = b2;
}

template<class T>
SIMD_INLINE void real_physics::rpMatrix2x2<T>::setToZero()
{
	 mRows[0].setToZero();
	 mRows[1].setToZero();
}

template<class T>
SIMD_INLINE rpMatrix2x2<T>& real_physics::rpMatrix2x2<T>::getColumn(int i) const
{
	assert(i>= 0 && i<2);
	return Vector2(mRows[0][i], mRows[1][i]);
}

template<class T>
SIMD_INLINE rpMatrix2x2<T>& real_physics::rpMatrix2x2<T>::getRow(int i) const
{
    assert(i>= 0 && i<2);
    return mRows[i];
}


template<class T>
SIMD_INLINE rpMatrix2x2<T> real_physics::rpMatrix2x2<T>::getTranspose() const
{
	 // Return the transpose matrix
	    return Matrix2x2(mRows[0][0], mRows[1][0],
	                     mRows[0][1], mRows[1][1]);
}

template<class T>
SIMD_INLINE T real_physics::rpMatrix2x2<T>::getDeterminant() const
{
    // Compute and return the determinant of the matrix
    return mRows[0][0] * mRows[1][1] - mRows[1][0] * mRows[0][1];
}

template<class T>
SIMD_INLINE T real_physics::rpMatrix2x2<T>::getTrace() const
{
	  // Compute and return the trace
	    return (mRows[0][0] + mRows[1][1]);
}


template<class T>
SIMD_INLINE rpMatrix2x2<T> real_physics::rpMatrix2x2<T>::getAbsoluteMatrix() const
{
	return rpMatrix2x2<T>(fabs(mRows[0][0]), fabs(mRows[0][1]),
	                     fabs(mRows[1][0]), fabs(mRows[1][1]));
}

template<class T>
SIMD_INLINE void real_physics::rpMatrix2x2<T>::setToIdentity()
{
    mRows[0][0] = 1.0; mRows[0][1] = 0.0;
    mRows[1][0] = 0.0; mRows[1][1] = 1.0;
}

template<class T>
SIMD_INLINE rpMatrix2x2<T> real_physics::rpMatrix2x2<T>::identity()
{
    // Return the isdentity matrix
    return rpMatrix2x2<T>(1.0, 0.0, 0.0, 1.0);
}

template<class T>
SIMD_INLINE rpMatrix2x2<T> real_physics::rpMatrix2x2<T>::zero()
{
	return rpMatrix2x2<T>(0.0, 0.0, 0.0, 0.0);
}

template<class T>
SIMD_INLINE bool real_physics::rpMatrix2x2<T>::operator ==(const rpMatrix2x2<T>& matrix) const
{
	return (mRows[0][0] == matrix.mRows[0][0] && mRows[0][1] == matrix.mRows[0][1] &&
	        mRows[1][0] == matrix.mRows[1][0] && mRows[1][1] == matrix.mRows[1][1]);
}

template<class T>
SIMD_INLINE bool real_physics::rpMatrix2x2<T>::operator !=(const rpMatrix2x2<T>& matrix) const
{
	 return !(*this == matrix);
}

template<class T>
SIMD_INLINE rpMatrix2x2<T>& real_physics::rpMatrix2x2<T>::operator +=(const rpMatrix2x2<T>& matrix)
{
	 mRows[0][0] += matrix.mRows[0][0]; mRows[0][1] += matrix.mRows[0][1];
	 mRows[1][0] += matrix.mRows[1][0]; mRows[1][1] += matrix.mRows[1][1];
     return *this;
}

template<class T>
SIMD_INLINE rpMatrix2x2<T>& real_physics::rpMatrix2x2<T>::operator -=(const rpMatrix2x2<T>& matrix)
{
	 mRows[0][0] -= matrix.mRows[0][0]; mRows[0][1] -= matrix.mRows[0][1];
	 mRows[1][0] -= matrix.mRows[1][0]; mRows[1][1] -= matrix.mRows[1][1];
	 return *this;
}

template<class T>
SIMD_INLINE rpMatrix2x2<T>& real_physics::rpMatrix2x2<T>::operator *=(T nb)
{
	 mRows[0][0] *= nb; mRows[0][1] *= nb;
	 mRows[1][0] *= nb; mRows[1][1] *= nb;
	 return *this;
}

template<class T>
SIMD_INLINE const rpVector2D<T>& real_physics::rpMatrix2x2<T>::operator [](int row) const
{
	 return mRows[row];
}

template<class T>
SIMD_INLINE rpVector2D<T>& real_physics::rpMatrix2x2<T>::operator [](int row)
{
	 return mRows[row];
}


template<class T>
SIMD_INLINE rpMatrix2x2<T> real_physics::rpMatrix2x2<T>::getInverse() const
{

    // Compute the determinant of the matrix
    T determinant = getDeterminant();

    // Check if the determinant is equal to zero
    assert(Abs(determinant) > MACHINE_EPSILON);

    T invDeterminant = T(1.0) / determinant;

    rpMatrix2x2<T> tempMatrix(mRows[1][1], -mRows[0][1], -mRows[1][0], mRows[0][0]);

    // Return the inverse matrix
    return (invDeterminant * tempMatrix);
}

template<class T>
SIMD_INLINE rpMatrix2x2<T>& real_physics::rpMatrix2x2<T>::operator =( const rpMatrix2x2<T>& matrix )
{
    // Check for self-assignment
    if (&matrix != this)
    {
        setAllValues(matrix.mRows[0][0], matrix.mRows[0][1],
                     matrix.mRows[1][0], matrix.mRows[1][1]);
    }
    return *this;
}


} /* namespace real_physics */



#endif /* SOURCE_REAL_PHYSICS_LINEARMATHS_RPMATRIX2X2_H_ */
