/*
 * rpMatrix3x3.cpp
 *
 *  Created on: 15 нояб. 2016 г.
 *      Author: wera
 */

#include "rpMatrix3x3.h"
#include "rpQuaternion.h"

namespace real_physics
{












//// Constructor of the class Matrix3x3
//template<class T>
// rpMatrix3x3<T>::rpMatrix3x3()
//{
//    // Initialize all values in the matrix to zero
//    setAllValues(0.0, 0.0, 0.0,
//    		     0.0, 0.0, 0.0,
//				 0.0, 0.0, 0.0);
//}



//// Constructor
//template<class T>
// rpMatrix3x3<T>::rpMatrix3x3(T value)
//{
//    setAllValues(value, value, value,
//    		     value, value, value,
//				 value, value, value);
//}





//// Constructor with arguments
//template<class T>
//rpMatrix3x3<T>::rpMatrix3x3(T a1, T a2, T a3,
//						    T b1, T b2, T b3,
//			 			    T c1, T c2, T c3)
//{
//
//	// Initialize the matrix with the values
//	setAllValues(a1, a2, a3,
//			     b1, b2, b3,
//			     c1, c2, c3);
//}




// Copy-constructor
//template<class T>
//rpMatrix3x3<T>::rpMatrix3x3(const rpMatrix3x3<T>& matrix)
//{
//    setAllValues(matrix.mRows[0][0], matrix.mRows[0][1], matrix.mRows[0][2],
//                 matrix.mRows[1][0], matrix.mRows[1][1], matrix.mRows[1][2],
//                 matrix.mRows[2][0], matrix.mRows[2][1], matrix.mRows[2][2]);
//}



//template<class T>
//SIMD_INLINE  rpMatrix3x3<T>& real_physics::rpMatrix3x3<T>::operator =(const rpMatrix3x3<T>& matrix)
//{
//    // Check for self-assignment
//    if (&matrix != this)
//    {
//        setAllValues(matrix.mRows[0][0], matrix.mRows[0][1], matrix.mRows[0][2],
//                     matrix.mRows[1][0], matrix.mRows[1][1], matrix.mRows[1][2],
//                     matrix.mRows[2][0], matrix.mRows[2][1], matrix.mRows[2][2]);
//    }
//    return *this;
//}


//template<class T>
//SIMD_INLINE rpMatrix3x3<T> real_physics::rpMatrix3x3<T>::getInverse() const
//{
//
//
//    // Compute the determinant of the matrix
//    T determinant = getDeterminant();
//
//    // Check if the determinant is equal to zero
//    assert(Abs(determinant) > MACHINE_EPSILON);
//
//    T invDeterminant = T(1.0) / determinant;
//
//    rpMatrix3x3<T> tempMatrix((mRows[1][1]*mRows[2][2]-mRows[2][1]*mRows[1][2]),
//                             -(mRows[0][1]*mRows[2][2]-mRows[2][1]*mRows[0][2]),
//                              (mRows[0][1]*mRows[1][2]-mRows[0][2]*mRows[1][1]),
//                             -(mRows[1][0]*mRows[2][2]-mRows[2][0]*mRows[1][2]),
//                              (mRows[0][0]*mRows[2][2]-mRows[2][0]*mRows[0][2]),
//                             -(mRows[0][0]*mRows[1][2]-mRows[1][0]*mRows[0][2]),
//                              (mRows[1][0]*mRows[2][1]-mRows[2][0]*mRows[1][1]),
//                             -(mRows[0][0]*mRows[2][1]-mRows[2][0]*mRows[0][1]),
//                              (mRows[0][0]*mRows[1][1]-mRows[0][1]*mRows[1][0]));
//
//    // Return the inverse matrix
//    return (invDeterminant * tempMatrix);
//
//}





} /* namespace real_physics */
