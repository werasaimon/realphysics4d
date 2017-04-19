/*
 * rpQuaternion.cpp
 *
 *  Created on: 15 нояб. 2016 г.
 *      Author: wera
 */

#include "rpQuaternion.h"

namespace real_physics
{






//template<class T>
//SIMD_INLINE real_physics::rpQuaternion<T>::rpQuaternion(const rpMatrix3x3<T>& matrix)
//{
//
//    // Get the trace of the matrix
//    T trace = matrix.getTrace();
//
//    T r;
//    T s;
//
//    if (trace < 0.0)
//    {
//        if (matrix[1][1] > matrix[0][0])
//        {
//            if(matrix[2][2] > matrix[1][1])
//            {
//                r = sqrt(matrix[2][2] - matrix[0][0] - matrix[1][1] + T(1.0));
//                s = T(0.5) / r;
//
//                // Compute the quaternion
//                x = (matrix[2][0] + matrix[0][2]) * s;
//                y = (matrix[1][2] + matrix[2][1]) * s;
//                z = T(0.5) * r;
//                w = (matrix[1][0] - matrix[0][1]) * s;
//            }
//            else
//            {
//                r = sqrt(matrix[1][1] - matrix[2][2] - matrix[0][0] + T(1.0));
//                s = T(0.5) / r;
//
//                // Compute the quaternion
//                x = (matrix[0][1] + matrix[1][0]) * s;
//                y = T(0.5) * r;
//                z = (matrix[1][2] + matrix[2][1]) * s;
//                w = (matrix[0][2] - matrix[2][0]) * s;
//            }
//        }
//        else if (matrix[2][2] > matrix[0][0])
//        {
//            r = sqrt(matrix[2][2] - matrix[0][0] - matrix[1][1] + T(1.0));
//            s = T(0.5) / r;
//
//            // Compute the quaternion
//            x = (matrix[2][0] + matrix[0][2]) * s;
//            y = (matrix[1][2] + matrix[2][1]) * s;
//            z = T(0.5) * r;
//            w = (matrix[1][0] - matrix[0][1]) * s;
//        }
//        else
//        {
//            r = sqrt(matrix[0][0] - matrix[1][1] - matrix[2][2] + T(1.0));
//            s = T(0.5) / r;
//
//            // Compute the quaternion
//            x = T(0.5) * r;
//            y = (matrix[0][1] + matrix[1][0]) * s;
//            z = (matrix[2][0] - matrix[0][2]) * s;
//            w = (matrix[2][1] - matrix[1][2]) * s;
//        }
//    }
//    else
//    {
//        r = sqrt(trace + T(1.0));
//        s = T(0.5) / r;
//
//        // Compute the quaternion
//        x = (matrix[2][1] - matrix[1][2]) * s;
//        y = (matrix[0][2] - matrix[2][0]) * s;
//        z = (matrix[1][0] - matrix[0][1]) * s;
//        w = T(0.5) * r;
//    }
//}


template<class T>
SIMD_INLINE void real_physics::rpQuaternion<T>::getRotationAngleAxis(T& angle, rpVector3D<T>& axis) const
{

	    rpQuaternion<T> quaternion;

	    // If the quaternion is unit
	    if (length() == 1.0)
	    {
	        quaternion = *this;
	    }
	    else
	    {
	        // We compute the unit quaternion
	        quaternion = getUnit();
	    }

	    // Compute the roation angle
	    angle = acos(quaternion.w) * T(2.0);

	    // Compute the 3D rotation axis
	    rpVector3D<T> rotationAxis(quaternion.x, quaternion.y, quaternion.z);

	    // Normalize the rotation axis
	    rotationAxis = rotationAxis.getUnit();

	    // Set the rotation axis values
        axis.setAllValues(rotationAxis.x, rotationAxis.y, rotationAxis.z);
}








template<class T>
SIMD_INLINE rpQuaternion<T> real_physics::rpQuaternion<T>::slerp( const rpQuaternion<T>& quaternion1,
		                                                          const rpQuaternion<T>& quaternion2,
								 T t)
{
	assert(t >= 0.0 && t <= 1.0);

	T invert = 1.0;

	// Compute cos(theta) using the quaternion scalar product
	T cosineTheta = quaternion1.dot(quaternion2);

	// Take care of the sign of cosineTheta
	if (cosineTheta < 0.0)
	{
		cosineTheta = -cosineTheta;
		invert = -1.0;
	}

	// Because of precision, if cos(theta) is nearly 1,
	// therefore theta is nearly 0 and we can write
	// sin((1-t)*theta) as (1-t) and sin(t*theta) as t
	const T epsilon = T(0.00001);
	if(1-cosineTheta < epsilon)
	{
		return quaternion1 * (T(1.0)-t) + quaternion2 * (t * invert);
	}

	// Compute the theta angle
	T theta = acos(cosineTheta);

	// Compute sin(theta)
	T sineTheta = sin(theta);

	// Compute the two coefficients that are in the spherical linear interpolation formula
	T coeff1 = sin((T(1.0)-t)*theta) / sineTheta;
	T coeff2 = sin(t*theta) / sineTheta * invert;

	// Compute and return the interpolated quaternion
	return quaternion1 * coeff1 + quaternion2 * coeff2;
}








} /* namespace real_physics */
