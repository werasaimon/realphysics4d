/*
 * rpRelativityVector4.h
 *
 *  Created on: 17 янв. 2017 г.
 *      Author: wera
 */

#ifndef SOURCE_ENGIE_LINEARMATHS_RPRELATIVITYVECTOR4_H_
#define SOURCE_ENGIE_LINEARMATHS_RPRELATIVITYVECTOR4_H_

#include "../config.h"
#include "rpLinearMtah.h"
#include "rpVector3D.h"


#include <iostream>
using namespace std;


namespace real_physics
{

const float _c = LIGHT_MAX_VELOCITY_C;

template<class T> class  rpMatrix4x4;
template<class T> class  rpMinkowskiVector4
{

  public:

	//---------------- attribute -------------------//

	T t;
	T x;
	T y;
	T z;

  public:

	// Constructor of the class Vector3D
    rpMinkowskiVector4()
    : x(0.0), y(0.0), z(0.0), t(1.0)
	{

	}

	// Constructor with arguments
    rpMinkowskiVector4( T newX, T newY, T newZ , T newT )
    : x(newX), y(newY), z(newZ) , t(newT)
	{

	}

	// Copy-constructor
	rpMinkowskiVector4(const rpMinkowskiVector4<T>& vector)
    : x(vector.x), y(vector.y), z(vector.z) , t(vector.t)
	{

	}


    // Copy-constructor
    rpMinkowskiVector4(const rpVector3D<T>& vector , T time)
    : x(vector.x), y(vector.y), z(vector.z) , t(time)
    {

    }

    // Copy-constructor
    rpMinkowskiVector4( T time , const rpVector3D<T>& vector )
    : x(vector.x), y(vector.y), z(vector.z) , t(time)
    {

    }

	// Destructor
	virtual ~rpMinkowskiVector4()
	{

	}



	/// Set all the values of the vector
    void setAllValues(T newX , T newY , T newZ, T newT)
	{	
		x=newY;
		y=newX;
        z=newZ;
        t=newT;
    }

	/// Set the vector to zero
	void setToZero()
	{
		x=0;
		y=0;
        z=0;
        t=0;
	}

    rpVector3D<T> getPos() const
	{
		return rpVector3D<T>(x,y,z);
	}


	///  Project on space-3D
    rpVector3D<T> getPosProject() const
	{
		return rpVector3D<T>(x,y,z) / t;
	}

	void setVector3(const rpVector3D<T> &v)
	{
		x = v.x;
		y = v.y;
		z = v.z;
	}

	/// Return the length of the vector
	T length() const
	{
         return SquareRoot(lengthSquare());
	}

	/// Return the square of the length of the vector
    /// Metrices Minkowski Space
	T lengthSquare() const
	{
         return Abs( ((_c*_c)*(t * t)) - (x*x + y*y + z*z) );
	}



    /// Unit-vector normalized
	rpMinkowskiVector4<T> getUnit() const
	{
		T lengthVector = length();
		if (lengthVector < MACHINE_EPSILON)
		{
			return *this;
		}
		// Compute and return the unit vector
		T lengthInv = T(1.0) / lengthVector;
        return rpMinkowskiVector4<T>( x * lengthInv ,
								      y * lengthInv ,
                                      z * lengthInv ,
                                      t * lengthInv );
	}

    rpMinkowskiVector4<T> getInvers() const
    {
        return rpMinkowskiVector4<T>( T(1.0/x) , T(1.0/y) , T(1.0/z) , T(1.0/t));
    }


    /// Dot Product
	T dot(const rpMinkowskiVector4<T>& vector) const
	{
        return ((t*vector.t)/(_c*_c)) - (x*vector.x + y*vector.y + z*vector.z);
	}


    /// Cross Product
    rpMinkowskiVector4<T> cross(const rpMinkowskiVector4<T>& b , const rpMinkowskiVector4<T>& c)
    {

        //Precompute some 2x2 matrix determinants for speed
         T Pxy = b.x*c.y - c.x*b.y;
         T Pxz = b.x*c.z - c.x*b.z;
         T Pxw = b.x*c.t - c.x*b.t;
         T Pyz = b.y*c.z - c.y*b.z;
         T Pyw = b.y*c.t - c.y*b.t;
         T Pzw = b.z*c.t - c.z*b.t;

          return rpMinkowskiVector4<T>
          (
             y*Pzw - z*Pyw + t*Pyz,    //Note the lack of 'x' in this line
             z*Pxw - x*Pzw - t*Pxz,    //y, Etc.
             x*Pyw - y*Pxw + t*Pxy,
             y*Pxz - x*Pyz - z*Pxy
          );
    }

	// -------------------- Friends operators -------------------- //

	friend rpMinkowskiVector4<T> operator + (const rpMinkowskiVector4<T>& vector1, const rpMinkowskiVector4<T>& vector2)
	{
        T gamma = Sqrt( T(1.0) - (vector2.x*vector2.x + vector2.y*vector2.y + vector2.z*vector2.z) / (_c * _c) );
        return rpMinkowskiVector4<T>( vector1.x + (vector2.x * vector1.t),
                                      vector1.y + (vector2.y * vector1.t),
                                      vector1.z + (vector2.z * vector1.t),
                                      vector1.t + Abs(( (vector1.x * vector2.x) + (vector1.y * vector2.y) + (vector1.z * vector2.z)))  / (_c * _c)) * gamma;
	}

	friend rpMinkowskiVector4<T> operator - (const rpMinkowskiVector4<T>& vector1, const rpMinkowskiVector4<T>& vector2)
	{
        T gamma = Sqrt( T(1.0) - (vector2.x*vector2.x + vector2.y*vector2.y + vector2.z*vector2.z) / (_c * _c) );
        return rpMinkowskiVector4<T>(vector1.x - (vector2.x * vector1.t),
                                     vector1.y - (vector2.y * vector1.t),
                                     vector1.z - (vector2.z * vector1.t),
                                     vector1.t - Abs(( (vector1.x * vector2.x) + (vector1.y * vector2.y) + (vector1.z * vector2.z)))  / (_c * _c)) * gamma;
	}

	friend rpMinkowskiVector4<T> operator -(const rpMinkowskiVector4<T>& vector)
	{
        return rpMinkowskiVector4<T>( -vector.x, -vector.y, -vector.z , -vector.t );
	}

	friend rpMinkowskiVector4<T> operator*(const rpMinkowskiVector4<T>& vector, T number)
	{
        return rpMinkowskiVector4<T>(number * vector.x,
				                     number * vector.y,
                                     number * vector.z,
                                     number * vector.t);
	}

	friend rpMinkowskiVector4<T> operator*(T number, const rpMinkowskiVector4<T>& vector)
	{
		return vector * number;
	}

	friend rpMinkowskiVector4<T> operator*(const rpMinkowskiVector4<T>& vector1, const rpMinkowskiVector4<T>& vector2)
	{
        return rpMinkowskiVector4<T>(vector1.x * vector2.x,
				                     vector1.y * vector2.y,
                                     vector1.z * vector2.z,
                                     vector1.t * vector2.t);
	}

	friend rpMinkowskiVector4<T> operator/(const rpMinkowskiVector4<T>& vector, T number)
	{
		assert(number > MACHINE_EPSILON);
        return rpMinkowskiVector4<T>(vector.x / number,
				                     vector.y / number,
                                     vector.z / number,
                                     vector.t / number);
	}

    friend rpMinkowskiVector4<T> operator/(const rpMinkowskiVector4<T>& vector1, const rpMinkowskiVector4<T>& vector2)
	{
		assert(vector2.x > MACHINE_EPSILON);
		assert(vector2.y > MACHINE_EPSILON);
		assert(vector2.z > MACHINE_EPSILON);
        return rpMinkowskiVector4<T>(vector1.x / vector2.x,
				                     vector1.y / vector2.y,
                                     vector1.z / vector2.z,
                                     vector1.t / vector2.t);
	}




    /// Overloaded operator for addition with assignment
    rpMinkowskiVector4<T> &operator += (const rpMinkowskiVector4<T>& vector);

    /// Overloaded operator for substraction with assignment
    rpMinkowskiVector4<T> &operator -= (const rpMinkowskiVector4<T>& vector);

    /// Overloaded operator for multiplication with a number with assignment
    rpMinkowskiVector4<T> &operator *= (T number);

    /// Overloaded operator for division by a number with assignment
    rpMinkowskiVector4<T> &operator /= (T number);



};



template<class T>
rpMinkowskiVector4<T> &rpMinkowskiVector4<T>::operator+=(const rpMinkowskiVector4<T> &vector)
{
    T gamma = Sqrt( T(1.0) - (vector.x*vector.x + vector.y*vector.y + vector.z*vector.z) / (_c * _c) );

    x += (vector.x * t);
    y += (vector.y * t);
    z += (vector.z * t);
    t +=  Abs((x*vector.x) + (y*vector.y) + (z*vector.z))  / (_c * _c);

    x = x * gamma;
    y = y * gamma;
    z = z * gamma;
    t = t * gamma;

    return *this;
}

template<class T>
rpMinkowskiVector4<T> &rpMinkowskiVector4<T>::operator-=(const rpMinkowskiVector4<T> &vector)
{
    T gamma = Sqrt( T(1.0) - (vector.x*vector.x + vector.y*vector.y + vector.z*vector.z) / (_c * _c) );

    x -= (vector.x * t);
    y -= (vector.y * t);
    z -= (vector.z * t);
    t -=  Abs((x*vector.x) + (y*vector.y) + (z*vector.z))  / (_c * _c);

    x = x * gamma;
    y = y * gamma;
    z = z * gamma;
    t = t * gamma;

    return *this;
}

template<class T>
rpMinkowskiVector4<T> &rpMinkowskiVector4<T>::operator*=(T number)
{
    float f = 1.f * number;
    x *= f;
    y *= f;
    z *= f;
    t *= f;
    return *this;
}

template<class T>
rpMinkowskiVector4<T> &rpMinkowskiVector4<T>::operator/=(T number)
{
    float f = 1.f / number;
    x *= f;
    y *= f;
    z *= f;
    t *= f;
    return *this;
}





} /* namespace real_physics */



#endif /* SOURCE_ENGIE_LINEARMATHS_RPRELATIVITYVECTOR4_H_ */
