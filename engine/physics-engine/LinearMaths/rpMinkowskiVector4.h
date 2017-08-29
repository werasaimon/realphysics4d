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
	: t(1.0), x(0.0), y(0.0), z(0.0)
	{

	}

	// Constructor with arguments
	rpMinkowskiVector4(T newT , T newX, T newY, T newZ)
	: t(newT) , x(newX), y(newY), z(newZ)
	{

	}

	// Copy-constructor
	rpMinkowskiVector4(const rpMinkowskiVector4<T>& vector)
	: t(vector.t) , x(vector.x), y(vector.y), z(vector.z)
	{

	}


	// Copy-constructor
	rpMinkowskiVector4(const rpVector3D<T>& vector , T time)
	: t(time) , x(vector.x), y(vector.y), z(vector.z)
	{

	}

	// Destructor
	virtual ~rpMinkowskiVector4()
	{

	}



	/// Set all the values of the vector
	void setAllValues(T newT , T newX , T newY , T newZ)
	{
		t=newT;
		x=newY;
		y=newX;
		z=newZ;
	}

	/// Set the vector to zero
	void setToZero()
	{
		t=0;
		x=0;
		y=0;
		z=0;
	}

	rpVector3D<T> getVector3() const
	{
		return rpVector3D<T>(x,y,z);
	}


	///  Project on space-3D
	rpVector3D<T> getProjVector3() const
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
		 return SquareRoot(Abs(lengthSquare()));
	}

	/// Return the square of the length of the vector
	T lengthSquare() const
	{
		 return ((t * t) - (x*x + y*y + z*z));
	}




	rpMinkowskiVector4<T> getUnit() const
	{
		T lengthVector = length();
		if (lengthVector < MACHINE_EPSILON)
		{
			return *this;
		}
		// Compute and return the unit vector
		T lengthInv = T(1.0) / lengthVector;
		return rpMinkowskiVector4<T>( t * lengthInv ,
				                      x * lengthInv ,
								      y * lengthInv ,
								      z * lengthInv);
	}


	/// dot Product
	T dot(const rpMinkowskiVector4<T>& vector) const
	{
		return (x*vector.x + y*vector.y + z*vector.z) - (t*vector.t);
	}



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
		return rpMinkowskiVector4<T>(vector1.t + vector2.t,
				                     vector1.x + vector2.x,
				                     vector1.y + vector2.y,
				                     vector1.z + vector2.z);
	}

	friend rpMinkowskiVector4<T> operator - (const rpMinkowskiVector4<T>& vector1, const rpMinkowskiVector4<T>& vector2)
	{
		return rpMinkowskiVector4<T>(vector1.t - vector2.t,
				                     vector1.x - vector2.x,
				                     vector1.y - vector2.y,
				                     vector1.z - vector2.z);
	}

	friend rpMinkowskiVector4<T> operator -(const rpMinkowskiVector4<T>& vector)
	{
		return rpMinkowskiVector4<T>(vector.t , -vector.x, -vector.y, -vector.z);
	}

	friend rpMinkowskiVector4<T> operator*(const rpMinkowskiVector4<T>& vector, T number)
	{
		return rpMinkowskiVector4<T>(number * vector.t,
				                     number * vector.x,
				                     number * vector.y,
				                     number * vector.z);
	}

	friend rpMinkowskiVector4<T> operator*(T number, const rpMinkowskiVector4<T>& vector)
	{
		return vector * number;
	}

	friend rpMinkowskiVector4<T> operator*(const rpMinkowskiVector4<T>& vector1, const rpMinkowskiVector4<T>& vector2)
	{
		return rpMinkowskiVector4<T>(vector1.t * vector2.t,
				                     vector1.x * vector2.x,
				                     vector1.y * vector2.y,
				                     vector1.z * vector2.z);
	}

	friend rpMinkowskiVector4<T> operator/(const rpMinkowskiVector4<T>& vector, T number)
	{
		assert(number > MACHINE_EPSILON);
		return rpMinkowskiVector4<T>(vector.t / number,
				                     vector.x / number,
				                     vector.y / number,
				                     vector.z / number);
	}

	friend rpMinkowskiVector4<T> operator/(const rpMinkowskiVector4<T>& vector1, const rpMinkowskiVector4<T>& vector2)
	{
		assert(vector2.x > MACHINE_EPSILON);
		assert(vector2.y > MACHINE_EPSILON);
		assert(vector2.z > MACHINE_EPSILON);
		return rpMinkowskiVector4<T>(vector1.t / vector2.t,
				                     vector1.x / vector2.x,
				                     vector1.y / vector2.y,
				                     vector1.z / vector2.z);
	}

};



} /* namespace real_physics */



#endif /* SOURCE_ENGIE_LINEARMATHS_RPRELATIVITYVECTOR4_H_ */
