/*
 * rpVector3D.h
 *
 *  Created on: 15 нояб. 2016 г.
 *      Author: wera
 */

#ifndef SOURCE_REAL_PHYSICS_LINEARMATHS_RPVECTOR3D_H_
#define SOURCE_REAL_PHYSICS_LINEARMATHS_RPVECTOR3D_H_


#include "rpLinearMtah.h"

namespace real_physics
{

  template<class T> class  rpMatrix3x3;
  template<class T> class  rpVector3D
  {
  public:


    //---------------- atribute -------------------//

    union
    {
      T f[3];
      struct
      {
			T x;
			T y;
			T z;
      };
    };

    //---------------------------------------------//


    // Constructor of the class Vector3D
    rpVector3D()
    : x(0.0), y(0.0), z(0.0)
    {

    }

    // Constructor with arguments
    rpVector3D(T newX, T newY, T newZ)
    : x(newX), y(newY), z(newZ)
    {

    }

    // Copy-constructor
    rpVector3D(const rpVector3D<T>& vector)
    : x(vector.x), y(vector.y), z(vector.z)
    {

    }

    // Destructor
    virtual ~rpVector3D();



    /// Set all the values of the vector
    void setAllValues(T newX , T newY , T newZ);

    /// Set the vector to zero
    void setToZero();

    /// Return the length of the vector
    T length() const;

    /// Return the square of the length of the vector
    T lengthSquare() const;


    /// Return the square of the length of the vector
    T length2() const;



    // Return the corresponding absolute value vector
    rpVector3D<T> getAbsoluteVector() const;

    // Return the axis with the minimal value
    int  getMinAxis() const;

    // Return the axis with the maximal value
    int  getMaxAxis() const;

    // Return true if the vector is unit and false otherwise
    bool isUnit() const;

    // Return true if the vector is the zero vector
    bool isZero() const;



    /// Return the minimum value among the three components of a vector
    T getMinValue() const;

    /// Return the maximum value among the three components of a vector
    T getMaxValue() const;


    //-------------------------------------------------------------------//



    /// Return the corresponding unit vector
    rpVector3D<T>  getUnit() const;

    /// Return one unit orthogonal vector of the current vector
    rpVector3D<T>  getOneUnitOrthogonalVector() const;

    // Projection onto another vector
    rpVector3D<T>  projection(const rpVector3D<T>& o) const;


    /// Return the Lenght unit vector [A.B]:leght
    T getDistanceTo(const rpVector3D<T>& other) const;

    /// Return the LenghtSquared unit vector [A.B]:leghtSquared
    T getSquaredDistanceTo(const rpVector3D<T>& other) const;

    /// Return the Angle unit vector [A.B]:Angle
    T AngleBetweenVectors( const rpVector3D<T>& Vector2 ) const;




    //-------------------------------------------------------------------//
    T getX() const { return x; }
    T getY() const { return y; }
    T getZ() const { return z; }
    //-------------------------------------------------------------------//

    /// Dot product of two vectors
    T dot(const rpVector3D<T>& vector) const;

    /// Cross product of two vectors
    rpVector3D<T> cross(const rpVector3D<T>& vector) const;

    rpVector3D<T>  dot3( const rpVector3D<T> &v0, const rpVector3D<T> &v1, const rpVector3D<T> &v2 ) const;

    /// Normalize the vector
    void normalize();

    //--------------------------------------------------------------------//


    /// Overloaded operator for the equality condition
    bool operator == (const rpVector3D<T>& vector) const;

    /// Overloaded operator for the is different condition
    bool operator != (const rpVector3D<T>& vector) const;

    /// Overloaded operator for addition with assignment
    rpVector3D<T>& operator+=(const rpVector3D<T>& vector);

    /// Overloaded operator for substraction with assignment
    rpVector3D<T>& operator-=(const rpVector3D<T>& vector);

    /// Overloaded operator for multiplication with a number with assignment
    rpVector3D<T>& operator*=(T number);

    /// Overloaded operator for division by a number with assignment
    rpVector3D<T>& operator/=(T number);



    /// Overloaded operator for value access
    T& operator[] (int index);

    /// Overloaded operator for value access
    const T& operator[] (int index) const;

    /// Overloaded operator
    rpVector3D<T>& operator=(const rpVector3D<T>& vector);

    /// Overloaded less than operator for ordering to be used inside std::set for instance
    bool operator<(const rpVector3D<T>& vector) const;


    // -------------------- Friends -------------------- //

    friend rpVector3D<T> operator+(const rpVector3D<T>& vector1, const rpVector3D<T>& vector2)
    {
      return rpVector3D<T>(vector1.x + vector2.x, vector1.y + vector2.y, vector1.z + vector2.z);
    }

    friend rpVector3D<T> operator-(const rpVector3D<T>& vector1, const rpVector3D<T>& vector2)
    {
      return rpVector3D<T>(vector1.x - vector2.x, vector1.y - vector2.y, vector1.z - vector2.z);
    }

    friend rpVector3D<T> operator-(const rpVector3D<T>& vector)
    {
      return rpVector3D<T>(-vector.x, -vector.y, -vector.z);
    }

    friend rpVector3D<T> operator*(const rpVector3D<T>& vector, T number)
    {
      return rpVector3D<T>(number * vector.x, number * vector.y, number * vector.z);
    }

    friend rpVector3D<T> operator*(T number, const rpVector3D<T>& vector)
    {
      return vector * number;
    }

    friend rpVector3D<T> operator*(const rpVector3D<T>& vector1, const rpVector3D<T>& vector2)
    {
      return rpVector3D<T>(vector1.x * vector2.x, vector1.y * vector2.y, vector1.z * vector2.z);
    }

    friend rpVector3D<T> operator^(const rpVector3D<T>& vector1, const rpVector3D<T>& vector2)
    {
      return rpVector3D<T>(vector1.cross(vector2));
    }


    friend rpVector3D<T> operator/(const rpVector3D<T>& vector, T number)
    {
      //assert(number > MACHINE_EPSILON);
      return rpVector3D<T>(vector.x / number, vector.y / number, vector.z / number);
    }

    friend rpVector3D<T> operator/(const rpVector3D<T>& vector1, const rpVector3D<T>& vector2)
    {
      //assert(vector2.x > MACHINE_EPSILON);
      //assert(vector2.y > MACHINE_EPSILON);
      //assert(vector2.z > MACHINE_EPSILON);
      return rpVector3D<T>(vector1.x / vector2.x, vector1.y / vector2.y, vector1.z / vector2.z);
    }





    //------------------------------- static method-------------------------//

    //-------- help Init Vectors ----------//
    static const rpVector3D<T> IDENTITY;
    static const rpVector3D<T> ZERO;
    static const rpVector3D<T> X;
    static const rpVector3D<T> Y;
    static const rpVector3D<T> Z;


    /// Clamp a vector such that it is no longer than a given maximum length
    static rpVector3D<T> clamp(const rpVector3D<T>& vector , T maxLength);

    /// Return a vector taking the minimum components of two vectors
    static rpVector3D<T> min(const rpVector3D<T>& vector1, const rpVector3D<T>& vector2);

    /// Return a vector taking the maximum components of two vectors
    static rpVector3D<T> max(const rpVector3D<T>& vector1, const rpVector3D<T>& vector2);

    /// Return the zero vector
    static rpVector3D<T> zero();


    static T AngleSigned(rpVector3D<T> v1, rpVector3D<T> v2, rpVector3D<T> normal)
    {
        return atan2( normal.dot(v1.cross(v2)), v1.dot(v2));
    }

    static rpVector3D<T> planeNormal(const rpVector3D<T>& V0 ,
    		                         const rpVector3D<T>& V1 ,
									 const rpVector3D<T>& V2);


    static void btPlaneSpace1(const rpVector3D<T>& n , rpVector3D<T>& p, rpVector3D<T>& q );

  };


  template<class T>
  SIMD_INLINE real_physics::rpVector3D<T>::~rpVector3D()
  {
  }

  template<class T>
  SIMD_INLINE void real_physics::rpVector3D<T>::setAllValues(T newX, T newY, T newZ)
  {
    x = newX;
    y = newY;
    z = newZ;
  }

  template<class T>
  SIMD_INLINE void real_physics::rpVector3D<T>::setToZero()
  {
    x = scalar(0);
    y = scalar(0);
    z = scalar(0);
  }

  template<class T>
  SIMD_INLINE T real_physics::rpVector3D<T>::length() const
  {
    return SquareRoot(x*x + y*y + z*z);
  }

  template<class T>
  SIMD_INLINE T real_physics::rpVector3D<T>::lengthSquare() const
  {
	  return x*x + y*y + z*z;
  }


  template<class T>
  SIMD_INLINE T rpVector3D<T>::length2() const
  {
	  return x*x + y*y + z*z;
  }

  template<class T>
  SIMD_INLINE rpVector3D<T> real_physics::rpVector3D<T>::getAbsoluteVector() const
  {
    return rpVector3D<T>(Abs(x) , Abs(y) , Abs(z));
  }

  template<class T>
  SIMD_INLINE int real_physics::rpVector3D<T>::getMinAxis() const
  {
    return (x < y ? (x < z ? 0 : 2) : (y < z ? 1 : 2));
  }

  template<class T>
  SIMD_INLINE int real_physics::rpVector3D<T>::getMaxAxis() const
  {
    return (x < y ? (y < z ? 2 : 1) : (x < z ? 2 : 0));
  }

  template<class T>
  SIMD_INLINE bool real_physics::rpVector3D<T>::isUnit() const
  {
    return approxEqual(lengthSquare(), 1.0);
  }

  template<class T>
  SIMD_INLINE bool real_physics::rpVector3D<T>::isZero() const
  {
    return approxEqual(lengthSquare(), 0.0);
  }

  template<class T>
  SIMD_INLINE T real_physics::rpVector3D<T>::getMinValue() const
  {
    return Min(Min(x, y), z);
  }


  template<class T>
  SIMD_INLINE T real_physics::rpVector3D<T>::getMaxValue() const
  {
    return Max(Max(x, y), z);
  }


  template<class T>
  SIMD_INLINE T real_physics::rpVector3D<T>::dot(const rpVector3D<T>& vector) const
  {
    return (x*vector.x + y*vector.y + z*vector.z);
  }


  template<class T>
  SIMD_INLINE rpVector3D<T> real_physics::rpVector3D<T>::cross( const rpVector3D<T>& vector) const
  {
    return rpVector3D<T>(y * vector.z - z * vector.y,
			             z * vector.x - x * vector.z,
			             x * vector.y - y * vector.x);
  }


  template<class T>
  SIMD_INLINE void real_physics::rpVector3D<T>::normalize()
  {
    T l = length();
    if (l < MACHINE_EPSILON)
    {
	  return;
    }
    x /= l;
    y /= l;
    z /= l;
  }


  template<class T>
  SIMD_INLINE bool real_physics::rpVector3D<T>::operator ==( const rpVector3D<T>& vector) const
  {
    //return (x == vector.x && y == vector.y && z == vector.z);
    return ((Abs(scalar(vector.x - x)) < MACHINE_EPSILON) &&
            (Abs(scalar(vector.y - y)) < MACHINE_EPSILON) &&
            (Abs(scalar(vector.z - z)) < MACHINE_EPSILON));
  }

  template<class T>
  SIMD_INLINE bool real_physics::rpVector3D<T>::operator !=(const rpVector3D<T>& vector) const
  {
    return !(*this == vector);
  }

  template<class T>
  SIMD_INLINE rpVector3D<T>& real_physics::rpVector3D<T>::operator +=(const rpVector3D<T>& vector)
  {
    x += vector.x;
    y += vector.y;
    z += vector.z;
    return *this;
  }

  template<class T>
  SIMD_INLINE rpVector3D<T>& real_physics::rpVector3D<T>::operator -=(const rpVector3D<T>& vector)
  {
    x -= vector.x;
    y -= vector.y;
    z -= vector.z;
    return *this;
  }

  template<class T>
  SIMD_INLINE rpVector3D<T>& real_physics::rpVector3D<T>::operator *=(T number)
  {
    x *= number;
    y *= number;
    z *= number;
    return *this;
  }

  template<class T>
  SIMD_INLINE rpVector3D<T>& real_physics::rpVector3D<T>::operator /=(T number)
  {
    assert(number > std::numeric_limits<T>::epsilon());
    x /= number;
    y /= number;
    z /= number;
    return *this;
  }


  template<class T>
  SIMD_INLINE T& real_physics::rpVector3D<T>::operator [](int index)
  {
    return f[index];
  }

  template<class T>
  SIMD_INLINE const T& real_physics::rpVector3D<T>::operator [](int index) const
  {
    return f[index];
  }

  template<class T>
  SIMD_INLINE rpVector3D<T>& real_physics::rpVector3D<T>::operator =(const rpVector3D<T>& vector)
  {
    if (&vector != this)
      {
		x = vector.x;
		y = vector.y;
		z = vector.z;
      }
    return *this;
  }


  template<class T>
  SIMD_INLINE bool real_physics::rpVector3D<T>::operator <(const rpVector3D<T>& vector) const
  {
    return (x == vector.x ? (y == vector.y ? z < vector.z : y < vector.y) : x < vector.x);
  }

  template<class T>
  SIMD_INLINE rpVector3D<T> real_physics::rpVector3D<T>::min(const rpVector3D<T>& vector1, const rpVector3D<T>& vector2)
  {
    return rpVector3D<T>(Min(vector1.x, vector2.x),
						 Min(vector1.y, vector2.y),
						 Min(vector1.z, vector2.z));
  }

  template<class T>
  SIMD_INLINE rpVector3D<T> real_physics::rpVector3D<T>::max(const rpVector3D<T>& vector1, const rpVector3D<T>& vector2)
  {
    return rpVector3D<T>(Max(vector1.x, vector2.x),
						 Max(vector1.y, vector2.y),
						 Max(vector1.z, vector2.z));
  }


  template<class T>
  SIMD_INLINE rpVector3D<T> real_physics::rpVector3D<T>::zero()
  {
    return rpVector3D<T>(0, 0, 0);
  }



  template<class T>
  SIMD_INLINE rpVector3D<T> rpVector3D<T>::getUnit() const
  {
	  T lengthVector = length();
	  if (lengthVector < MACHINE_EPSILON)
	  {
		  return *this;
	  }
	  // Compute and return the unit vector
	  T lengthInv = T(1.0) / lengthVector;
	  return rpVector3D<T>(x * lengthInv, y * lengthInv, z * lengthInv);
  }



  template<class T>
  SIMD_INLINE rpVector3D<T> rpVector3D<T>::projection(const rpVector3D<T>& o) const
  {
	  T C = dot(o)/o.lengthSquare();
	  return o*C;
  }


  template<class T>
  SIMD_INLINE T rpVector3D<T>::getDistanceTo(const rpVector3D<T>& other) const
  {
     	rpVector3D<T> diff = *this - other;
      	return diff.length();
  }

  template<class T>
  SIMD_INLINE T rpVector3D<T>::getSquaredDistanceTo(const rpVector3D<T>& other) const
  {
	  rpVector3D<T> diff = *this - other;
      	return diff.lengthSquare();
  }


  template<class T>
  SIMD_INLINE T rpVector3D<T>::AngleBetweenVectors(const rpVector3D<T>& Vector2) const
  {
	  rpVector3D<T> Vector1(*this);
	  T dotProduct = Vector1.dot(Vector2);
	  T vectorsMagnitude = (Vector1.length()) * (Vector2.length());
	  T angle = acos(dotProduct / vectorsMagnitude);
     // if( __isnan(angle)) return 0;
	  return (angle);
  }

  template<class T>
  SIMD_INLINE rpVector3D<T> real_physics::rpVector3D<T>::planeNormal( const rpVector3D<T>& V0,
		                                                              const rpVector3D<T>& V1,
  		                                                              const rpVector3D<T>& V2)
  {
	  rpVector3D<T> Norm;
	  rpVector3D<T> E = V1;
	  rpVector3D<T> F = V2;
	  E -= V0;
	  F -= V1;
	  Norm = E.cross(F);
	  Norm.normalize();
	  return Norm;
  }

template<class T>
SIMD_INLINE rpVector3D<T> rpVector3D<T>::clamp(const rpVector3D<T>& vector, T maxLength)
{
	 if (vector.lengthSquare() > maxLength * maxLength)
	 {
	        return vector.getUnit() * maxLength;
	 }

  return vector;
}

template<class T>
SIMD_INLINE rpVector3D<T> rpVector3D<T>::dot3(const rpVector3D<T>& v0, const rpVector3D<T>& v1, const rpVector3D<T>& v2) const
{
	return rpVector3D<T> ( v0.dot(v0), v1.dot(v1), v2.dot(v2));
}



  template<class T>
  SIMD_INLINE void rpVector3D<T>::btPlaneSpace1(const rpVector3D<T>& n, rpVector3D<T>& p, rpVector3D<T>& q)
  {

  	  if (Abs(n[2]) > SIMDSQRT12)
  	    {
  	        // choose p in y-z plane
  	        T a = n[1]*n[1] + n[2]*n[2];
  	        T k = btRecipSqrt(a);
            p[0] = scalar(0);
  	        p[1] = -n[2]*k;
  	        p[2] = n[1]*k;
  	        // set q = n x p
  	        q[0] = a*k;
  	        q[1] = -n[0]*p[2];
  	        q[2] = n[0]*p[1];
  	    }
  	    else
  	    {
  	        // choose p in x-y plane
  	        T a = n[0]*n[0] + n[1]*n[1];
  	        T k = btRecipSqrt(a);
  	        p[0] = -n[1]*k;
  	        p[1] = n[0]*k;
            p[2] = scalar(0);
  	        // set q = n x p
  	        q[0] = -n[2]*p[1];
  	        q[1] = n[2]*p[0];
  	        q[2] = a*k;
  	    }
  }


  template<class T>
   SIMD_INLINE rpVector3D<T> rpVector3D<T>::getOneUnitOrthogonalVector() const
   {
     assert(length() > MACHINE_EPSILON);

     // Get the minimum element of the vector
     rpVector3D<T> vectorAbs(fabs(x), fabs(y), fabs(z));
     int minElement = vectorAbs.getMinAxis();

     if (minElement == 0)
     {
     	return rpVector3D<T>(0.0, -z, y) / SquareRoot(y*y + z*z);
     }
     else if (minElement == 1)
     {
     	return rpVector3D<T>(-z, 0.0, x) / SquareRoot(x*x + z*z);
     }
     else
     {
     	return rpVector3D<T>(-y, x, 0.0) / SquareRoot(x*x + y*y);
     }

   }


  template<class T> const rpVector3D<T> rpVector3D<T>::ZERO(0.0f, 0.0f, 0.0f);
  template<class T> const rpVector3D<T> rpVector3D<T>::X(1.0f, 0.0f, 0.0f);
  template<class T> const rpVector3D<T> rpVector3D<T>::Y(0.0f, 1.0f, 0.0f);
  template<class T> const rpVector3D<T> rpVector3D<T>::Z(0.0f, 0.0f, 1.0f);
  template<class T> const rpVector3D<T> rpVector3D<T>::IDENTITY(1.0f, 1.0f, 1.0f);

} /* namespace real_physics */



#endif /* SOURCE_REAL_PHYSICS_LINEARMATHS_RPVECTOR3D_H_ */
