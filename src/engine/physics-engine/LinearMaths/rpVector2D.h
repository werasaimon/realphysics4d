/*
 * rpVector2D.h
 *
 *  Created on: 15 нояб. 2016 г.
 *      Author: wera
 */

#ifndef SOURCE_REAL_PHYSICS_LINEARMATHS_RPVECTOR2D_H_
#define SOURCE_REAL_PHYSICS_LINEARMATHS_RPVECTOR2D_H_


#include "rpLinearMtah.h"

namespace real_physics
{




  template<class T> class  rpVector2D
  {
  public:


    //---------------- atribute -------------------//

    union
    {
      T f[2];
      struct
      {
        T x;
        T y;
      };
    };

    //---------------------------------------------//


    // -------------------- Methods -------------------- //

    /// Constructor of the class Vector3D
    rpVector2D()
    : x(0.0), y(0.0)
    {
    }

    /// Constructor with arguments
    rpVector2D(T newX, T newY)
    : x(newX), y(newY)
    {
    }

    /// Copy-constructor
    rpVector2D(const rpVector2D<T>& vector)
    : x(vector.x), y(vector.y)
    {
    }

    virtual ~rpVector2D()
    {
    }

    //----------------------------------------------------//

    /// Set all the values of the vector
    void setAllValues(T newX, T newY);

    /// Set the vector to zero
    void setToZero();



    /// Return the length of the vector
    T length() const;

    /// Return the square of the length of the vector
    T lengthSquare() const;


    /// Return the corresponding absolute value vector
    rpVector2D<T> getAbsoluteVector() const;


    /// Return the axis with the minimal value
    int getMinAxis() const;

    /// Return the axis with the maximal value
    int getMaxAxis() const;

    /// Return true if the vector is unit and false otherwise
    bool isUnit() const;

    /// Return true if the current vector is the zero vector
    bool isZero() const;


    //---------------------- orto-normals -----------------------//


    /// Return the corresponding unit vector
    rpVector2D<T>  getUnit() const;

    /// Return one unit orthogonal vector of the current vector
    rpVector2D<T>  getOneUnitOrthogonalVector() const;


    //---------------------- operator method --------------------//

    /// Dot product of two vector
    T dot(const rpVector2D<T>& vector) const;

    /// Croos Product of two vector
    T cross(const rpVector2D<T>& vector) const;


    /// Normalize the vector
    void normalize();

    /// Overloaded operator for the equality condition
    bool operator == (const rpVector2D<T>& vector) const;

    /// Overloaded operator for the is different condition
    bool operator != (const rpVector2D<T>& vector) const;





    /// Overloaded operator for addition with assignment
    rpVector2D<T>& operator += (const rpVector2D<T>& vector);

    /// Overloaded operator for substraction with assignment
    rpVector2D<T>& operator -= (const rpVector2D<T>& vector);


    /// Overloaded operator for multiplication with a number with assignment
    rpVector2D<T>& operator *= (T number);

    /// Overloaded operator for division by a number with assignment
    rpVector2D<T>& operator /= (T number);



    /// Overloaded operator for value access
    T& operator[] (int index);

    /// Overloaded operator for value access
    const T& operator[] (int index) const;



    /// Overloaded operator
    rpVector2D<T>& operator=(const rpVector2D<T>& vector);

    /// Overloaded less than operator for ordering to be used inside std::set for instance
    bool operator<(const rpVector2D<T>& vector) const;


    // -------------------- Friends Method -------------------- //


    friend rpVector2D<T> operator + (const rpVector2D<T>& vector1, const rpVector2D<T>& vector2)
    {
      return rpVector2D<T>(vector1.x + vector2.x,
			   vector1.y + vector2.y);
    }


    friend rpVector2D<T> operator - (const rpVector2D<T>& vector1, const rpVector2D<T>& vector2)
    {
      return rpVector2D<T>(vector1.x - vector2.x,
			   vector1.y - vector2.y);
    }


    friend rpVector2D<T> operator -(const rpVector2D<T>& vector)
    {
      return rpVector2D<T>(-vector.x, -vector.y);
    }


    friend rpVector2D<T> operator * (const rpVector2D<T>& vector, T number)
    {
      return rpVector2D<T>(number * vector.x, number * vector.y);
    }


    friend rpVector2D<T> operator * (T number, const rpVector2D<T>& vector)
    {
      return vector * number;
    }

    friend rpVector2D<T> operator * (const rpVector2D<T>& vector1, const rpVector2D<T>& vector2)
    {
      return rpVector2D<T>(vector1.x * vector2.x, vector1.y * vector2.y);
    }

    friend rpVector2D<T> operator / (const rpVector2D<T>& vector, T number)
    {
      assert(number > MACHINE_EPSILON);
      return rpVector2D<T>(vector.x / number, vector.y / number);
    }

    friend rpVector2D<T> operator/(const rpVector2D<T>& vector1, const rpVector2D<T>& vector2)
    {
      assert(vector2.x > MACHINE_EPSILON);
      assert(vector2.y > MACHINE_EPSILON);
      return rpVector2D<T>(vector1.x / vector2.x , vector1.y / vector2.y);

    }


    //------------------------------- static method-------------------------//

    static const rpVector2D<T> IDENTITY;
    static const rpVector2D<T> ZERO;
    static const rpVector2D<T> X;
    static const rpVector2D<T> Y;


    /// Return a vector taking the minimum components of two vectors
    static rpVector2D<T> min(const rpVector2D<T>& vector1, const rpVector2D<T>& vector2);

    /// Return a vector taking the maximum components of two vectors
    static rpVector2D<T> max(const rpVector2D<T>& vector1, const rpVector2D<T>& vector2);

    /// Return the zero vector
    static rpVector2D<T> zero();


  };



  //------------------------------------------------------------//



  template<class T>
  SIMD_INLINE void rpVector2D<T>::setAllValues(T newX, T newY)
  {
    x = newX;
    y = newY;
  }


  template<class T>
  SIMD_INLINE void rpVector2D<T>::setToZero()
  {
    x = scalar(0);
    y = scalar(0);
  }


  template<class T>
  SIMD_INLINE T rpVector2D<T>::length() const
  {
    return Sqrt(x*x + y*y);
  }


  template<class T>
  SIMD_INLINE T rpVector2D<T>::lengthSquare() const
  {
    return (x*x + y*y);
  }




  template<class T>
  SIMD_INLINE rpVector2D<T> rpVector2D<T>::getAbsoluteVector() const
  {
    return Vector2(Abs(x), Abs(y));
  }


  template<class T>
  SIMD_INLINE int rpVector2D<T>::getMinAxis() const
  {
    return (x < y ? 0 : 1);
  }


  template<class T>
  SIMD_INLINE int rpVector2D<T>::getMaxAxis() const
  {
    return (x < y ? 1 : 0);
  }

  template<class T>
  SIMD_INLINE bool rpVector2D<T>::isUnit() const
  {
    return approxEqual(lengthSquare(), 1.0);
  }

  template<class T>
  SIMD_INLINE bool rpVector2D<T>::isZero() const
  {
    return approxEqual(lengthSquare(), 0.0);
  }


  //----------------------------------------------------------//

  template<class T>
  SIMD_INLINE T rpVector2D<T>::dot(const rpVector2D<T>& vector) const
  {
    return (x*vector.x + y*vector.y);
  }

  template<class T>
  SIMD_INLINE T rpVector2D<T>::cross(const rpVector2D<T> &vector) const
  {
      return (x*vector.y - y*vector.x);
  }

  template<class T>
  SIMD_INLINE void rpVector2D<T>::normalize()
  {
    T l = length();
    if (l < MACHINE_EPSILON)
      {
	return;
      }
    x /= l;
    y /= l;
  }



  template<class T>
  SIMD_INLINE bool rpVector2D<T>::operator == ( const rpVector2D<T>& vector) const
  {
    return (x == vector.x && y == vector.y);
  }

  template<class T>
  SIMD_INLINE bool rpVector2D<T>::operator != (const rpVector2D<T>& vector) const
  {
    return !(*this == vector);
  }

  template<class T>
  SIMD_INLINE rpVector2D<T>& rpVector2D<T>::operator +=(const rpVector2D<T>& vector)
  {
    x += vector.x;
    y += vector.y;
    return *this;
  }

  template<class T>
  SIMD_INLINE rpVector2D<T>& rpVector2D<T>::operator -=(const rpVector2D<T>& vector)
  {
    x -= vector.x;
    y -= vector.y;
    return *this;
  }

  template<class T>
  SIMD_INLINE rpVector2D<T>& rpVector2D<T>::operator *=(T number)
  {
    x *= number;
    y *= number;
    return *this;
  }

  template<class T>
  SIMD_INLINE rpVector2D<T>& rpVector2D<T>::operator /=(T number)
  {
    //assert(number > std::numeric_limits<T>::epsilon());
    x /= number;
    y /= number;
    return *this;
  }

  template<class T>
  SIMD_INLINE T& rpVector2D<T>::operator [](int index)
  {
    return f[index];
  }


  template<class T>
  SIMD_INLINE const T& rpVector2D<T>::operator [](int index) const
  {
    return f[index];
  }


  template<class T>
  SIMD_INLINE rpVector2D<T>& rpVector2D<T>::operator =(const rpVector2D<T>& vector)
  {
    if (&vector != this)
    {
       x = vector.x;
       y = vector.y;
    }
    return *this;
  }


  template<class T>
  SIMD_INLINE bool rpVector2D<T>::operator < (const rpVector2D<T>& vector) const
  {
    return (x == vector.x ? y < vector.y : x < vector.x);
  }


  //----------------------------- static method -------------------------//

  template<class T>
  SIMD_INLINE rpVector2D<T> rpVector2D<T>::min( const rpVector2D<T>& vector1, const rpVector2D<T>& vector2 )
  {
    return rpVector2D<T>(Min(vector1.x, vector2.x),
             Min(vector1.y, vector2.y));
  }

  template<class T>
  SIMD_INLINE rpVector2D<T> rpVector2D<T>::max( const rpVector2D<T>& vector1, const rpVector2D<T>& vector2)
  {
    return rpVector2D<T>(Max(vector1.x, vector2.x),
                         Max(vector1.y, vector2.y));
  }



  template<class T>
  SIMD_INLINE rpVector2D<T> rpVector2D<T>::getUnit() const
  {

  	     T lengthVector = length();
  	    if (lengthVector < MACHINE_EPSILON)
  	    {
  	        return *this;
  	    }

  	    // Compute and return the unit vector
  	    T lengthInv = T(1.0) / lengthVector;
  	    return rpVector2D<T>(x * lengthInv, y * lengthInv);
  }


  template<class T>
  SIMD_INLINE rpVector2D<T> rpVector2D<T>::getOneUnitOrthogonalVector() const
  {
  	T l = length();
  	assert(l > MACHINE_EPSILON);
  	return rpVector2D<T>(-y / l, x / l);
  }




} /* namespace real_physics */



#endif /* SOURCE_REAL_PHYSICS_LINEARMATHS_RPVECTOR2D_H_ */
