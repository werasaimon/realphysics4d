/*
 * rpQuaternion.h
 *
 *  Created on: 15 нояб. 2016 г.
 *      Author: wera
 */

#ifndef SOURCE_REAL_PHYSICS_LINEARMATHS_RPQUATERNION_H_
#define SOURCE_REAL_PHYSICS_LINEARMATHS_RPQUATERNION_H_


// Libraries
#include <cmath>
#include "rpVector3D.h"
#include "rpMatrix3x3.h"


namespace real_physics
{

  template<class T> class rpQuaternion
  {


  public:

    // -------------------- Attributes -------------------- //

    T x;
    T y;
    T z;
    T w;

    // -------------------- Methods -------------------- //

    // Constructor of the class
    rpQuaternion()
    : x(0.0), y(0.0), z(0.0), w(0.0)
    {

    }

    // Constructor with arguments
    rpQuaternion(T newX, T newY, T newZ, T newW)
    :x(newX), y(newY), z(newZ), w(newW)
    {

    }

    // Constructor with the component w and the vector v=(x y z)
    rpQuaternion(T newW, const rpVector3D<T>& v)
    : x(v.x), y(v.y), z(v.z), w(newW)
    {

    }


    // Constructor with the component w and the vector v=(x y z)
    rpQuaternion( const rpVector3D<T>& v , T newW)
    : x(v.x), y(v.y), z(v.z), w(newW)
    {

    }

    /// Constructor which convert Euler angles (in radians) to a quaternion
    rpQuaternion(T angleX, T angleY, T angleZ)
    {
       initWithEulerAngles(angleX, angleY, angleZ);
    }

    /// Constructor which convert Euler angles (in radians) to a quaternion
    rpQuaternion(const rpVector3D<T>& eulerAngles)
    {
      initWithEulerAngles(eulerAngles.x, eulerAngles.y, eulerAngles.z);
    }

    /// Copy-constructor
    rpQuaternion(const rpQuaternion<T>& quaternion)
    :x(quaternion.x), y(quaternion.y), z(quaternion.z), w(quaternion.w)
    {

    }

    /// Create a unit quaternion from a rotation matrix
    rpQuaternion(const rpMatrix3x3<T>& matrix)
    {
    	// Get the trace of the matrix
    	T trace = matrix.getTrace();

    	T r;
    	T s;

    	if (trace < 0.0)
    	{
    		if (matrix[1][1] > matrix[0][0])
    		{
    			if(matrix[2][2] > matrix[1][1])
    			{
    				r = SquareRoot(matrix[2][2] - matrix[0][0] - matrix[1][1] + T(1.0));
    				s = T(0.5) / r;

    				// Compute the quaternion
					x = (matrix[2][0] + matrix[0][2]) * s;
					y = (matrix[1][2] + matrix[2][1]) * s;
					z = T(0.5) * r;
					w = (matrix[1][0] - matrix[0][1]) * s;
    			}
    			else
    			{
    				r = SquareRoot(matrix[1][1] - matrix[2][2] - matrix[0][0] + T(1.0));
    				s = T(0.5) / r;

    				// Compute the quaternion
    				x = (matrix[0][1] + matrix[1][0]) * s;
    				y = T(0.5) * r;
    				z = (matrix[1][2] + matrix[2][1]) * s;
    				w = (matrix[0][2] - matrix[2][0]) * s;
    			}
    		}
    		else if (matrix[2][2] > matrix[0][0])
    		{
    			r = SquareRoot(matrix[2][2] - matrix[0][0] - matrix[1][1] + T(1.0));
    			s = T(0.5) / r;

    			// Compute the quaternion
    			x = (matrix[2][0] + matrix[0][2]) * s;
    			y = (matrix[1][2] + matrix[2][1]) * s;
    			z = T(0.5) * r;
    			w = (matrix[1][0] - matrix[0][1]) * s;
    		}
    		else
    		{
    			r = SquareRoot(matrix[0][0] - matrix[1][1] - matrix[2][2] + T(1.0));
    			s = T(0.5) / r;

    			// Compute the quaternion
    			x = T(0.5) * r;
    			y = (matrix[0][1] + matrix[1][0]) * s;
    			z = (matrix[2][0] - matrix[0][2]) * s;
    			w = (matrix[2][1] - matrix[1][2]) * s;
    		}
    	}
    	else
    	{
    		r = SquareRoot(trace + T(1.0));
    		s = T(0.5) / r;

    		// Compute the quaternion
    		x = (matrix[2][1] - matrix[1][2]) * s;
    		y = (matrix[0][2] - matrix[2][0]) * s;
    		z = (matrix[1][0] - matrix[0][1]) * s;
    		w = T(0.5) * r;
    	}

    }

    /// Destructor
    ~rpQuaternion()
    {

    }

    //-------------------------------------------------------------------//


    /// Set all the values
    void setAllValues(T newX, T newY, T newZ, T newW);

    /// Set the quaternion to zero
    void setToZero();

    /// Set to the identity quaternion
    void setToIdentity();



	/**@brief Return the angle [0, 2Pi] of rotation represented by this quaternion */
	scalar getAngle() const
	{
		scalar s = scalar(2.) * ArcCos(w);
		return s;
	}

    /// Return the radius
    T getW() const
    {
       return w;
    }

    /// Return the vector v=(x y z) of the quaternion
    rpVector3D<T> getVectorV() const;

    /// Return the length of the quaternion
    T length() const;

    /// Return the square of the length of the quaternion
    T lengthSquare() const;



    /// Normalize the quaternion
    void normalize();

    /// Inverse the quaternion
    void inverse();


    /// Return the unit quaternion
    rpQuaternion<T> getUnit() const;

    /// Return the conjugate quaternion
    rpQuaternion<T> getConjugate() const;

    /// Return the inverse of the quaternion
    rpQuaternion<T> getInverse() const;



    /// Return the orientation matrix corresponding to this quaternion
    rpMatrix3x3<T> getMatrix() const;


    /// Compute the rotation angle (in radians) and the rotation axis
    void getRotationAngleAxis(T& angle, rpVector3D<T>& axis) const;




    /// The angle between the vectors is simple to find: the dot product gives its cosine.
    /// The needed axis is also simple to find: it’s the cross product of the two vectors.
    void orientateBetweenAngleAxis(const rpVector3D<T>& NormStartVec, const rpVector3D<T>& NormEndVec);


    /// The angle between the vectors is simple to find: the dot product gives its cosine.
    /// The needed axis is also simple to find: it’s the cross product of the two vectors.
    static rpQuaternion<T> rotationBetweenVectors(  rpVector3D<T> start ,  rpVector3D<T> dest );


    /// Compute the spherical linear interpolation between two quaternions
    static rpQuaternion<T> slerp(const rpQuaternion<T>& quaternion1, const rpQuaternion<T>& quaternion2, T t);

    /// Return the identity quaternion
    static rpQuaternion<T> identity();






    /// Dot product between two quaternions
    T dot(const rpQuaternion<T>& quaternion) const;


    /// Overloaded operator for the addition
    rpQuaternion<T> operator  +  (const rpQuaternion<T>& quaternion) const;

    /// Overloaded operator for the substraction
    rpQuaternion<T> operator  -  (const rpQuaternion<T>& quaternion) const;

    /// Overloaded operator for addition with assignment
    rpQuaternion<T>& operator += (const rpQuaternion<T>& quaternion);

    /// Overloaded operator for substraction with assignment
    rpQuaternion<T>& operator -= (const rpQuaternion<T>& quaternion);






    /// Overloaded operator for the multiplication with a constant
    rpQuaternion<T> operator*(T nb) const;

    /// Overloaded operator for the multiplication
    rpQuaternion<T> operator*(const rpQuaternion<T>& quaternion) const;


     /// Overloaded operator for the multiplication dimensional dot product
    rpQuaternion<T> operator ^ (const rpQuaternion<T> &OtherQuat) const
    {
        rpQuaternion<T> Temp = *this;
        rpQuaternion<T>quat = (OtherQuat);
        return (Temp = Temp * quat.getConjugate());
    }

    /// Overloaded operator for the multiplication with a vector
    rpVector3D<T>   operator*(const rpVector3D<T>& point) const;

    /// Overloaded operator for assignment
    rpQuaternion<T>& operator=(const rpQuaternion<T>& quaternion);

    /// Overloaded operator for equality condition
    bool operator==(const rpQuaternion<T>& quaternion) const;



 // private:

    /// Initialize the quaternion using Euler angles
    void initWithEulerAngles(T angleX, T angleY, T angleZ)
    {
    	T angle = angleX * T(0.5);
        const T sinX = Sin(angle);
        const T cosX = Cos(angle);

    	angle = angleY * T(0.5);
        const T sinY = Sin(angle);
        const T cosY = Cos(angle);

    	angle = angleZ * T(0.5);
        const T sinZ = Sin(angle);
        const T cosZ = Cos(angle);

    	const T cosYcosZ = cosY * cosZ;
    	const T sinYcosZ = sinY * cosZ;
    	const T cosYsinZ = cosY * sinZ;
    	const T sinYsinZ = sinY * sinZ;

    	x = sinX * cosYcosZ - cosX * sinYsinZ;
    	y = cosX * sinYcosZ + sinX * cosYsinZ;
    	z = cosX * cosYsinZ - sinX * sinYcosZ;
    	w = cosX * cosYcosZ + sinX * sinYsinZ;

    	// Normalize the quaternion
    	normalize();
    }


    rpQuaternion<T>  createRotation( rpVector3D<T> RotAxis , T AngleInRads )
    {
        scalar Thetad2 = AngleInRads * scalar(0.5);
        scalar SinTd2 = Sin(Thetad2);
        scalar CosTd2 = Cos(Thetad2);

        // Normalize the quaternion axis
        RotAxis.normalize();

        w = CosTd2;
        x = (SinTd2 * RotAxis).x;
        y = (SinTd2 * RotAxis).y;
        z = (SinTd2 * RotAxis).z;

        return *this;
    }



    rpVector3D<T> getEulerAngles() const
    {
        // Store the Euler angles in radians
        rpVector3D<T> pitchYawRoll;

        rpQuaternion<T> q(*this);

        T sqw = q.w * q.w;
        T sqx = q.x * q.x;
        T sqy = q.y * q.y;
        T sqz = q.z * q.z;

        // If quaternion is normalised the unit is one, otherwise it is the correction factor
        T unit = sqx + sqy + sqz + sqw;
        T test = q.x * q.y + q.z * q.w;

        if (test > 0.4999f * unit)                                // 0.4999f OR 0.5f - EPSILON
        {
            // Singularity at north pole
            pitchYawRoll.y = 2.f * atan2(q.x, q.w);               // Yaw
            pitchYawRoll.x = Pi() * 0.5f;                         // Pitch
            pitchYawRoll.z = 0.f;                                 // Roll
            return pitchYawRoll;
        }
        else if (test < -0.4999f * unit)                          // -0.4999f OR -0.5f + EPSILON
        {
            // Singularity at south pole
            pitchYawRoll.y = -2.f * atan2(q.x, q.w);              // Yaw
            pitchYawRoll.x = -Pi() * 0.5f;                        // Pitch
            pitchYawRoll.z = 0.f;                                 // Roll
            return pitchYawRoll;
        }
        else
        {
            pitchYawRoll.y = atan2(2.f * q.y * q.w - 2.f * q.x * q.z,  sqx - sqy - sqz + sqw);      // Yaw
            pitchYawRoll.x =  asin(2.f * test / unit);                                              // Pitch
            pitchYawRoll.z = atan2(2.f * q.x * q.w - 2.f * q.y * q.z, -sqx + sqy - sqz + sqw);      // Roll
        }

        return pitchYawRoll;
    }



   rpQuaternion<T> lookRotation(rpVector3D<T>& lookAt, rpVector3D<T>& upDirection)
   {
        rpVector3D<T> forward = lookAt.getUnit();
        rpVector3D<T> right   = upDirection.getUnit().cross(forward);
        rpVector3D<T> up      = forward.cross(right);

        rpQuaternion<T> ret;
        ret.w = sqrtf(1.0f + right.x + up.y + forward.z) * 0.5f;
        float w4_recip = 1.0f / (4.0f * ret.w);
        ret.x = (forward.y - up.z) * w4_recip;
        ret.y = (right.z - forward.x) * w4_recip;
        ret.z = (up.x - right.y) * w4_recip;

        return ret;
    }


    //---------------------- friendships -------------------------//
     //friend rpMatrix3x3<T> setQuaternion(const rpQuaternion<T>& Quat);


  };

  template<class T>
  SIMD_INLINE void real_physics::rpQuaternion<T>::setAllValues(T newX, T newY, T newZ, T newW)
  {
    x = newX;
    y = newY;
    z = newZ;
    w = newW;
  }

  template<class T>
  SIMD_INLINE void real_physics::rpQuaternion<T>::setToZero()
  {
    x = 0;
    y = 0;
    z = 0;
    w = 0;
  }

  template<class T>
  SIMD_INLINE void real_physics::rpQuaternion<T>::setToIdentity()
  {
    x = 0;
    y = 0;
    z = 0;
    w = 1;
  }

  template<class T>
  SIMD_INLINE rpVector3D<T> real_physics::rpQuaternion<T>::getVectorV() const
  {
    // Return the vector v
    return rpVector3D<T>(x, y, z);
  }

  template<class T>
  SIMD_INLINE T real_physics::rpQuaternion<T>::length() const
  {
    return SquareRoot( x*x + y*y + z*z + w*w );
  }

  template<class T>
  SIMD_INLINE T real_physics::rpQuaternion<T>::lengthSquare() const
  {
    return x*x + y*y + z*z + w*w;
  }

  template<class T>
  SIMD_INLINE void real_physics::rpQuaternion<T>::normalize()
  {
    T l = length();
    // Check if the length is not equal to zero

    assert (l > MACHINE_EPSILON );

    x /= l;
    y /= l;
    z /= l;
    w /= l;

  }

  template<class T>
  SIMD_INLINE void real_physics::rpQuaternion<T>::inverse()
  {

    // Get the square length of the quaternion
    T lengthSquareQuaternion = lengthSquare();

    assert (lengthSquareQuaternion > MACHINE_EPSILON);

    // Compute and return the inverse quaternion
    x /= -lengthSquareQuaternion;
    y /= -lengthSquareQuaternion;
    z /= -lengthSquareQuaternion;
    w /=  lengthSquareQuaternion;

  }

  template<class T>
  SIMD_INLINE rpQuaternion<T> real_physics::rpQuaternion<T>::getUnit() const
  {
    T lengthQuaternion = length();

    // Check if the length is not equal to zero
    assert (lengthQuaternion > MACHINE_EPSILON);

    // Compute and return the unit quaternion
    return rpQuaternion<T>(x / lengthQuaternion,
						   y / lengthQuaternion,
						   z / lengthQuaternion,
						   w / lengthQuaternion);
  }

  template<class T>
  SIMD_INLINE rpQuaternion<T> real_physics::rpQuaternion<T>::getConjugate() const
  {
    return rpQuaternion<T>(-x, -y, -z, w);
  }

  template<class T>
  SIMD_INLINE rpQuaternion<T> real_physics::rpQuaternion<T>::getInverse() const
  {
    T lengthSquareQuaternion = lengthSquare();

    assert (lengthSquareQuaternion > MACHINE_EPSILON);

    // Compute and return the inverse quaternion
    return rpQuaternion<T>(-x / lengthSquareQuaternion,
						   -y / lengthSquareQuaternion,
						   -z / lengthSquareQuaternion,
						    w / lengthSquareQuaternion);
  }



  template<class T>
  SIMD_INLINE rpQuaternion<T> real_physics::rpQuaternion<T>::identity()
  {
    return rpQuaternion<T>(0.0, 0.0, 0.0, 1.0);
  }

  template<class T>
  SIMD_INLINE T real_physics::rpQuaternion<T>::dot(const rpQuaternion<T>& quaternion) const
  {
    return (x*quaternion.x +
			y*quaternion.y +
			z*quaternion.z +
			w*quaternion.w);
  }

  template<class T>
  SIMD_INLINE rpQuaternion<T> real_physics::rpQuaternion<T>::operator +(const rpQuaternion<T>& quaternion) const
  {
    // Return the result quaternion
    return rpQuaternion<T>(x + quaternion.x,
						   y + quaternion.y,
						   z + quaternion.z,
						   w + quaternion.w);
  }

  template<class T>
  SIMD_INLINE rpQuaternion<T> real_physics::rpQuaternion<T>::operator -(const rpQuaternion<T>& quaternion) const
  {
    // Return the result of the substraction
    return rpQuaternion<T>(x - quaternion.x,
						   y - quaternion.y,
						   z - quaternion.z,
						   w - quaternion.w);
  }


  template<class T>
  SIMD_INLINE rpQuaternion<T>& real_physics::rpQuaternion<T>::operator +=(const rpQuaternion<T>& quaternion)
  {
    x += quaternion.x;
    y += quaternion.y;
    z += quaternion.z;
    w += quaternion.w;
    return *this;
  }

  template<class T>
  SIMD_INLINE rpQuaternion<T>& real_physics::rpQuaternion<T>::operator -=(const rpQuaternion<T>& quaternion)
  {
    x -= quaternion.x;
    y -= quaternion.y;
    z -= quaternion.z;
    w -= quaternion.w;
    return *this;
  }


  template<class T>
  SIMD_INLINE rpQuaternion<T> real_physics::rpQuaternion<T>::operator *(T nb) const
  {
    return rpQuaternion<T>(nb * x, nb * y, nb * z, nb * w);
  }

  template<class T>
  SIMD_INLINE rpQuaternion<T> real_physics::rpQuaternion<T>::operator *(const rpQuaternion<T>& quaternion) const
  {
      return rpQuaternion<T> (w * quaternion.w - getVectorV().dot(quaternion.getVectorV()),
                              w * quaternion.getVectorV() + quaternion.w * getVectorV() +
                              getVectorV().cross(quaternion.getVectorV()));
  }

  template<class T>
  SIMD_INLINE rpVector3D<T> real_physics::rpQuaternion<T>::operator *(const rpVector3D<T>& point) const
  {
    rpQuaternion<T>  p(point.x, point.y, point.z, 0.0);
    return (((*this) * p) * getConjugate()).getVectorV();
  }

  template<class T>
  SIMD_INLINE rpQuaternion<T>& real_physics::rpQuaternion<T>::operator =(const rpQuaternion<T>& quaternion)
  {
    // Check for self-assignment
	  if (this != &quaternion)
	  {
		  x = quaternion.x;
		  y = quaternion.y;
		  z = quaternion.z;
		  w = quaternion.w;
	  }
    // Return this quaternion
    return *this;
  }

  template<class T>
  SIMD_INLINE void rpQuaternion<T>::orientateBetweenAngleAxis( const rpVector3D<T>& NormStartVec, const rpVector3D<T>& NormEndVec)
  {

      scalar CosTheta = NormStartVec.dot(NormEndVec);

      scalar CosTd2Sq = (CosTheta + 1.0f) * 0.5f;
      scalar CosTd2 = SquareRoot(CosTd2Sq);
      scalar SinTd2 = SquareRoot(1.0f - CosTd2Sq);

      rpVector3D<T> RotAxis;

      if (CosTheta < -1 + 0.001f)								//very nearly opposite
      {
          rpVector3D<T> NonParraVec(0.0f, 0.0f, 1.0f);//Get a wsVector3 thats not aligned with the startvec

          RotAxis = NonParraVec.cross(NormStartVec);

          if (RotAxis.length2() < 0.01f)		//Still aligned
          {
              NonParraVec = rpVector3D<T>(1.0f, 0.0f, 0.0f);//This one must be OK then
              RotAxis = NonParraVec.cross(NormStartVec);
          }

          RotAxis.normalize();
          createRotation(RotAxis,T(180.0f));
      }
      else
      {
          RotAxis = NormStartVec.cross(NormEndVec);
      }

      if (SinTd2 < 0.00004f)			//wsVector3s lined up, no need for an axis
      {
          RotAxis = rpVector3D<T>(0.0f, 0.0f, 0.0f);	//No axis needed if no rotation
      }
      else
      {
          RotAxis.normalize();

      }

      w = CosTd2;
      x = (RotAxis).x;
      y = (RotAxis).y;
      z = (RotAxis).z;

//      T s = CosTd2Sq;
//      T invs = 1.0 / s;

//      w = s * 0.5f;
//      x = RotAxis.x * invs;
//      y = RotAxis.y * invs;
//      z = RotAxis.z * invs;


  }


  /// The angle between the vectors is simple to find: the dot product gives its cosine.
  /// The needed axis is also simple to find: it’s the cross product of the two vectors.
  template<class T>
  SIMD_INLINE  rpQuaternion<T> rpQuaternion<T>::rotationBetweenVectors(  rpVector3D<T> start ,  rpVector3D<T> dest )
  {

      //start.normalize();
      //dest.normalize();

      T cosTheta = start.dot(dest);
      rpVector3D<T> rotationAxis;

      if (cosTheta < -1 + 0.0001f)
      {
          // special case when vectors in opposite directions:
          // there is no "ideal" rotation axis
          // So guess one; any will do as long as it's perpendicular to start
          rotationAxis = rpVector3D<T>::Z.cross(start);
          if (rotationAxis.length2() < T(0.0001) )
          {
              // bad luck, they were parallel, try again!
              rotationAxis = rpVector3D<T>::X.cross(start);
          }

          rotationAxis.normalize();
          static rpQuaternion<T> q;
          return q.createRotation(rotationAxis,T(180.0f));
      }

      rotationAxis = start.cross(dest);
      //rotationAxis = rotationAxis.normalize();



      T s = Sqrt( (1+cosTheta)*2 );
      T invs = 1.0 / s;

      return rpQuaternion<T>
      (
        rotationAxis.x * invs,
        rotationAxis.y * invs,
        rotationAxis.z * invs,
        s * 0.5f
      );

  }

  template<class T>
  SIMD_INLINE bool real_physics::rpQuaternion<T>::operator ==(const rpQuaternion<T>& quaternion) const
  {

    return (x == quaternion.x &&
            y == quaternion.y &&
            z == quaternion.z &&
            w == quaternion.w);
  }





  template<class T>
  rpMatrix3x3<T> real_physics::rpQuaternion<T>::getMatrix() const
  {

    T nQ = x*x + y*y + z*z + w*w;
    T s = 0.0;

    if(nQ > 0.0)
    {
     	s = T(2.0) / nQ;
    }

    // Computations used for optimization (less multiplications)
    T xs  = x*s;
    T ys  = y*s;
    T zs  = z*s;
    T wxs = w*xs;
    T wys = w*ys;
    T wzs = w*zs;
    T xxs = x*xs;
    T xys = x*ys;
    T xzs = x*zs;
    T yys = y*ys;
    T yzs = y*zs;
    T zzs = z*zs;

    // Create the matrix corresponding to the quaternion
    return rpMatrix3x3<T>(T(1.0) - yys - zzs, xys-wzs, xzs + wys,
  						  xys + wzs, T(1.0) - xxs - zzs, yzs-wxs,
  						  xzs-wys, yzs + wxs, T(1.0) - xxs - yys);
  }





} /* namespace real_physics */



#endif /* SOURCE_REAL_PHYSICS_LINEARMATHS_RPQUATERNION_H_ */
