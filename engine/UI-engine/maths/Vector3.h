#ifndef VECTOR3_H
#define VECTOR3_H

// Libraries
#include <cmath>
#include <cassert>
#include <limits>



#include "../../physics-engine/LinearMaths/mathematics.h"


namespace utility_engine
{

// Class Vector3
// This class represents a 3D vector.
class Vector3
{

    public:



    static const Vector3 NULL_V;
    static const Vector3 X;
    static const Vector3 Y;
    static const Vector3 Z;


        // -------------------- Attributes -------------------- //

        // Components of the vector
        float x, y, z;

        // -------------------- Methods -------------------- //


        Vector3(const real_physics::Vector3 &vector)
        : x(vector.x), y(vector.y), z(vector.z)
        {
        }

        // Constructor
        Vector3(float x=0, float y=0, float z=0) : x(x), y(y), z(z) {}

        // Constructor
        Vector3(const Vector3& vector) : x(vector.x), y(vector.y), z(vector.z) {}

        // Constructor
        ~Vector3() {}

        // = operator
        Vector3& operator=(const Vector3& vector)
        {
            if (&vector != this)
            {
                x = vector.x;
                y = vector.y;
                z = vector.z;
            }
            return *this;
        }


        // = operator
        Vector3& operator=(const real_physics::Vector3& vector)
        {   
            x = vector.x;
            y = vector.y;
            z = vector.z;

            return *this;
        }



        // -------------------- Friends -------------------- //

         // + operator
        friend Vector3 operator+(const Vector3& vector1, const Vector3& vector2)
        {
          return Vector3(vector1.x + vector2.x, vector1.y + vector2.y, vector1.z + vector2.z);
        }

         // - operator
        friend Vector3 operator-(const Vector3& vector1, const Vector3& vector2)
        {
          return Vector3(vector1.x - vector2.x, vector1.y - vector2.y, vector1.z - vector2.z);
        }

        // * operator
        friend Vector3 operator*(const Vector3& vector, float number)
        {
          return Vector3(number * vector.x, number * vector.y, number * vector.z);
        }

        // * operator
        friend Vector3 operator*(float number, const Vector3& vector)
        {
          return Vector3(vector.x * number , vector.y * number , vector.z * number );
        }

        // * operator vector
        friend Vector3 operator*(const Vector3& vector1, const Vector3& vector2)
        {
          return Vector3(vector1.x * vector2.x, vector1.y * vector2.y, vector1.z * vector2.z);
        }

         // ^ operator vector
        friend Vector3 operator^(const Vector3& vector1, const Vector3& vector2)
        {
          return Vector3(vector1.cross(vector2));
        }

        // / operator
        friend Vector3 operator/(const Vector3& vector, float number)
        {
          return Vector3(vector.x / number, vector.y / number, vector.z / number);
        }

        // / operator vector
        friend Vector3 operator/(const Vector3& vector1, const Vector3& vector2)
        {
          return Vector3(vector1.x / vector2.x, vector1.y / vector2.y, vector1.z / vector2.z);
        }


        // - operator this
        Vector3 operator-() const
        {
            return Vector3(-x, -y, -z);
        }

        ///-------------------------------------------------------------///


        // += operator
        Vector3& operator+=(const Vector3 &v)
        {
            x += v.x; y += v.y; z += v.z;
            return *this;
        }

        // -= operator
        Vector3& operator-=(const Vector3 &v)
        {
            x -= v.x; y -= v.y; z -= v.z;
            return *this;
        }

        // == operator
        bool operator==(const Vector3 &v) const
        {
            return x == v.x && y == v.y && z == v.z;
        }

        // != operator
        bool operator!=(const Vector3 &v) const
        {
          return !( *this == v );
        }


        // *= operator
        Vector3 &operator*=(float f)
        {
            x *= f; y *= f; z *= f;
            return *this;
        }


        // /= operator
        Vector3 &operator/=(float f)
        {
            assert(f > std::numeric_limits<float>::epsilon());
            float inv = 1.f / f;
            x *= inv; y *= inv; z *= inv;
            return *this;
        }


        // [] operator
        float &operator[](int i)
        {
            assert(i >= 0 && i <= 2);
            switch (i)
            {
             case 0: return x;
             case 1: return y;
             case 2: return z;
            }

            return z;
        }

        // [] operator
        const float &operator[](int i) const
        {
            assert(i >= 0 && i <= 2);
            switch (i)
            {
             case 0: return x;
             case 1: return y;
             case 2: return z;
            }

            return z;
        }

        float *data()
        {
            return &this->operator [](0);
        }

        const float *data() const
        {
            return &this->operator [](0);
        }

        // Cross product operator
        Vector3 cross(const Vector3 &v) const
        {
            return Vector3(y * v.z - z * v.y,
                           z * v.x - x * v.z,
                           x * v.y - y * v.x);
        }

        // Dot product operator
        float dot(const Vector3 &v) const
        {
            return x * v.x + y * v.y + z * v.z;
        }

        // Normalize the vector and return it
        Vector3 normalize() const
        {
            float l = length();
            if(l < std::numeric_limits<float>::epsilon() )
            {
              assert(false);
            }
            float _x = x / l;
            float _y = y / l;
            float _z = z / l;

            return Vector3(_x , _y , _z);
        }

        bool isNull() const
        {
          return( x == 0. && y == 0. && z == 0. );
        }

        // Clamp the values between 0 and 1
        Vector3 clamp01()
        {
            if (x>1.f) x=1.f;  else if (x<0.f) x=0.f;
            if (y>1.f) y=1.f;  else if (y<0.f) y=0.f;
            if (z>1.f) z=1.f;  else if (z<0.f) z=0.f;
            return *this;
        }

        // Return the squared length of the vector
        float lengthSquared() const { return x*x + y*y + z*z; }
        float length2() const { return x*x + y*y + z*z; }

        // Return the length of the vector
        float length() const { return sqrt(lengthSquared()); }


        float AngleBetweenVectors(const Vector3& Vector2) const
        {
            Vector3 Vector1(*this);
            float dotProduct = Vector1.dot(Vector2);
            float vectorsMagnitude = (Vector1.length()) * (Vector2.length());
            float angle = acos(dotProduct / vectorsMagnitude);
            if( __isnan(angle)) return 0;
            return (angle);
        }

        static float AngleSigned( const Vector3& v1, const Vector3& v2, const Vector3& normal )
        {
            return atan2(normal.dot(v1.cross(v2)), v1.dot(v2));
        }
};



}

#endif
