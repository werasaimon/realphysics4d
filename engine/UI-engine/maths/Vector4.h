#ifndef VECTOR4_H
#define VECTOR4_H

// Libraries
#include <math.h>
#include <assert.h>



namespace utility_engine
{


// Class Vector4
// This class represents a 4D vector.
class Vector4
{

    public:

        // -------------------- Attributes -------------------- //

        // Components of the vector
        union
        {
            struct {  float x, y, z, w; };
            float data[4];
        };


        // -------------------- Methods -------------------- //

        // Constructor
        Vector4(float x=0, float y=0, float z=0, float w=0) : x(x), y(y), z(z), w(w) {}

        // Constructor
        Vector4(const Vector4& vector) : x(vector.x), y(vector.y), z(vector.z), w(vector.w) {}

        // + operator
        Vector4 operator+(const Vector4 &v) const
        {
            return Vector4(x + v.x, y + v.y, z + v.z, w + v.w);
        }

        // += operator
        Vector4& operator+=(const Vector4 &v)
        {
            x += v.x;
            y += v.y;
            z += v.z;
            w += v.w;
            return *this;
        }

        // - operator
        Vector4 operator-(const Vector4 &v) const
        {
            return Vector4(x - v.x, y - v.y, z - v.z, w - v.w);
        }

        // -= operator
        Vector4& operator-=(const Vector4 &v)
        {
            x -= v.x; y -= v.y; z -= v.z, w -=v.w;
            return *this;
        }

        // = operator
        Vector4& operator=(const Vector4& vector)
        {
            if (&vector != this)
            {
                x = vector.x;
                y = vector.y;
                z = vector.z;
                w = vector.w;
            }
            return *this;
        }

        // == operator
        bool operator==(const Vector4 &v) const
        {
            return x == v.x &&
                   y == v.y &&
                   z == v.z &&
                   w == v.w;
        }

        // * operator
        Vector4 operator*(float f) const
        {
            return Vector4(f*x, f*y, f*z, f*w);
        }

        // *= operator
        Vector4 &operator*=(float f)
        {
            x *= f; y *= f; z *= f; w *= f;
            return *this;
        }

        // / operator
        Vector4 operator/(float f) const
        {
            assert(f!=0);
            float inv = 1.f / f;
            return Vector4(x * inv, y * inv, z * inv, w * inv);
        }

        // /= operator
        Vector4 &operator/=(float f)
        {
            assert(f!=0);
            float inv = 1.f / f;
            x *= inv;
            y *= inv;
            z *= inv;
            w *= inv;
            return *this;
        }

        // - operator
        Vector4 operator-() const
        {
            return Vector4(-x, -y, -z, -w);
        }

        // [] operator
        float &operator[](int i)
        {
            assert(i >= 0 && i <= 3);
            switch (i)
            {
             case 0: return x;
             case 1: return y;
             case 2: return z;
             case 3: return w;
            }
            return w;
        }

        // Dot product operator
        float dot(const Vector4 &v) const
        {
            return x * v.x + y * v.y + z * v.z + w * v.w;
        }

        // Multiply two vectors by their components
        Vector4 componentMul(const Vector4 &v) const
        {
            return Vector4(x * v.x, y * v.y, z * v.z, w * v.w);
        }

        // Clamp the values between 0 and 1
        Vector4 clamp01()
        {
            if (x>1.f) x=1.f; else if (x<0.f) x=0.f;
            if (y>1.f) y=1.f; else if (y<0.f) y=0.f;
            if (z>1.f) z=1.f; else if (z<0.f) z=0.f;
            if (w>1.f) w=1.f; else if (w<0.f) w=0.f;
            return *this;
        }

        // Return the squared length of the vector
        float lengthSquared() const { return x * x + y * y + z * z + w * w; }

        // Return the length of the vector
        float length() const { return sqrt(lengthSquared()); }


        // Normalize the vector and return it
        Vector4 normalize() const
        {
            float l = length();
            if(l < std::numeric_limits<float>::epsilon() )
            {
              assert(false);
            }
            float _x = x / l;
            float _y = y / l;
            float _z = z / l;
            float _w = w / l;

            return Vector4(_x , _y , _z , _w);
        }


        Vector4 BuildPlan(const Vector4& p_point1, const Vector4& p_normal)
        {
           Vector4 normal, res;
           normal = p_normal.normalize();
           res.w = normal.dot(p_point1);
           res.x = normal.x;
           res.y = normal.y;
           res.z = normal.z;
           return res;
     }
};

}



#endif //_VECTOR4_H
