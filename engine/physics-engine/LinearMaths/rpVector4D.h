#ifndef RPVECTOR4D_H
#define RPVECTOR4D_H

#include "rpLinearMtah.h"


namespace real_physics
{


template<class T> class  rpVector4D
{
public:


  //---------------- atribute -------------------//

  union
  {
    T f[4];
    struct
    {
          T x;
          T y;
          T z;
          T w;
    };
  };

  //---------------------------------------------//


  // -------------------- Methods -------------------- //

  // Constructor
  rpVector4D(T x=0, T y=0, T z=0, T w=0)
      : x(x), y(y), z(z), w(w)
  {

  }

  // Constructor
  rpVector4D(const rpVector4D<T>& vector)
    : x(vector.x), y(vector.y), z(vector.z), w(vector.w)
  {

  }

  // + operator
  rpVector4D<T> operator+( const rpVector4D<T>& v) const
  {
      return rpVector4D<T>(x + v.x, y + v.y, z + v.z, w + v.w);
  }

  // += operator
  rpVector4D<T>& operator+=(const rpVector4D<T> &v)
  {
      x += v.x;
      y += v.y;
      z += v.z;
      w += v.w;
      return *this;
  }

  // - operator
  rpVector4D<T> operator-(const rpVector4D<T> &v) const
  {
      return rpVector4D<T>(x - v.x, y - v.y, z - v.z, w - v.w);
  }

  // -= operator
  rpVector4D<T>& operator-=(const rpVector4D<T> &v)
  {
      x -= v.x; y -= v.y; z -= v.z, w -=v.w;
      return *this;
  }

  // = operator
  rpVector4D<T>& operator=(const rpVector4D<T>& vector)
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
  bool operator==(const rpVector4D<T> &v) const
  {
      return x == v.x &&
             y == v.y &&
             z == v.z &&
             w == v.w;
  }

  // * operator
  rpVector4D<T> operator*(T f) const
  {
      return rpVector4D<T>(f*x, f*y, f*z, f*w);
  }

  // *= operator
  rpVector4D<T> &operator*=(T f)
  {
      x *= f; y *= f; z *= f; w *= f;
      return *this;
  }

  // / operator
  rpVector4D<T> operator/(T f) const
  {
      assert(f!=0);
      T inv = 1.f / f;
      return rpVector4D<T>(x * inv, y * inv, z * inv, w * inv);
  }

  // /= operator
  rpVector4D<T> &operator/=(T f)
  {
      assert(f!=0);
      T inv = 1.f / f;
      x *= inv;
      y *= inv;
      z *= inv;
      w *= inv;
      return *this;
  }

  // - operator
  rpVector4D<T> operator-() const
  {
      return rpVector4D<T>(-x, -y, -z, -w);
  }

  // [] operator
  T &operator[](int i)
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


  T *data()
  {
      return &this->operator [](0);
  }

  const T *data() const
  {
      return &this->operator [](0);
  }


  // Dot product operator
  T dot(const rpVector4D<T> &v) const
  {
      return x * v.x + y * v.y + z * v.z + w * v.w;
  }


  // Cross product operator
  void cross(const rpVector4D<T> &v)
  {
      rpVector4D<T> result;

      result.x = y * v.z - z * v.y;
      result.y = z * v.x - x * v.z;
      result.z = x * v.y - y * v.x;
      result.w = 0;

      *this = result;
  }


  // Cross product operator
  rpVector4D<T> cross(const rpVector4D<T>& b , const rpVector4D<T>& c)
  {

      //Precompute some 2x2 matrix determinants for speed
       T Pxy = b.x*c.y - c.x*b.y;
       T Pxz = b.x*c.z - c.x*b.z;
       T Pxw = b.x*c.w - c.x*b.w;
       T Pyz = b.y*c.z - c.y*b.z;
       T Pyw = b.y*c.w - c.y*b.w;
       T Pzw = b.z*c.w - c.z*b.w;

        return rpVector4D<T>
        (
           y*Pzw - z*Pyw + w*Pyz,    //Note the lack of 'x' in this line
           z*Pxw - x*Pzw - w*Pxz,    //y, Etc.
           x*Pyw - y*Pxw + w*Pxy,
           y*Pxz - x*Pyz - z*Pxy
        );
  }

  // Multiply two vectors by their components
  rpVector4D<T> componentMul(const rpVector4D<T> &v) const
  {
      return rpVector4D<T>(x * v.x, y * v.y, z * v.z, w * v.w);
  }

  // Return the squared length of the vector
  T lengthSquared() const { return x * x + y * y + z * z + w * w; }

  // Return the length of the vector
  T length() const { return sqrt(lengthSquared()); }


  // Normalize the vector and return it
  rpVector4D<T> normalize() const
  {
      T l = length();
      if(l < std::numeric_limits<T>::epsilon() )
      {
        assert(false);
      }
      T _x = x / l;
      T _y = y / l;
      T _z = z / l;
      T _w = w / l;

      return rpVector4D<T>(_x , _y , _z , _w);
  }


  rpVector4D<T> BuildPlan(const rpVector4D<T>& p_point1, const rpVector4D<T>& p_normal)
  {
     rpVector4D<T> normal, res;
     normal = p_normal.normalize();
     res.w = normal.dot(p_point1);
     res.x = normal.x;
     res.y = normal.y;
     res.z = normal.z;
     return res;
  }




};


}

#endif // RPVECTOR4D_H
