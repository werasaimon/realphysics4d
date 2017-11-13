#include "Matrix4.h"
#include "Vector2.h"
#include "Vector3.h"
#include "Vector4.h"




using namespace utility_engine;

#define _M_PI 3.14159265359

Matrix4 Matrix4::Rotate(float angle, const Vector3& u)
{

        Matrix4 Rotate;

       // angle = angle / 180.0f * (float)_M_PI;

        Vector3 v = u.normalize();

        float c = 1.0f - cos(angle), s = sin(angle);

        float M[16]= { 1.f , 0.f , 0.f , 0.f ,
                       0.f , 1.f , 0.f , 0.f ,
                       0.f , 0.f , 1.f , 0.f ,
                       0.f , 0.f , 0.f , 1.f };

        M[0] = 1.0f + c * (v.x * v.x - 1.0f);
        M[1] = c * v.x * v.y + v.z * s;
        M[2] = c * v.x * v.z - v.y * s;
        M[4] = c * v.x * v.y - v.z * s;
        M[5] = 1.0f + c * (v.y * v.y - 1.0f);
        M[6] = c * v.y * v.z + v.x * s;
        M[8] = c * v.x * v.z + v.y * s;
        M[9] = c * v.y * v.z - v.x * s;
        M[10] = 1.0f + c * (v.z * v.z - 1.0f);

        Rotate.setDataValue(M);
        return Rotate;
}

Matrix3 Matrix4::getMatrix3x3() const
{
  return Matrix3
  (
     m[0][0] , m[0][1] , m[0][2] ,
     m[1][0] , m[1][1] , m[1][2] ,
     m[2][0] , m[2][1] , m[2][2]
  );
}



Matrix4 Matrix4::Perspective2(float mFieldOfView, float aspect, float mNearPlane, float mFarPlane )
{

    float top = mNearPlane * tan((mFieldOfView / 2.0f) * (float(_M_PI) / 180.0f));
    float bottom = -top;
    float left = bottom * aspect;
    float right = top * aspect;

    float fx = 2.0f * mNearPlane / (right - left);
    float fy = 2.0f * mNearPlane / (top - bottom);
    float fz = -(mFarPlane + mNearPlane) / (mFarPlane - mNearPlane);
    float fw = -2.0f * mFarPlane * mNearPlane / (mFarPlane - mNearPlane);

    // Recompute the projection matrix
    return  Matrix4(fx, 0, 0, 0,
                    0, fy, 0, 0,
                    0, 0, fz, fw,
                    0, 0, -1, 0);

}




//Matrix4 Matrix4::Perspectivee(float fovy, float aspect, float n, float f)
//{
//    mat4x4 Perspective;
//    float coty = 1.0f / tan(fovy * (float)_M_PI / 360.0f);
//    Perspective.M[0] = coty / aspect;
//    Perspective.M[5] = coty;
//    Perspective.M[10] = (n + f) / (n - f);
//    Perspective.M[11] = -1.0f;
//    Perspective.M[14] = 2.0f * n * f / (n - f);
//    Perspective.M[15] = 0.0f;
//    Matrix4 matrix;
//    matrix.setDataValue(Perspective.M);
//    return matrix;
//}




Matrix4 Matrix4::Look(const Vector3& eye, const Vector3& center, const Vector3& up)
{

    Vector3 Z =  (eye- center).normalize();
    Vector3 X =  (up.cross(Z)).normalize();
    Vector3 Y =  (Z.cross(X)).normalize();

    return Matrix4(X.x, X.y, X.z, -X.dot(eye),
                   Y.x, Y.y, Y.z, -Y.dot(eye),
                   Z.x, Z.y, Z.z, -Z.dot(eye),
                   0, 0, 0, 1);

}

Matrix4 Matrix4::Ortho(float left, float right, float bottom, float top, float n, float f)
{
           float M[16];

            M[0] = 1.0f; M[4] = 0.0f; M[8]  = 0.0f; M[12] = 0.0f;
            M[1] = 0.0f; M[5] = 1.0f; M[9]  = 0.0f; M[13] = 0.0f;
            M[2] = 0.0f; M[6] = 0.0f; M[10] = 1.0f; M[14] = 0.0f;
            M[3] = 0.0f; M[7] = 0.0f; M[11] = 0.0f; M[15] = 1.0f;

            M[0] = 2.0f / (right - left);
            M[5] = 2.0f / (top - bottom);
            M[10] = -2.0f / (f - n);
            M[12] = -(right + left) / (right - left);
            M[13] = -(top + bottom) / (top - bottom);
            M[14] = -(f + n) / (f - n);


            Matrix4 Mm;
            Mm.setDataValue(M);
            return Mm;
}


