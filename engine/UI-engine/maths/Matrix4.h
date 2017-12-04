#ifndef MATRIX4_H
#define MATRIX4_H

// Libraries
#include <math.h>
#include <assert.h>
#include <iostream>
#include "Vector3.h"
#include "Vector4.h"
#include "Matrix3.h"


namespace utility_engine
{


static const float ZPI = 3.14159265358979323846f;
static const float RAD2DEG = 1.0; //(180.f / ZPI);
static const float DEG2RAD = 1.0; //(ZPI / 180.f);




class Vector3;
class Vector4;

class  Matrix4;
static Matrix4 dotMatrix( Matrix4 Matrix1 , Matrix4 Matrix2);
// Class Matrix4
// This class represents a 4x4 matrix
class Matrix4
{

   public:


     static Matrix4   Perspective2(float mFieldOfView, float aspect, float mNearPlane, float mFarPlane);
     static Matrix4   Perspectivee(float fovy, float aspect, float n, float f);
     static Matrix4   Look(const Vector3 &eye, const Vector3 &center, const Vector3 &up);
     static Matrix4   Ortho(float left, float right, float bottom, float top, float n, float f);
     static Matrix4   Rotate(float angle, const Vector3& u);
        // -------------------- Attributes -------------------- //

        // Elements of the matrix     
         union
         {
             float m[4][4];
             float m16[16];
 ///////////////////////////////////////////////////////////////////////////////////////////////////
             struct
             {
                 Vector4 right, up, dir, position;
             } v;

             Vector4 component[4];
         };

        // -------------------- Methods -------------------- //

        // Constructor
        Matrix4(void)
        {
            m[0][0] = 0.0; m[0][1] = 0.0; m[0][2] = 0.0;  m[0][3] = 0.0;
            m[1][0] = 0.0; m[1][1] = 0.0; m[1][2] = 0.0;  m[1][3] = 0.0;
            m[2][0] = 0.0; m[2][1] = 0.0; m[2][2] = 0.0;  m[2][3] = 0.0;
            m[3][0] = 0.0; m[3][1] = 0.0; m[3][2] = 0.0;  m[3][3] = 0.0;
        }

        // Constructor
        Matrix4(float m_00, float m_01, float m_02, float m_03,
                float m_10, float m_11, float m_12, float m_13,
                float m_20, float m_21, float m_22, float m_23,
                float m_30, float m_31, float m_32, float m_33)
        {
            m[0][0] = m_00; m[0][1] = m_01; m[0][2] = m_02;  m[0][3] = m_03;
            m[1][0] = m_10; m[1][1] = m_11; m[1][2] = m_12;  m[1][3] = m_13;
            m[2][0] = m_20; m[2][1] = m_21; m[2][2] = m_22;  m[2][3] = m_23;
            m[3][0] = m_30; m[3][1] = m_31; m[3][2] = m_32;  m[3][3] = m_33;
        }

        // Constructor
        Matrix4(float n[4][4])
        {
            m[0][0]=n[0][0]; m[0][1]=n[0][1]; m[0][2]=n[0][2]; m[0][3]=n[0][3];
            m[1][0]=n[1][0]; m[1][1]=n[1][1]; m[1][2]=n[1][2]; m[1][3]=n[1][3];
            m[2][0]=n[2][0]; m[2][1]=n[2][1]; m[2][2]=n[2][2]; m[2][3]=n[2][3];
            m[3][0]=n[3][0]; m[3][1]=n[3][1]; m[3][2]=n[3][2]; m[3][3]=n[3][3];
        }

        // Constructor
        Matrix4(const Vector3& a1, const Vector3& a2, const Vector3& a3)
        {
            m[0][0] = a1.x; m[0][1] = a2.x; m[0][2] = a3.x;  m[0][3] = 0.f;
            m[1][0] = a1.y; m[1][1] = a2.y; m[1][2] = a3.y;  m[1][3] = 0.f;
            m[2][0] = a1.z; m[2][1] = a2.z; m[2][2] = a3.z;  m[2][3] = 0.f;
            m[3][0] = 0.f;  m[3][1] = 0.f;  m[3][2] = 0.f;   m[3][3] = 1.f;
        }

        // Constructor
        Matrix4(const Vector4& a1, const Vector4& a2, const Vector4& a3)
        {
            m[0][0] = a1.x; m[0][1] = a2.x; m[0][2] = a3.x;  m[0][3] = 0.f;
            m[1][0] = a1.y; m[1][1] = a2.y; m[1][2] = a3.y;  m[1][3] = 0.f;
            m[2][0] = a1.z; m[2][1] = a2.z; m[2][2] = a3.z;  m[2][3] = 0.f;
            m[3][0] = a1.w; m[3][1] = a2.w; m[3][2] = a3.w;  m[3][3] = 1.f;
        }

        // Constructor
        Matrix4(const Matrix4& matrix)
        {

            setAllValues(matrix.m[0][0], matrix.m[0][1], matrix.m[0][2], matrix.m[0][3],
                         matrix.m[1][0], matrix.m[1][1], matrix.m[1][2], matrix.m[1][3],
                         matrix.m[2][0], matrix.m[2][1], matrix.m[2][2], matrix.m[2][3],
                         matrix.m[3][0], matrix.m[3][1], matrix.m[3][2], matrix.m[3][3]);
        }

        // + operator
        Matrix4 operator+(const Matrix4 &n) const
        {
            return Matrix4(m[0][0]+n.m[0][0], m[0][1]+n.m[0][1], m[0][2]+n.m[0][2], m[0][3]+n.m[0][3],
                           m[1][0]+n.m[1][0], m[1][1]+n.m[1][1], m[1][2]+n.m[1][2], m[1][3]+n.m[1][3],
                           m[2][0]+n.m[2][0], m[2][1]+n.m[2][1], m[2][2]+n.m[2][2], m[2][3]+n.m[2][3],
                           m[3][0]+n.m[3][0], m[3][1]+n.m[3][1], m[3][2]+n.m[3][2], m[3][3]+n.m[3][3]);
        }

        // += operator
        Matrix4& operator+=(const Matrix4 &n)
        {
            m[0][0]+=n.m[0][0]; m[0][1]+=n.m[0][1]; m[0][2]+=n.m[0][2]; m[0][3]+=n.m[0][3];
            m[1][0]+=n.m[1][0]; m[1][1]+=n.m[1][1]; m[1][2]+=n.m[1][2]; m[1][3]+=n.m[1][3];
            m[2][0]+=n.m[2][0]; m[2][1]+=n.m[2][1]; m[2][2]+=n.m[2][2]; m[2][3]+=n.m[2][3];
            m[3][0]+=n.m[3][0]; m[3][1]+=n.m[3][1]; m[3][2]+=n.m[3][2]; m[3][3]+=n.m[3][3];
            return *this;
        }

        // - operator
        Matrix4 operator-(const Matrix4 &n) const
        {
            return Matrix4(m[0][0]-n.m[0][0], m[0][1]-n.m[0][1], m[0][2]-n.m[0][2], m[0][3]-n.m[0][3],
                           m[1][0]-n.m[1][0], m[1][1]-n.m[1][1], m[1][2]-n.m[1][2], m[1][3]-n.m[1][3],
                           m[2][0]-n.m[2][0], m[2][1]-n.m[2][1], m[2][2]-n.m[2][2], m[2][3]-n.m[2][3],
                           m[3][0]-n.m[3][0], m[3][1]-n.m[3][1], m[3][2]-n.m[3][2], m[3][3]-n.m[3][3]);
        }

        // -= operator
        Matrix4& operator-=(const Matrix4 &n)
        {
            m[0][0]-=n.m[0][0]; m[0][1]-=n.m[0][1]; m[0][2]-=n.m[0][2]; m[0][3]-=n.m[0][3];
            m[1][0]-=n.m[1][0]; m[1][1]-=n.m[1][1]; m[1][2]-=n.m[1][2]; m[1][3]-=n.m[1][3];
            m[2][0]-=n.m[2][0]; m[2][1]-=n.m[2][1]; m[2][2]-=n.m[2][2]; m[2][3]-=n.m[2][3];
            m[3][0]-=n.m[3][0]; m[3][1]-=n.m[3][1]; m[3][2]-=n.m[3][2]; m[3][3]-=n.m[3][3];
            return *this;
        }

        // = operator
        Matrix4& operator=(const Matrix4& matrix)
        {
            if (&matrix != this)
            {
                setAllValues(matrix.m[0][0], matrix.m[0][1], matrix.m[0][2], matrix.m[0][3],
                             matrix.m[1][0], matrix.m[1][1], matrix.m[1][2], matrix.m[1][3],
                             matrix.m[2][0], matrix.m[2][1], matrix.m[2][2], matrix.m[2][3],
                             matrix.m[3][0], matrix.m[3][1], matrix.m[3][2], matrix.m[3][3]);
            }
            return *this;
        }

        // == operator
        bool operator==(const Matrix4 &n) const
        {
            return m[0][0]==n.m[0][0] && m[0][1]==n.m[0][1] && m[0][2]==n.m[0][2] && m[0][3]==n.m[0][3] &&
                    m[1][0]==n.m[1][0] && m[1][1]==n.m[1][1] && m[1][2]==n.m[1][2] && m[1][3]==n.m[1][3] &&
                    m[2][0]==n.m[2][0] && m[2][1]==n.m[2][1] && m[2][2]==n.m[2][2] && m[2][3]==n.m[2][3] &&
                    m[3][0]==n.m[3][0] && m[3][1]==n.m[3][1] && m[3][2]==n.m[3][2] && m[3][3]==n.m[3][3];
        }

        // * operator
        Matrix4 operator*(const Matrix4 &n) const
        {
                    /**
                    Matrix4 o;
                    for(int i = 0; i  < 4; i++)
                    {
                        for(int j = 0; j < 4; j++)
                        {
                            float v = 0;
                            for(int k = 0; k < 4; k++)
                            {
                                v += m[i][k] * n.m[k][j];
                            }
                            o.m[i][j] = v;
                        }
                    }
                   return o;
                   /**/

            return dotMatrix(*this, n);
        }


        // * operator
        Vector3 operator*(const Vector3 &v) const
        {
            Vector3 u =Vector3(m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z + m[0][3],
                               m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z + m[1][3],
                               m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z + m[2][3]);
            float w = m[3][0]*v.x + m[3][1]*v.y + m[3][2]*v.z + m[3][3];
            return u/w;
        }

        // * operator
        Vector4 operator*(const Vector4 &v) const
        {
            Vector4 u = Vector4(m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z + v.w*m[0][3],
                                m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z + v.w*m[1][3],
                                m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z + v.w*m[2][3],
                                m[3][0]*v.x + m[3][1]*v.y + m[3][2]*v.z + v.w*m[3][3]);
            if(u.w != 0)
                return u/u.w;
            else
                return u;
        }

        // * operator
        Matrix4 operator*(float f) const
        {
            return Matrix4(m[0][0]*f, m[0][1]*f, m[0][2]*f,  m[0][3]*f,
                           m[1][0]*f, m[1][1]*f, m[1][2]*f,  m[1][3]*f,
                           m[2][0]*f, m[2][1]*f, m[2][2]*f,  m[2][3]*f,
                           m[3][0]*f, m[3][1]*f, m[3][2]*f,  m[3][3]*f);
        }

        // * operator
        Matrix4 &operator*=(float f)
        {
            m[0][0]*=f; m[0][1]*=f; m[0][2]*=f;  m[0][3]*=f;
            m[1][0]*=f; m[1][1]*=f; m[1][2]*=f;  m[1][3]*=f;
            m[2][0]*=f; m[2][1]*=f; m[2][2]*=f;  m[2][3]*=f;
            m[3][0]*=f; m[3][1]*=f; m[3][2]*=f;  m[3][3]*=f;
            return *this;
        }

        // / operator
        Matrix4 operator/(float f) const
        {
            assert(f!=0);
            return Matrix4(m[0][0]/f, m[0][1]/f, m[0][2]/f,  m[0][3]/f,
                           m[1][0]/f, m[1][1]/f, m[1][2]/f,  m[1][3]/f,
                           m[2][0]/f, m[2][1]/f, m[2][2]/f,  m[2][3]/f,
                           m[3][0]/f, m[3][1]/f, m[3][2]/f,  m[3][3]/f);
        }

        // /= operator
        Matrix4 &operator/=(float f)
        {
            assert(f!=0);
            m[0][0]/=f; m[0][1]/=f; m[0][2]/=f;  m[0][3]/=f;
            m[1][0]/=f; m[1][1]/=f; m[1][2]/=f;  m[1][3]/=f;
            m[2][0]/=f; m[2][1]/=f; m[2][2]/=f;  m[2][3]/=f;
            m[3][0]/=f; m[3][1]/=f; m[3][2]/=f;  m[3][3]/=f;
            return *this;
        }

        // - operator
        Matrix4 operator-() const
        {
            return Matrix4(-m[0][0], -m[0][1], -m[0][2],  -m[0][3],
                           -m[1][0], -m[1][1], -m[1][2],  -m[1][3],
                           -m[2][0], -m[2][1], -m[2][2],  -m[2][3],
                           -m[3][0], -m[3][1], -m[3][2],  -m[3][3]);
        }

        // Return the transpose matrix
        Matrix4 getTranspose() const
        {
            return Matrix4(m[0][0], m[1][0], m[2][0], m[3][0],
                           m[0][1], m[1][1], m[2][1], m[3][1],
                           m[0][2], m[1][2], m[2][2], m[3][2],
                           m[0][3], m[1][3], m[2][3], m[3][3]);
        }

        // Return the inversed matrix
        Matrix4 getInverse() const
        {
            int indxc[4], indxr[4];
            int ipiv[4] = { 0, 0, 0, 0 };
            float minv[4][4];
            float temp;

            for (int s=0; s<4; s++) {
                for (int t=0; t<4; t++) {
                    minv[s][t] = m[s][t];
                }
            }

            for (int i = 0; i < 4; i++) {
                int irow = -1, icol = -1;
                float big = 0.;
                // Choose pivot
                for (int j = 0; j < 4; j++) {
                    if (ipiv[j] != 1) {
                        for (int k = 0; k < 4; k++) {
                            if (ipiv[k] == 0) {
                                if (fabs(minv[j][k]) >= big) {
                                    big = float(fabs(minv[j][k]));
                                    irow = j;
                                    icol = k;
                                }
                            }
                            else if (ipiv[k] > 1) {
                                std::cout << "ERROR: Singular matrix in MatrixInvert\n";
                            }
                        }
                    }
                }
                ++ipiv[icol];
                // Swap rows _irow_ and _icol_ for pivot
                if (irow != icol) {
                    for (int k = 0; k < 4; ++k){
                        temp = minv[irow][k];
                        minv[irow][k] = minv[icol][k];
                        minv[icol][k] = temp;
                    }
                }
                indxr[i] = irow;
                indxc[i] = icol;
                if (minv[icol][icol] == 0.){
                    std::cout << "Singular matrix in MatrixInvert\n";
                }
                // Set $m[icol][icol]$ to one by scaling row _icol_ appropriately
                float pivinv = 1.f / minv[icol][icol];
                minv[icol][icol] = 1.f;
                for (int j = 0; j < 4; j++) {
                    minv[icol][j] *= pivinv;
                }

                // Subtract this row from others to zero out their columns
                for (int j = 0; j < 4; j++) {
                    if (j != icol) {
                        float save = minv[j][icol];
                        minv[j][icol] = 0;
                        for (int k = 0; k < 4; k++) {
                            minv[j][k] -= minv[icol][k]*save;
                        }
                    }
                }
            }
            // Swap columns to reflect permutation
            for (int j = 3; j >= 0; j--)
            {
                if (indxr[j] != indxc[j])
                {
                    for (int k = 0; k < 4; k++)
                    {
                        temp = minv[k][indxr[j]];
                        minv[k][indxr[j]] = minv[k][indxc[j]];
                        minv[k][indxc[j]] = temp;
                    }
                }
            }
            return Matrix4(minv);
        }

        // Method to set all the values in the matrix
        void setAllValues(float a1, float a2, float a3, float a4,
                          float b1, float b2, float b3, float b4,
                          float c1, float c2, float c3, float c4,
                          float d1, float d2, float d3, float d4)
        {
            m[0][0] = a1; m[0][1] = a2; m[0][2] = a3, m[0][3] = a4;
            m[1][0] = b1; m[1][1] = b2; m[1][2] = b3; m[1][3] = b4;
            m[2][0] = c1; m[2][1] = c2; m[2][2] = c3; m[2][3] = c4;
            m[3][0] = d1; m[3][1] = d2; m[3][2] = d3; m[3][3] = d4;
        }

        // Set the matrix to the identity matrix
        Matrix4 setToIdentity()
        {
            m[0][0] = 1.f; m[0][1] = 0.f; m[0][2] = 0.f;  m[0][3] = 0.f;
            m[1][0] = 0.f; m[1][1] = 1.f; m[1][2] = 0.f;  m[1][3] = 0.f;
            m[2][0] = 0.f; m[2][1] = 0.f; m[2][2] = 1.f;  m[2][3] = 0.f;
            m[3][0] = 0.f; m[3][1] = 0.f; m[3][2] = 0.f;  m[3][3] = 1.f;
            return *this;
        }

        Matrix3 getMatrix3x3() const ;

        // Display the matrix
        void print() const
        {
            for (int i=0; i<4; i++)
            {
                for (int j=0; j<4; j++)
                {
                    std::cout << m[i][j];
                }
                std::cout << std::endl;
            }
        }



        float* operator & ()
        {
            return (float*)this;
        }

        // Return the pointer to the data array of the matrix
        float* dataBlock()
        {
            return m[0];
        }

        // Return the constant pointer to the data array of the matrix
        const float* dataBlock() const
        {
            return m[0];
        }

        // Return a given value from the matrix
        float getValue(int i, int j) const
        {
            assert(i >= 0 && i<4 && j >= 0 && j<4);
            return m[i][j];
        }


        void getDataValue( float *M )
        {
             for (int i=0; i<4; i++)
             {
                for (int j=0; j<4; j++)
                {
                    M[i*4 + j] = m[i][j];
                }
             }
        }



        void setDataValue( const float *M )
        {
             for (int i=0; i<4; i++)
             {
                for (int j=0; j<4; j++)
                {
                   m[i][j] =  M[i*4 + j];
                }
             }

        }

        // Return the trace of the matrix
        float getTrace() const
        {
            // Compute and return the trace
            return (m[0][0] + m[1][1] + m[2][2] + m[3][3]);
        }


        // Return a 4x4 translation matrix
        static Matrix4 translationMatrix(const Vector3& v);
        static Matrix4 translationMatrix2(const Vector3& v);

        // Return a 4x4 rotation matrix
        static Matrix4 rotationMatrix(const Vector3& axis, float angle);
        static Matrix4 rotationMatrix2(const Vector3& axis, float angle);


        static Matrix4 scaleMatrix(const Vector3 &scale);



        /// Return a skew-symmetric matrix using a given vector that can be used
        /// to compute cross product with another vector using matrix multiplication
        static Matrix4 computeSkewSymmetricMatrixForCrossProduct(const Vector4& vector)
        {
            return Matrix4( 0        , -vector.z,  vector.y,  vector.w,
                            vector.z , 0        , -vector.x,  vector.y,
                           -vector.y , vector.x , 0        , -vector.x,
                           -vector.w ,-vector.y , vector.x , 0);
        }




        void OrthoNormalize()
        {
           v.right.normalize();
           v.up.normalize();
           v.dir.normalize();
        }


        static void DecomposeMatrixToComponents(const float *matrix, float *translation, float *rotation, float *scale)
        {
            Matrix4 mat = *(Matrix4*)matrix;

            scale[0] = mat.v.right.length();
            scale[1] = mat.v.up.length();
            scale[2] = mat.v.dir.length();

            mat.OrthoNormalize();

            rotation[0] = RAD2DEG * atan2f(mat.m[1][2], mat.m[2][2]);
            rotation[1] = RAD2DEG * atan2f(-mat.m[0][2], sqrtf(mat.m[1][2] * mat.m[1][2] + mat.m[2][2]* mat.m[2][2]));
            rotation[2] = RAD2DEG * atan2f(mat.m[0][1], mat.m[0][0]);

            translation[0] = mat.v.position.x;
            translation[1] = mat.v.position.y;
            translation[2] = mat.v.position.z;
        }

        static void RecomposeMatrixFromComponents(const float *translation, const float *rotation, const float *scale, float *matrix)
        {
            Matrix4 mat = *(Matrix4*)matrix;

            const Vector3 directionUnary[3] = { Vector3::X ,
                                                Vector3::Y ,
                                                Vector3::Z };

            Matrix4 rot[3];
            for (int i = 0; i < 3;i++)
            {
                rot[i] = Matrix4::rotationMatrix( directionUnary[i] , rotation[i] * DEG2RAD);
            }

            mat = rot[0] * rot[1] * rot[2];

            float validScale[3];
            for (int i = 0; i < 3; i++)
            {
                if (fabsf(scale[i]) < FLT_EPSILON)
                {
                    validScale[i] = 0.001f;
                }
                else
                {
                    validScale[i] = scale[i];
                }
            }
            mat.v.right *= validScale[0];
            mat.v.up *= validScale[1];
            mat.v.dir *= validScale[2];
            mat.v.position = Vector4(translation[0], translation[1], translation[2], 1.f);
        }



        void DecomposeMatrixToComponents( Vector3 &translation, Vector3 &rotation, Vector3 &scale )
        {
            DecomposeMatrixToComponents( m16 , translation.data() , rotation.data() , scale.data() );
        }

        void RecomposeMatrixFromComponents(const Vector3& translation, const Vector3& rotation, const Vector3& scale)
        {
            RecomposeMatrixFromComponents( translation.data() , rotation.data() , scale.data() , m16 );
        }

};

// * operator
inline Matrix4 operator*(float f, const Matrix4 & m)
{
    return (m * f);
}

// Return a 4x4 translation matrix
inline Matrix4 Matrix4::translationMatrix(const Vector3& v)
{
    return Matrix4(1, 0, 0, v.x,
                   0, 1, 0, v.y,
                   0, 0, 1, v.z,
                   0, 0, 0, 1);
}






// Return a 4x4 rotation matrix
inline Matrix4 Matrix4::rotationMatrix(const Vector3& axis, float angle)
{

    //angle = angle / 180.0f * (float)M_PI;

    float cosA = cos(angle);
    float sinA = sin(angle);
    Matrix4 rotationMatrix;
    rotationMatrix.setToIdentity();

    rotationMatrix.m[0][0] = cosA + (1-cosA) * axis.x * axis.x;
    rotationMatrix.m[0][1] = (1-cosA) * axis.x * axis.y - axis.z * sinA;
    rotationMatrix.m[0][2] = (1-cosA) * axis.x * axis.z + axis.y * sinA;
    rotationMatrix.m[0][3] = 0.f;

    rotationMatrix.m[1][0] = (1-cosA) * axis.x * axis.y + axis.z * sinA;
    rotationMatrix.m[1][1] = cosA + (1-cosA) * axis.y * axis.y;
    rotationMatrix.m[1][2] = (1-cosA) * axis.y * axis.z - axis.x * sinA;
    rotationMatrix.m[1][3] = 0.f;

    rotationMatrix.m[2][0] = (1-cosA) * axis.x * axis.z - axis.y * sinA;
    rotationMatrix.m[2][1] = (1-cosA) * axis.y * axis.z + axis.x * sinA;
    rotationMatrix.m[2][2] = cosA + (1-cosA) * axis.z * axis.z;
    rotationMatrix.m[2][3] = 0.f;

    rotationMatrix.m[3][0] = 0.f;
    rotationMatrix.m[3][1] = 0.f;
    rotationMatrix.m[3][2] = 0.f;
    rotationMatrix.m[3][3] = 1.f;

    return rotationMatrix;
}


/**/
// Return a 4x4 boost
inline Matrix4 Matrix4::rotationMatrix2(const Vector3& axis, float angle)
{

    //angle = angle / 180.0f * (float)M_PI;

    float cosA = cos(angle);
    float sinA = sin(angle);
    Matrix4 rotationMatrix;
    rotationMatrix.setToIdentity();

    rotationMatrix.m[0][0] = axis.x * axis.x + (1.0 - axis.x * axis.x) * cosA;
    //rotationMatrix.m[0][0] = cosA + (1-cosA) * axis.x * axis.x;
    rotationMatrix.m[0][1] = (1-cosA) * axis.x * axis.y - axis.z * sinA;
    rotationMatrix.m[0][2] = (1-cosA) * axis.x * axis.z + axis.y * sinA;
    rotationMatrix.m[0][3] = 0.f;

    rotationMatrix.m[1][0] = (1-cosA) * axis.x * axis.y + axis.z * sinA;
    rotationMatrix.m[1][1] = axis.y * axis.y + (1.0 - axis.y * axis.y)* cosA;
    //rotationMatrix.m[1][1] = cosA + (1-cosA) * axis.y * axis.y;
    rotationMatrix.m[1][2] = (1-cosA) * axis.y * axis.z - axis.x * sinA;
    rotationMatrix.m[1][3] = 0.f;

    rotationMatrix.m[2][0] = (1-cosA) * axis.x * axis.z - axis.y * sinA;
    rotationMatrix.m[2][1] = (1-cosA) * axis.y * axis.z + axis.x * sinA;
    rotationMatrix.m[2][2] = axis.z * axis.z + (1.0 - axis.z * axis.z) * cosA;
    //rotationMatrix.m[2][2] = cosA + (1-cosA) * axis.z * axis.z;
    rotationMatrix.m[2][3] = 0.f;

    rotationMatrix.m[3][0] = 0.f;
    rotationMatrix.m[3][1] = 0.f;
    rotationMatrix.m[3][2] = 0.f;
    rotationMatrix.m[3][3] = 1.f;

    return rotationMatrix;
}

/**/

inline Matrix4 Matrix4::scaleMatrix(const Vector3 &scale)
{
      /**
      return Matrix4(1.f + scale.x, 0, 0, 0,
                     0, 1.f + scale.y, 0, 0,
                     0, 0, 1.f + scale.z, 0,
                     0, 0, 0, 1);
      /**/

      float _x = 1.f + scale.x;
      float _y = 1.f + scale.y;
      float _z = 1.f + scale.z;

      Matrix4 M;
      M.v.right    = Vector4(_x , 0.f, 0.f, 0.f);
      M.v.up       = Vector4(0.f, _y , 0.f, 0.f);
      M.v.dir      = Vector4(0.f, 0.f, _z , 0.f);
      M.v.position = Vector4(0.f, 0.f, 0.f, 1.f);

      return M;
}


static  Matrix4 biasMatrix = Matrix4(0.5f, 0.0f, 0.0f, 0.0f,
                                     0.0f, 0.5f, 0.0f, 0.0f,
                                     0.0f, 0.0f, 0.5f, 0.0f,
                                     0.5f, 0.5f, 0.5f, 1.0f);

static  Matrix4 biasMatrixInverse = Matrix4(2.0f, 0.0f, 0.0f, 0.0f,
                                            0.0f, 2.0f, 0.0f, 0.0f,
                                            0.0f, 0.0f, 2.0f, 0.0f,
                                           -1.0f,-1.0f,-1.0f, 1.0f);


//	Matrix4        Look(const Vector3 &eye, const Vector3 &center, const Vector3 &up);
//	Matrix4        Ortho(float left, float right, float bottom, float top, float n, float f);
//	Matrix4        Perspective(float fovy, float aspect, float n, float f);
//	Matrix4        rotate(float angle, const Vector3& u);


static Matrix4 dotMatrix( Matrix4 Matrix1 , Matrix4 Matrix2)
{
    Matrix4 Matrix3;

    float  M[16];
    float M1[16];
    float M2[16];
    Matrix1.getDataValue(M1);
    Matrix2.getDataValue(M2);

    M[0]  = M1[0] * M2[0] + M1[4] * M2[1] + M1[8] * M2[2] + M1[12] * M2[3];
    M[1]  = M1[1] * M2[0] + M1[5] * M2[1] + M1[9] * M2[2] + M1[13] * M2[3];
    M[2]  = M1[2] * M2[0] + M1[6] * M2[1] + M1[10] * M2[2] + M1[14] * M2[3];
    M[3]  = M1[3] * M2[0] + M1[7] * M2[1] + M1[11] * M2[2] + M1[15] * M2[3];
    M[4]  = M1[0] * M2[4] + M1[4] * M2[5] + M1[8] * M2[6] + M1[12] * M2[7];
    M[5]  = M1[1] * M2[4] + M1[5] * M2[5] + M1[9] * M2[6] + M1[13] * M2[7];
    M[6]  = M1[2] * M2[4] + M1[6] * M2[5] + M1[10] * M2[6] + M1[14] * M2[7];
    M[7]  = M1[3] * M2[4] + M1[7] * M2[5] + M1[11] * M2[6] + M1[15] * M2[7];
    M[8]  = M1[0] * M2[8] + M1[4] * M2[9] + M1[8] * M2[10] + M1[12] * M2[11];
    M[9]  = M1[1] * M2[8] + M1[5] * M2[9] + M1[9] * M2[10] + M1[13] * M2[11];
    M[10] = M1[2] * M2[8] + M1[6] * M2[9] + M1[10] * M2[10] + M1[14] * M2[11];
    M[11] = M1[3] * M2[8] + M1[7] * M2[9] + M1[11] * M2[10] + M1[15] * M2[11];
    M[12] = M1[0] * M2[12] + M1[4] * M2[13] + M1[8] * M2[14] + M1[12] * M2[15];
    M[13] = M1[1] * M2[12] + M1[5] * M2[13] + M1[9] * M2[14] + M1[13] * M2[15];
    M[14] = M1[2] * M2[12] + M1[6] * M2[13] + M1[10] * M2[14] + M1[14] * M2[15];
    M[15] = M1[3] * M2[12] + M1[7] * M2[13] + M1[11] * M2[14] + M1[15] * M2[15];

    Matrix3.setDataValue(M);
    return Matrix3;
}


}




#endif //_MATRIX4_H
