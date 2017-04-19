/*
 * rpMatrix4x4.h
 *
 *  Created on: 25 февр. 2017 г.
 *      Author: wera
 */

#ifndef SRC_PHYSICS_ENGINE_LINEARMATHS_RPMATRIX4X4_H_
#define SRC_PHYSICS_ENGINE_LINEARMATHS_RPMATRIX4X4_H_


#include "rpVector2D.h"
#include "rpVector3D.h"
#include "rpMinkowskiVector4.h"
#include "rpRelativityFunction.h"

namespace real_physics
{

// Matrix Riemann curvature tensor
template<class T> class  rpMatrix4x4
{
    private :

	// -------------------- Attributes -------------------- //

	// Elements of the matrix
	T m[4][4];




    public :

	 // -------------------- Methods -------------------- //

	// Constructor
	rpMatrix4x4<T>(T m_00=0, T m_01=0, T m_02=0, T m_03=0,
			       T m_10=0, T m_11=0, T m_12=0, T m_13=0,
			       T m_20=0, T m_21=0, T m_22=0, T m_23=0,
			       T m_30=0, T m_31=0, T m_32=0, T m_33=0)
	{
		m[0][0] = m_00; m[0][1] = m_01; m[0][2] = m_02;  m[0][3] = m_03;
		m[1][0] = m_10; m[1][1] = m_11; m[1][2] = m_12;  m[1][3] = m_13;
		m[2][0] = m_20; m[2][1] = m_21; m[2][2] = m_22;  m[2][3] = m_23;
		m[3][0] = m_30; m[3][1] = m_31; m[3][2] = m_32;  m[3][3] = m_33;
	}

	// Constructor
	rpMatrix4x4<T>(T n[4][4])
    {
		m[0][0]=n[0][0]; m[0][1]=n[0][1]; m[0][2]=n[0][2]; m[0][3]=n[0][3];
		m[1][0]=n[1][0]; m[1][1]=n[1][1]; m[1][2]=n[1][2]; m[1][3]=n[1][3];
		m[2][0]=n[2][0]; m[2][1]=n[2][1]; m[2][2]=n[2][2]; m[2][3]=n[2][3];
		m[3][0]=n[3][0]; m[3][1]=n[3][1]; m[3][2]=n[3][2]; m[3][3]=n[3][3];
	}


	// Set the matrix to the identity matrix
	rpMatrix4x4<T> setToIdentity()
	{
		m[0][0] = 1.f; m[0][1] = 0.f; m[0][2] = 0.f;  m[0][3] = 0.f;
		m[1][0] = 0.f; m[1][1] = 1.f; m[1][2] = 0.f;  m[1][3] = 0.f;
		m[2][0] = 0.f; m[2][1] = 0.f; m[2][2] = 1.f;  m[2][3] = 0.f;
		m[3][0] = 0.f; m[3][1] = 0.f; m[3][2] = 0.f;  m[3][3] = 1.f;
		return *this;
    }



	// Return the transpose matrix
	rpMatrix4x4<T> getTranspose() const
	{
		return rpMatrix4x4<T>(m[0][0], m[1][0], m[2][0], m[3][0],
				              m[0][1], m[1][1], m[2][1], m[3][1],
				              m[0][2], m[1][2], m[2][2], m[3][2],
				              m[0][3], m[1][3], m[2][3], m[3][3]);
	}






	// Return the inversed matrix
	rpMatrix4x4<T> getInverse() const
	{
		int indxc[4], indxr[4];
		int ipiv[4] = { 0, 0, 0, 0 };
		float minv[4][4];
		float temp;

		for (int s=0; s<4; s++)
		{
			for (int t=0; t<4; t++)
			{
				minv[s][t] = m[s][t];
			}
		}

		for (int i = 0; i < 4; i++)
		{
			int irow = -1, icol = -1;
			float big = 0.;
			// Choose pivot
			for (int j = 0; j < 4; j++)
			{
				if (ipiv[j] != 1) {
					for (int k = 0; k < 4; k++)
					{
						if (ipiv[k] == 0)
						{
							if (fabs(minv[j][k]) >= big)
							{
								big = float(fabs(minv[j][k]));
								irow = j;
								icol = k;
							}
						}
						else if (ipiv[k] > 1)
						{
							std::cout << "ERROR: Singular matrix in MatrixInvert\n";
						}
					}
				}
			}
			++ipiv[icol];
			// Swap rows _irow_ and _icol_ for pivot
			if (irow != icol)
			{
				for (int k = 0; k < 4; ++k)
				{
					temp = minv[irow][k];
					minv[irow][k] = minv[icol][k];
					minv[icol][k] = temp;
				}
			}
			indxr[i] = irow;
			indxc[i] = icol;
			if (minv[icol][icol] == 0.)
			{
				std::cout << "Singular matrix in MatrixInvert\n";
			}
			// Set $m[icol][icol]$ to one by scaling row _icol_ appropriately
			float pivinv = 1.f / minv[icol][icol];
			minv[icol][icol] = 1.f;
			for (int j = 0; j < 4; j++)
			{
				minv[icol][j] *= pivinv;
			}

			// Subtract this row from others to zero out their columns
			for (int j = 0; j < 4; j++)
			{
				if (j != icol)
				{
					float save = minv[j][icol];
					minv[j][icol] = 0;
					for (int k = 0; k < 4; k++)
					{
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
		return rpMatrix4x4<T>(minv);
	}







	rpMinkowskiVector4<T> operator*( const rpMinkowskiVector4<T> &v ) const
	{
		rpMinkowskiVector4<T> u = rpMinkowskiVector4<T>(m[0][0]*v.t + m[0][1]*v.x + m[0][2]*v.y + v.z*m[0][3],
				                                        m[1][0]*v.t + m[1][1]*v.x + m[1][2]*v.y + v.z*m[1][3],
				                                        m[2][0]*v.t + m[2][1]*v.x + m[2][2]*v.y + v.z*m[2][3],
				                                        m[3][0]*v.t + m[3][1]*v.x + m[3][2]*v.y + v.z*m[3][3]);
		if(u.t > MACHINE_EPSILON)
			return u/u.t;
		else
			return u;
	}





	/*****************************************************
	 *  Help info to web site:  https://arxiv.org/pdf/1103.0156.pdf
	 *****************************************************/
	rpMatrix4x4<T> getLorentzBoost( const rpVector3D<T> &v ) const
	{
		/// lorentz factor
		T y = gammaInvertFunction(v);

		T l = v.length();
		T gamma = (y  - 1.0);

		l = (l > MACHINE_EPSILON)? l : MACHINE_EPSILON;
		rpVector3D<T> n = v / l;

		m[0][0]= y;     m[0][1]= -y * v.x;                  m[0][2]= -y * v.y;                  m[0][3]= -y * v.z;
		m[1][0]=-y*v.x; m[1][1]= 1.0+(gamma*((n.x * n.x))); m[1][2]=     (gamma*((n.x * n.y))); m[1][3]=     (gamma*((n.x * n.z)));
		m[2][0]=-y*v.y; m[2][1]=     (gamma*((n.y * n.x))); m[2][2]= 1.0+(gamma*((n.y * n.y))); m[2][3]=     (gamma*((n.y * n.z)));
		m[3][0]=-y*v.z; m[3][1]=     (gamma*((n.z * n.x))); m[3][2]=     (gamma*((n.z * n.y))); m[3][3]= 1.0+(gamma*((n.z * n.z)));

        return *this;
	}


    rpMatrix4x4<T> getMetricesTensor4DSpaceMinkowski( bool contject = true ) const
	{
		T n = (contject)? 1.0 : -1.0;

		m[0][0]= -n; m[0][1]= 0; m[0][2]= 0; m[0][3]= 0;
		m[1][0]=0;   m[1][1]= n; m[1][2]= 0; m[1][3]= 0;
		m[2][0]=0;   m[2][1]= 0; m[2][2]= n; m[2][3]= 0;
		m[3][0]=0;   m[3][1]= 0; m[3][2]= 0; m[3][3]= n;

	}


   /// Return a skew-symmetric matrix using a given vector that can be used
   /// to compute cross product with another vector using matrix multiplication
   static rpMatrix3x3<T> computeSkewSymmetricMatrixForCrossProduct(const rpMinkowskiVector4<T>& vector)
   {
	   return rpMatrix4x4<T>(0       , -vector.z,  vector.y,  vector.t,
	  			            vector.z , 0        , -vector.x,  vector.y,
	  			           -vector.y , vector.x , 0        , -vector.x,
						   -vector.t ,-vector.y , vector.x , 0);
   }


   /// Return a  matrix using a given vector that can be used
   /// to compute dot product with another vector using matrix multiplication
   static rpMatrix4x4<T> MatrixTensorProduct( const  rpMinkowskiVector4<T> v1 , const  rpMinkowskiVector4<T> v2 )
   {
    	return rpMatrix4x4<T>(v1.t * v2.t , v1.t * v2.x , v1.t * v2.y , v1.t * v2.z,
    			              v1.x * v2.t , v1.x * v2.x , v1.x * v2.y , v1.x * v2.z,
    			              v1.y * v2.t , v1.y * v2.x , v1.y * v2.y , v1.y * v2.z,
    			              v1.z * v2.t , v1.z * v2.x , v1.z * v2.y , v1.z * v2.z);
   }

};

} /* namespace real_physics */

#endif /* SRC_PHYSICS_ENGINE_LINEARMATHS_RPMATRIX4X4_H_ */
