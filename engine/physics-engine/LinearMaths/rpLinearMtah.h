/*
 * LinearMtah.h
 *
 *  Created on: 16 нояб. 2016 г.
 *      Author: wera
 */

#ifndef LINEARMATHS_RPLINEARMTAH_H_
#define LINEARMATHS_RPLINEARMTAH_H_

#include <assert.h>

#include <algorithm>
#include <stdlib.h>

#include <math.h>
#include <cmath>

#include <limits>

#include "../config.h"

namespace real_physics
{

    #define SIMD_INLINE  inline

	#define ATTRIBUTE_ALIGNED16(a)  __declspec(align(16)) a
	#define ATTRIBUTE_ALIGNED64(a)  __declspec(align(64)) a
	#define ATTRIBUTE_ALIGNED128(a) __declspec(align(128)) a

	//# define CCD_FABS(x) (fabsf(x))
	#define CCD_EPS 1E-6


	#define FLT_MIN 1.175494351e-38F /* min positive value */
	#define FLT_MAX 3.402823466e+38F /* max value */




	#define FLT_EPSILON 1.1920928955078125E-7f
	#define MACHINE_EPSILON 10E-7f



 	// ---------- Mathematics functions ---------- //

 	template<typename T> SIMD_INLINE T Min(T a, T b)
 	{
 		return (a > b) ? b : a;
 	}

 	template<typename T> SIMD_INLINE T Max(T a, T b)
 	{
 		return (a < b) ? b : a;
 	}

 	template<typename T> SIMD_INLINE T Min(T a, T b, T c)
 	{
 		return Min<T>(Min<T>(a, b), c);
 	}

 	template<typename T> SIMD_INLINE T Max(T a, T b, T c)
 	{
 		return Max<T>(Max<T>(a, b), c);
 	}

 	template<typename T> SIMD_INLINE T Clamp(T a, T min, T max)
 	{
 		return Max<T>(Min<T>(a, max), min);
 	}

 	template<typename T> SIMD_INLINE T Wrap(T a, T min, T max)
 	{
 		return (a < min) ? max - (min - a) : (a > max) ? min - (max - a) : a;
 	}

 	template<typename T> SIMD_INLINE void Swap(T& a, T& b)
 	{
 		T c = a;
 		a = b;
 		b = c;
 	}

 	//-----------------------------------------------------//



#define btRecipSqrt(x) ((scalar)(scalar(1.0)/sqrt(scalar(x))))
#define SIMDSQRT12       scalar(0.7071067811865475244008443621048490)




 	SIMD_INLINE scalar btSqrt( scalar y )
 	{

			#ifdef USE_APPROXIMATION
			#ifdef __LP64__
					float xhalf = 0.5f*y;
					int i = *(int*)&y;
					i = 0x5f375a86 - (i>>1);
					y = *(float*)&i;
					y = y*(1.5f - xhalf*y*y);
					y = y*(1.5f - xhalf*y*y);
					y = y*(1.5f - xhalf*y*y);
					y=1/y;
					return y;
			#else
				scalar x, z, tempf;
				unsigned long *tfptr = ((unsigned long *)&tempf) + 1;
				tempf = y;
				*tfptr = (0xbfcdd90a - *tfptr)>>1; /* estimate of 1/sqrt(y) */
				x =  tempf;
				z =  y*scalar(0.5);
				x = (scalar(1.5)*x)-(x*x)*(x*z);         /* iteration formula     */
				x = (scalar(1.5)*x)-(x*x)*(x*z);
				x = (scalar(1.5)*x)-(x*x)*(x*z);
				x = (scalar(1.5)*x)-(x*x)*(x*z);
				x = (scalar(1.5)*x)-(x*x)*(x*z);
				return x*y;
			#endif
			#else
                return sqrtf(y);
			#endif

 	}


 	SIMD_INLINE scalar Sign(scalar x)
 	{
 		return (x < 0.0f) ? -1.0f : 1.0f;
 	}
 	SIMD_INLINE iint Sign(iint x)
 	{
 		return (x < 0) ? -1 : 1;
 	}

 	SIMD_INLINE scalar Pi(void)
 	{
 		static const scalar gPi = (scalar) atan(1.0f) * 4.0f;
 		return gPi;
 	}
 	SIMD_INLINE scalar TwoPi(void)
 	{
 		static const scalar gTwoPi = (scalar) atan(1.0f) * 8.0f;
 		return gTwoPi;
 	}
 	SIMD_INLINE scalar Modulo(scalar x, scalar div)
 	{
 		return (scalar) fmod((double) x, (double) div);
 	}
 	SIMD_INLINE scalar Abs(scalar x)
 	{
 		return (scalar) fabs(x);
 	}
 	SIMD_INLINE iint Abs( iint x )
 	{
 		return (x) & 0x70000000;
 	}
 	SIMD_INLINE scalar DegreesToRadians(scalar Degrees)
 	{
 		return Degrees * (Pi() / 180.0f);
 	}
 	SIMD_INLINE scalar RadiansToDegrees(scalar Radians)
 	{
 		return Radians * (180.0f / Pi());
 	}
 	SIMD_INLINE scalar Sin(scalar Radians)
 	{
 		return (scalar) sin(Radians);
 	}
 	SIMD_INLINE scalar Cos(scalar Radians)
 	{
 		return (scalar) cos(Radians);
 	}
 	SIMD_INLINE scalar SquareRoot(scalar In)
 	{
 		return (scalar) btSqrt(In);
 	}
 	SIMD_INLINE scalar Tangent(scalar Radians)
 	{
 		return (scalar) tan(Radians);
 	}
 	SIMD_INLINE scalar ArcTangent(scalar X)
 	{
 		return (scalar) atan(X);
 	}
 	SIMD_INLINE scalar ArcTang(scalar X, scalar Y)
 	{
 		return (scalar) atan(X);
 	}
 	SIMD_INLINE scalar Rand(scalar r = 1.0f)
 	{
 		return rand() / ((scalar) RAND_MAX) * r;
 	}
 	SIMD_INLINE scalar Rand(scalar min, scalar max)
 	{
 		return min + Rand(max - min);
 	}
 	SIMD_INLINE scalar ArcSin(scalar X)
 	{
 		return (scalar) asin(X);
 	}
 	SIMD_INLINE scalar ArcCos(scalar X)
 	{
 		return (scalar) acos(X);
 	}

 	//-----------------------------------------------------//


 	/// Function to test if two real numbers are (almost) equal
 	/// We test if two numbers a and b are such that (a-b) are in [-EPSILON; EPSILON]
 	SIMD_INLINE bool approxEqual(scalar a, scalar b, scalar epsilon = MACHINE_EPSILON)
 	{
 		return (Abs(a - b) < scalar(epsilon) );
 	}


 	SIMD_INLINE int clamp(int value, int lowerLimit, int upperLimit)
 	{
 		assert(lowerLimit <= upperLimit);
 		return std::min(std::max(value, lowerLimit), upperLimit);
 	}



 	SIMD_INLINE scalar clamp(scalar value, scalar lowerLimit, scalar upperLimit)
 	{
 		assert(lowerLimit <= upperLimit);
 		return std::min(std::max(value, lowerLimit), upperLimit);
 	}


 	SIMD_INLINE scalar min3(scalar a, scalar b, scalar c)
 	{
 		return std::min(std::min(a, b), c);
 	}


 	SIMD_INLINE scalar max3(scalar a, scalar b, scalar c)
 	{
 		return std::max(std::max(a, b), c);
 	}


 	SIMD_INLINE bool sameSign(scalar a, scalar b)
 	{
 		return a * b >= scalar(0.0);
 	}

}



#endif /* LINEARMATHS_RPLINEARMTAH_H_ */
