/*
 * rpTransformUtil.h
 *
 *  Created on: 19 дек. 2016 г.
 *      Author: wera
 */

#ifndef SOURCE_ENGIE_LINEARMATHS_RPTRANSFORMUTIL_H_
#define SOURCE_ENGIE_LINEARMATHS_RPTRANSFORMUTIL_H_

#include "rpTransform.h"
#include <math.h>
#include <stdlib.h>//size_t for MSVC 6.0
#include <float.h>
#include "rpMinkowskiVector4.h"

namespace real_physics
{






#define SIMD_2_PI scalar(6.283185307179586232)
#define SIMD_PI (SIMD_2_PI * scalar(0.5))
#define SIMD_HALF_PI (SIMD_2_PI * scalar(0.25))
#define ANGULAR_MOTION_THRESHOLD scalar(0.5)*SIMD_HALF_PI

#define SIMD_EPSILON      FLT_EPSILON


template<class T> class rpTransformUtil
{
   public:

	static rpTransform<T> integrateTransform( const rpTransform<T>& curTrans , const rpVector3D<T>& linvel, const rpVector3D<T>& angvel, T timeStep)
	{

		//Exponential map
		//google for "Practical Parameterization of Rotations Using the Exponential Map", F. Sebastian Grassia
		rpVector3D<T> axis;
        T fAngle = angvel.length();

        //limit the angular motion
        if (fAngle*timeStep > ANGULAR_MOTION_THRESHOLD)
        {
        	fAngle = ANGULAR_MOTION_THRESHOLD / timeStep;
        }

        if (fAngle < T(0.001))
		{
			// use Taylor's expansions of sync function
			axis = angvel * (T(0.5) * timeStep -
					        (timeStep * timeStep * timeStep) *
                            (T(0.020833333333)) * fAngle * fAngle);
		}
		else
		{
			// sync(fAngle) = sin(c*fAngle)/t
            axis = angvel * (Sin(T(0.5) * fAngle * timeStep) / fAngle);
		}


        rpQuaternion<T> dorn(axis, Cos(fAngle * timeStep * T(0.5)));
		rpQuaternion<T> predictedOrn = dorn * curTrans.getOrientation();
		predictedOrn.normalize();


        return rpTransform<T>(curTrans.getPosition() + linvel * timeStep , predictedOrn );

	}







	static void integrateReletiviteLinear( rpMinkowskiVector4<T>& curPos , const rpVector3D<T>& linvel ,const T timeStep ,const T gammaInv )
	{
		//T gammaSet = (gammaInv - 1.0);

		T  l = (0.0001 + linvel.length2());
		rpVector3D<T> r = curPos.getVector3();

		curPos.setVector3( r + linvel * timeStep - ((gammaInv - 1.0) * ((r.dot(linvel)) * linvel) / l) * timeStep);
		curPos.t = gammaInv * (curPos.t + r.dot(linvel) / (LIGHT_MAX_VELOCITY_C * LIGHT_MAX_VELOCITY_C));

	}




	//-------------------------  differential method ------------------------------------//


	static void	calculateVelocityQuaternion(const rpVector3D<T> & pos0,const rpVector3D<T> & pos1,const rpQuaternion<T>& orn0,const rpQuaternion<T>& orn1,scalar timeStep,rpVector3D<T> & linVel,rpVector3D<T> & angVel)
	{
		linVel = (pos1 - pos0) / timeStep;
		rpVector3D<T>  axis;
		scalar  angle;
		if (orn0 != orn1)
		{
			calculateDiffAxisAngleQuaternion(orn0,orn1,axis,angle);
			angVel = axis * angle / timeStep;
		} else
		{
			angVel.setValue(0,0,0);
		}
	}

	//-------------------------  differential method ------------------------------------//

	static void calculateDiffAxisAngleQuaternion(const rpQuaternion<T>& orn0,const rpQuaternion<T>& orn1a,rpVector3D<T> & axis,T& angle)
	{
		rpQuaternion<T> orn1 = orn0.nearest(orn1a);
		rpQuaternion<T> dorn = orn1 * orn0.inverse();
		angle = dorn.getAngle();
		axis = rpVector3D<T> (dorn.x(),dorn.y(),dorn.z());
		axis[3] = T(0.);
		//check for axis length
		T len = axis.length2();
		if (len < SIMD_EPSILON*SIMD_EPSILON)
			axis = rpVector3D<T> (T(1.),T(0.),T(0.));
		else
			axis /= btSqrt(len);
	}


	//-------------------------  differential method ------------------------------------//


	static void	calculateVelocity(const rpTransform<T>& transform0 , const rpTransform<T>& transform1 ,T timeStep , rpVector3D<T>& linVel, rpVector3D<T>& angVel)
	{
		linVel = (transform1.getPosition() - transform0.getPosition()) / timeStep;
		rpVector3D<T>  axis;
		T  angle;
		calculateDiffAxisAngle(transform0,transform1,axis,angle);
		angVel = axis * angle / timeStep;
	}

	static void calculateDiffAxisAngle(const rpTransform<T>& transform0,const rpTransform<T>& transform1,rpVector3D<T> & axis,T& angle)
	{
		rpMatrix3x3<T> dmat = transform1.getBasis() * transform0.getBasis().getInverse();
		rpQuaternion<T> dorn = rpQuaternion<T>(dmat);

		///floating point inaccuracy can lead to w component > 1..., which breaks
		dorn.normalize();

		angle = dorn.getAngle();
		axis = rpVector3D<T> (dorn.x,dorn.y,dorn.z);
		//axis[3] = T(0.);
		//check for axis length
		T len = axis.length2();
		if (len < SIMD_EPSILON*SIMD_EPSILON)
			axis = rpVector3D<T> (T(1.),T(0.),T(0.));
		else
			axis /= btSqrt(len);
	}
};





///The btConvexSeparatingDistanceUtil can help speed up convex collision detection
///by conservatively updating a cached separating distance/vector instead of re-calculating the closest distance
template<class T> class btConvexSeparatingDistanceUtil
{
	rpQuaternion<T>	m_ornA;
	rpQuaternion<T>	m_ornB;
	rpVector3D<T>	m_posA;
	rpVector3D<T>	m_posB;

	rpVector3D<T>	m_separatingNormal;

	T	m_boundingRadiusA;
	T	m_boundingRadiusB;
	T	m_separatingDistance;

public:

	btConvexSeparatingDistanceUtil(T	boundingRadiusA,T	boundingRadiusB)
		:m_boundingRadiusA(boundingRadiusA),
		m_boundingRadiusB(boundingRadiusB),
		m_separatingDistance(0.f)
	{
	}

	T	getConservativeSeparatingDistance()
	{
		return m_separatingDistance;
	}

	void	updateSeparatingDistance(const rpTransform<T>& transA,const rpTransform<T>& transB)
	{
		const rpVector3D<T>& toPosA = transA.getOrigin();
		const rpVector3D<T>& toPosB = transB.getOrigin();
		rpQuaternion<T> toOrnA = transA.getRotation();
		rpQuaternion<T> toOrnB = transB.getRotation();

		if (m_separatingDistance>0.f)
		{


			rpVector3D<T> linVelA,angVelA,linVelB,angVelB;
			rpTransformUtil<T>::calculateVelocityQuaternion(m_posA,toPosA,m_ornA,toOrnA,T(1.),linVelA,angVelA);
			rpTransformUtil<T>::calculateVelocityQuaternion(m_posB,toPosB,m_ornB,toOrnB,T(1.),linVelB,angVelB);
			T maxAngularProjectedVelocity = angVelA.length() * m_boundingRadiusA + angVelB.length() * m_boundingRadiusB;
			rpVector3D<T> relLinVel = (linVelB-linVelA);
			T relLinVelocLength = relLinVel.dot(m_separatingNormal);
			if (relLinVelocLength<0.f)
			{
				relLinVelocLength = 0.f;
			}

			T	projectedMotion = maxAngularProjectedVelocity +relLinVelocLength;
			m_separatingDistance -= projectedMotion;
		}

		m_posA = toPosA;
		m_posB = toPosB;
		m_ornA = toOrnA;
		m_ornB = toOrnB;
	}

	void	initSeparatingDistance(const rpVector3D<T>& separatingVector,T separatingDistance,const rpTransform<T>& transA,const rpTransform<T>& transB)
	{
		m_separatingDistance = separatingDistance;

		if (m_separatingDistance>0.f)
		{
			m_separatingNormal = separatingVector;

            const rpVector3D<T>& toPosA = transA.getPosition();
			const rpVector3D<T>& toPosB = transB.getPosition();
			rpQuaternion<T> toOrnA = transA.getOrientation();
			rpQuaternion<T> toOrnB = transB.getOrientation();
			m_posA = toPosA;
			m_posB = toPosB;
			m_ornA = toOrnA;
			m_ornB = toOrnB;
		}
	}

};



}


#endif /* SOURCE_ENGIE_LINEARMATHS_RPTRANSFORMUTIL_H_ */
