/*
 * rpDistanceJoint.cpp
 *
 *  Created on: 8 янв. 2017 г.
 *      Author: wera
 */

#include "rpDistanceJoint.h"

namespace real_physics
{



void rpDistanceJoint::initBeforeSolve( scalar timeStep )
{


	mR1World = Vector3::ZERO;
	mR2World = Vector3::ZERO;

	Vector3 dp = Body2->getTransform().getPosition() -
			     Body1->getTransform().getPosition();


	scalar deltaLength = dp.length() - mDistance;
	Vector3 n = dp.getUnit();

	Vector3 helpR0 = Vector3::ZERO;
	Vector3 helpR1 = Vector3::ZERO;

	jacobian[0] = -n;
	jacobian[1] = -helpR0.cross(n);
	jacobian[2] =  n;
	jacobian[3] =  helpR1.cross(n);


	mEffectiveMass =  Body1->mMassInverse + Body2->mMassInverse;
	mEffectiveMass = 1.0f / mEffectiveMass;



	softnessOverDt = softness;
	mEffectiveMass += softnessOverDt;


     mBias = 0;
    if (mPositionCorrectionTechnique == BAUMGARTE_JOINTS )
    {
         mBias   = (deltaLength * biasFactor);
    }


	mLength =  deltaLength;

}



void rpDistanceJoint::warmstart()
{

	Body1->applyImpulseLinear(  jacobian[0] * mAccumulatedImpulse);
	Body2->applyImpulseLinear(  jacobian[2] * mAccumulatedImpulse);

}



void rpDistanceJoint::solveVelocityConstraint()
{



	// Get the velocities
	const Vector3& v1 = Body1->mLinearVelocity;// constraintSolverData.linearVelocities[mIndexBody1];
	const Vector3& v2 = Body2->mLinearVelocity;// constraintSolverData.linearVelocities[mIndexBody2];
	const Vector3& w1 = Body1->mAngularVelocity;// constraintSolverData.angularVelocities[mIndexBody1];
	const Vector3& w2 = Body2->mAngularVelocity;// constraintSolverData.angularVelocities[mIndexBody2];


	scalar jv =       v1.dot(jacobian[0])
                    + w1.dot(jacobian[1])
					+ v2.dot(jacobian[2])
					+ w2.dot(jacobian[3]);


	scalar softnessScalar = mAccumulatedImpulse * softnessOverDt;

	scalar lambda = -mEffectiveMass * (jv + mBias + softnessScalar);
	mAccumulatedImpulse += lambda;


	Body1->applyImpulseLinear( jacobian[0] * lambda);
	Body2->applyImpulseLinear( jacobian[2] * lambda);

}



void rpDistanceJoint::solvePositionConstraint()
{



	Vector3 v1p = Body1->mSplitLinearVelocity;
	Vector3 w1p = Body1->mSplitAngularVelocity;

	Vector3 v2p = Body2->mSplitLinearVelocity;
	Vector3 w2p = Body2->mSplitAngularVelocity;

	scalar jv =         v1p.dot(jacobian[0])
                      + w1p.dot(jacobian[1])
					  + v2p.dot(jacobian[2])
					  + w2p.dot(jacobian[3]);

	scalar lambda = -mEffectiveMass * (mLength);


	Body1->applySplitImpulseLinear(  jacobian[0] * lambda );
	Body2->applySplitImpulseLinear(  jacobian[2] * lambda );

}

} /* namespace real_physics */
