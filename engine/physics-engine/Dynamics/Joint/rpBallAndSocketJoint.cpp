/*
 * rpBallAndSocketJoint.cpp
 *
 *  Created on: 9 янв. 2017 г.
 *      Author: wera
 */

// Libraries
#include "rpBallAndSocketJoint.h"

namespace real_physics
{




// Static variables definition
const scalar rpBallAndSocketJoint::BETA = scalar(0.2);

// Constructor
rpBallAndSocketJoint::rpBallAndSocketJoint(const rpBallAndSocketJointInfo& jointInfo)
:rpJoint(jointInfo), mImpulse(Vector3(0, 0, 0))
{

	isWarmStartingActive = true;
	isSplitActive = true;


	/// Pair Body to collision solver
    Body1 = static_cast<rpPhysicsRigidBody*>(mBody1);
    Body2 = static_cast<rpPhysicsRigidBody*>(mBody2);


    // Compute the local-space anchor point the constraint error
    mLocalAnchorPointBody1 = Body1->mTransform.getInverse() * jointInfo.anchorPointWorldSpace;
    mLocalAnchorPointBody2 = Body2->mTransform.getInverse() * jointInfo.anchorPointWorldSpace;


    softness = 0.0001f;

}


// Destructor
rpBallAndSocketJoint::~rpBallAndSocketJoint()
{

}



// Initialize before solving the constraint
void rpBallAndSocketJoint::initBeforeSolve( scalar timeStep  )
{

	// Get the bodies center of mass and orientations
	//const Vector3& x1 = Body1->mCenterOfMassWorld;
	//const Vector3& x2 = Body2->mCenterOfMassWorld;
    const Vector3& x1 = Body1->mTransform.getPosition4().getPos();
    const Vector3& x2 = Body2->mTransform.getPosition4().getPos();
	const Quaternion& orientationBody1 = Body1->mTransform.getOrientation();
	const Quaternion& orientationBody2 = Body2->mTransform.getOrientation();

	// Get the inertia tensor of bodies
	mI1 = Body1->getInertiaTensorInverseWorld();
	mI2 = Body2->getInertiaTensorInverseWorld();

	// Compute the vector from body center to the anchor point in world-space
	mR1World = orientationBody1 * mLocalAnchorPointBody1;
	mR2World = orientationBody2 * mLocalAnchorPointBody2;


    /***************************
	Vector3 p1 = x1 + mR1World;
	Vector3 p2 = x2 + mR2World;

	glPushMatrix();
	glTranslatef(p1.x, p1.y, p1.z);
	glutWireCube(0.5);
	glPopMatrix();


	glPushMatrix();
	glTranslatef(p2.x, p2.y, p2.z);
	glutWireCube(0.5);
	glPopMatrix();
	/*************************/

	// Compute the corresponding skew-symmetric matrices
	Matrix3x3 skewSymmetricMatrixU1= Matrix3x3::computeSkewSymmetricMatrixForCrossProduct(mR1World);
	Matrix3x3 skewSymmetricMatrixU2= Matrix3x3::computeSkewSymmetricMatrixForCrossProduct(mR2World);

	// Compute the matrix K=JM^-1J^t (3x3 matrix)
	scalar inverseMassBodies = Body1->mMassInverse + Body2->mMassInverse;
	Matrix3x3 massMatrix = Matrix3x3(inverseMassBodies, 0, 0,
			                         0, inverseMassBodies, 0,
			                         0, 0, inverseMassBodies) +
					                 skewSymmetricMatrixU1 * mI1 * skewSymmetricMatrixU1.getTranspose() +
				   	                 skewSymmetricMatrixU2 * mI2 * skewSymmetricMatrixU2.getTranspose();

	// Compute the inverse mass matrix K^-1
	mInverseMassMatrix.setToZero();
	if (mBody1->getType() == DYNAMIC || mBody2->getType() == DYNAMIC )
	{
		mInverseMassMatrix = massMatrix.getInverse();
	}

	// Compute the bias "b" of the constraint
	mBiasVector.setToZero();
    if (mPositionCorrectionTechnique == BAUMGARTE_JOINTS)
	{
		scalar biasFactor = BETA/timeStep;
		mBiasVector = biasFactor * (x2 + mR2World -
				                    x1 - mR1World);
	}


	// If warm-starting is not enabled
	if (!isWarmStartingActive)
	{
		// Reset the accumulated impulse
		mImpulse.setToZero();
	}
}




// Warm start the constraint (apply the previous impulse at the beginning of the step)
void rpBallAndSocketJoint::warmstart()
{

	// Compute the impulse P=J^T * lambda for the body 1
	const Vector3 linearImpulseBody1 = -mImpulse;
	const Vector3 linearImpulseBody2 =  mImpulse;

	// Compute the impulse P=J^T * lambda for the body 2
	const Vector3 angularImpulseBody1 =  mImpulse.cross(mR1World);
	const Vector3 angularImpulseBody2 = -mImpulse.cross(mR2World);

	// Apply the impulse to the body 1
	Body1->applyImpulseLinear(linearImpulseBody1);
	Body1->applyImpulseAngular(angularImpulseBody1);

	// Apply the impulse to the body to the body 2
	Body2->applyImpulseLinear(linearImpulseBody2);
	Body2->applyImpulseAngular(angularImpulseBody2);

}




// Solve the velocity constraint
void rpBallAndSocketJoint::solveVelocityConstraint()
{

	// Get the velocities
	const Vector3& v1 = Body1->mLinearVelocity;
	const Vector3& v2 = Body2->mLinearVelocity;
	const Vector3& w1 = Body1->mAngularVelocity;
	const Vector3& w2 = Body2->mAngularVelocity;

	// Compute J*v
	const Vector3 Jv = v2 + w2.cross(mR2World) -
			           v1 - w1.cross(mR1World);



	// Compute the Lagrange multiplier lambda
    const Vector3 deltaLambda = mInverseMassMatrix * (-Jv - mBiasVector);

    mImpulse += (deltaLambda - (deltaLambda.getUnit() * mImpulse.length() * softness));

	// Compute the impulse P=J^T * lambda for the body 1
	const Vector3 linearImpulseBody1 = -deltaLambda;
	const Vector3 linearImpulseBody2 =  deltaLambda;

	// Compute the impulse P=J^T * lambda for the body 2
	const Vector3 angularImpulseBody1 =  deltaLambda.cross(mR1World);
	const Vector3 angularImpulseBody2 = -deltaLambda.cross(mR2World);

	// Apply the impulse to the body 1
	Body1->applyImpulseLinear(linearImpulseBody1);
	Body1->applyImpulseAngular(angularImpulseBody1);

	// Apply the impulse to the body 2
	Body2->applyImpulseLinear(linearImpulseBody2);
	Body2->applyImpulseAngular(angularImpulseBody2);

}




// Solve the position constraint (for position error correction)
void rpBallAndSocketJoint::solvePositionConstraint()
{

    // If the error position correction technique is not the non-linear-gauss-seidel, we do
    // do not execute this method
    if (mPositionCorrectionTechnique != NON_LINEAR_GAUSS_SEIDEL) return;

    // Get the bodies center of mass and orientations
    Vector3    x1 = Body1->mTransform.getPosition4().getPos();
    Vector3    x2 = Body2->mTransform.getPosition4().getPos();
    Quaternion q1 = Body1->mTransform.getOrientation();
    Quaternion q2 = Body2->mTransform.getOrientation();

    // Get the inverse mass and inverse inertia tensors of the bodies
    scalar inverseMassBody1 = Body1->mMassInverse;
    scalar inverseMassBody2 = Body2->mMassInverse;

    // Recompute the inverse inertia tensors
    mI1 = Body1->getInertiaTensorInverseWorld();
    mI2 = Body2->getInertiaTensorInverseWorld();

    // Compute the vector from body center to the anchor point in world-space
    mR1World = q1 * mLocalAnchorPointBody1;
    mR2World = q2 * mLocalAnchorPointBody2;

    // Compute the corresponding skew-symmetric matrices
    Matrix3x3 skewSymmetricMatrixU1= Matrix3x3::computeSkewSymmetricMatrixForCrossProduct(mR1World);
    Matrix3x3 skewSymmetricMatrixU2= Matrix3x3::computeSkewSymmetricMatrixForCrossProduct(mR2World);

    // Recompute the inverse mass matrix K=J^TM^-1J of of the 3 translation constraints
    scalar inverseMassBodies = inverseMassBody1 + inverseMassBody2;
    Matrix3x3 massMatrix = Matrix3x3(inverseMassBodies, 0, 0,
                                     0, inverseMassBodies, 0,
                                     0, 0, inverseMassBodies) +
            skewSymmetricMatrixU1 * mI1 * skewSymmetricMatrixU1.getTranspose() +
            skewSymmetricMatrixU2 * mI2 * skewSymmetricMatrixU2.getTranspose();

    mInverseMassMatrix.setToZero();
    if (mBody1->getType() == DYNAMIC || mBody2->getType() == DYNAMIC)
    {
        mInverseMassMatrix = massMatrix.getInverse();
    }




    if(isSplitActive)
    {

        const Vector3 v1p = Body1->mSplitLinearVelocity;
        const Vector3 w1p = Body1->mSplitAngularVelocity;

        const Vector3 v2p = Body2->mSplitLinearVelocity;
        const Vector3 w2p = Body2->mSplitAngularVelocity;

        // Compute J*v
        const Vector3 Jv = v2p + w2p.cross(mR2World) -
                           v1p - w1p.cross(mR1World);


        const Vector3 lambda = mInverseMassMatrix * ( -Jv );


        // Compute the impulse of body
        const Vector3 linearImpulseBody1 = -lambda;
        const Vector3 linearImpulseBody2 =  lambda;

        // Compute the impulse of body
        const Vector3 angularImpulseBody1 =  lambda.cross(mR1World);
        const Vector3 angularImpulseBody2 = -lambda.cross(mR2World);

        // Apply the impulse to the body 1
        Body1->applySplitImpulseLinear(linearImpulseBody1 );
        Body1->applySplitImpulseAngular(angularImpulseBody1 );

        // Apply the impulse to the body 2
        Body2->applySplitImpulseLinear(linearImpulseBody2 );
        Body2->applySplitImpulseAngular(angularImpulseBody2 );
    }




    // Compute the constraint error (value of the C(x) function)
    Vector3 constraintError = (x2 + mR2World -
                               x1 - mR1World);


    // Relaxation offset damping to the constraint error
    scalar dampingRelaxation = 0.0008;
    if( constraintError.length() > dampingRelaxation)
    {
        constraintError -= ( constraintError.getUnit() * scalar(1.0 - dampingRelaxation));
    }
    else
    {
        constraintError = Vector3::ZERO;
    }



    // Compute the Lagrange multiplier lambda
    // TODO : Do not solve the system by computing the inverse each time and multiplying with the
    //        right-hand side vector but instead use a method to directly solve the linear system.
    const Vector3 lambda = mInverseMassMatrix * ( -constraintError  );




    // Compute the impulse of body
    const Vector3 linearImpulseBody1 = -lambda;
    const Vector3 linearImpulseBody2 =  lambda;

    // Compute the impulse of body
    const Vector3 angularImpulseBody1 =  lambda.cross(mR1World);
    const Vector3 angularImpulseBody2 = -lambda.cross(mR2World);


    // Compute the pseudo velocity of body 1
    const Vector3 v1 = inverseMassBody1 * linearImpulseBody1;
    const Vector3 w1 = mI1 * angularImpulseBody1;

    // Compute the pseudo velocity of body 2
    const Vector3 v2 = inverseMassBody2 * linearImpulseBody2;
    const Vector3 w2 = mI2 * angularImpulseBody2;



    // Update the body center of mass and orientation
    Body1->setWorldTransform(TransformUtil::integrateTransform( Body1->mTransform , v1 , w1 , 1.0  ));
    Body2->setWorldTransform(TransformUtil::integrateTransform( Body2->mTransform , v2 , w2 , 1.0  ));


    /**

    // Update the body center of mass and orientation of body 1
    x1 += v1;
    q1 += Quaternion(0, w1) * q1 * scalar(0.5);
    q1.normalize();



    // Update the body position/orientation of body 2
    x2 += v2;
    q2 += Quaternion(0, w2) * q2 * scalar(0.5);
    q2.normalize();


    Body1->setWorldTransform(Transform(x1,q1));
    Body2->setWorldTransform(Transform(x2,q2));

    /**/


}



} /* namespace real_physics */
