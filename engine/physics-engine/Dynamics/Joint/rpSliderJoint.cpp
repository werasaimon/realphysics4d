/*
 * rpSliderJoint.cpp
 *
 *  Created on: 10 янв. 2017 г.
 *      Author: wera
 */

#include "rpSliderJoint.h"

namespace real_physics
{

// Static variables definition
const scalar rpSliderJoint::BETA = scalar(0.2);

// Constructor
rpSliderJoint::rpSliderJoint(const rpSliderJointInfo& jointInfo)
            : rpJoint(jointInfo), mImpulseTranslation(0, 0), mImpulseRotation(0, 0, 0),
              mImpulseLowerLimit(0), mImpulseUpperLimit(0), mImpulseMotor(0),
              mIsLimitEnabled(jointInfo.isLimitEnabled), mIsMotorEnabled(jointInfo.isMotorEnabled),
              mLowerLimit(jointInfo.minTranslationLimit),
              mUpperLimit(jointInfo.maxTranslationLimit), mIsLowerLimitViolated(false),
              mIsUpperLimitViolated(false), mMotorSpeed(jointInfo.motorSpeed),
              mMaxMotorForce(jointInfo.maxMotorForce){

    assert(mUpperLimit >= 0.0);
    assert(mLowerLimit <= 0.0);
    assert(mMaxMotorForce >= 0.0);

    isWarmStartingActive = true;

    Body1 = static_cast<rpRigidPhysicsBody*>(mBody1);
    Body2 = static_cast<rpRigidPhysicsBody*>(mBody2);



    // Compute the local-space anchor point for each body
    const Transform& transform1 = mBody1->getTransform();
    const Transform& transform2 = mBody2->getTransform();
    mLocalAnchorPointBody1 = transform1.getInverse() * jointInfo.anchorPointWorldSpace;
    mLocalAnchorPointBody2 = transform2.getInverse() * jointInfo.anchorPointWorldSpace;

    // Compute the inverse of the initial orientation difference between the two bodies
    mInitOrientationDifferenceInv = transform2.getOrientation() *
                                    transform1.getOrientation().getInverse();
    mInitOrientationDifferenceInv.normalize();
    mInitOrientationDifferenceInv.inverse();

    // Compute the slider axis in local-space of body 1
    mSliderAxisBody1 = mBody1->getTransform().getOrientation().getInverse() *  jointInfo.sliderAxisWorldSpace;
    mSliderAxisBody1.normalize();
}

// Destructor
rpSliderJoint::~rpSliderJoint()
{

}

// Initialize before solving the constraint
void rpSliderJoint::initBeforeSolve( scalar timeStep )
{

    // Get the bodies positions and orientations
    const Vector3& x1 = Body1->mCenterOfMassWorld;
    const Vector3& x2 = Body2->mCenterOfMassWorld;
    const Quaternion& orientationBody1 = Body1->getTransform().getOrientation();
    const Quaternion& orientationBody2 = Body2->getTransform().getOrientation();

    // Get the inertia tensor of bodies
    mI1 = Body1->getInertiaTensorInverseWorld();
    mI2 = Body2->getInertiaTensorInverseWorld();

    // Vector from body center to the anchor point
    mR1 = orientationBody1 * mLocalAnchorPointBody1;
    mR2 = orientationBody2 * mLocalAnchorPointBody2;

    // Compute the vector u (difference between anchor points)
    const Vector3 u = x2 + mR2 - x1 - mR1;

    // Compute the two orthogonal vectors to the slider axis in world-space
    mSliderAxisWorld = orientationBody1 * mSliderAxisBody1;
    mSliderAxisWorld.normalize();
    mN1 = mSliderAxisWorld.getOneUnitOrthogonalVector();
    mN2 = mSliderAxisWorld.cross(mN1);

    // Check if the limit constraints are violated or not
    scalar uDotSliderAxis = u.dot(mSliderAxisWorld);
    scalar lowerLimitError = uDotSliderAxis - mLowerLimit;
    scalar upperLimitError = mUpperLimit - uDotSliderAxis;
    bool oldIsLowerLimitViolated = mIsLowerLimitViolated;
    mIsLowerLimitViolated = lowerLimitError <= 0;
    if (mIsLowerLimitViolated != oldIsLowerLimitViolated)
    {
        mImpulseLowerLimit = 0.0;
    }
    bool oldIsUpperLimitViolated = mIsUpperLimitViolated;
    mIsUpperLimitViolated = upperLimitError <= 0;
    if (mIsUpperLimitViolated != oldIsUpperLimitViolated)
    {
        mImpulseUpperLimit = 0.0;
    }

    // Compute the cross products used in the Jacobians
    mR2CrossN1 = mR2.cross(mN1);
    mR2CrossN2 = mR2.cross(mN2);
    mR2CrossSliderAxis = mR2.cross(mSliderAxisWorld);
    const Vector3 r1PlusU = mR1 + u;
    mR1PlusUCrossN1 = (r1PlusU).cross(mN1);
    mR1PlusUCrossN2 = (r1PlusU).cross(mN2);
    mR1PlusUCrossSliderAxis = (r1PlusU).cross(mSliderAxisWorld);

    // Compute the inverse of the mass matrix K=JM^-1J^t for the 2 translation
    // constraints (2x2 matrix)
    scalar sumInverseMass = Body1->mMassInverse + Body2->mMassInverse;
    Vector3 I1R1PlusUCrossN1 = mI1 * mR1PlusUCrossN1;
    Vector3 I1R1PlusUCrossN2 = mI1 * mR1PlusUCrossN2;
    Vector3 I2R2CrossN1 = mI2 * mR2CrossN1;
    Vector3 I2R2CrossN2 = mI2 * mR2CrossN2;
    const scalar el11 = sumInverseMass + mR1PlusUCrossN1.dot(I1R1PlusUCrossN1) + mR2CrossN1.dot(I2R2CrossN1);
    const scalar el12 = mR1PlusUCrossN1.dot(I1R1PlusUCrossN2) + mR2CrossN1.dot(I2R2CrossN2);
    const scalar el21 = mR1PlusUCrossN2.dot(I1R1PlusUCrossN1) + mR2CrossN2.dot(I2R2CrossN1);
    const scalar el22 = sumInverseMass + mR1PlusUCrossN2.dot(I1R1PlusUCrossN2) + mR2CrossN2.dot(I2R2CrossN2);
    Matrix2x2 matrixKTranslation(el11, el12, el21, el22);
    mInverseMassMatrixTranslationConstraint.setToZero();
    if (mBody1->getType() == DYNAMIC || mBody2->getType() == DYNAMIC)
    {
        mInverseMassMatrixTranslationConstraint = matrixKTranslation.getInverse();
    }

    // Compute the bias "b" of the translation constraint
    mBTranslation.setToZero();
    scalar biasFactor = (BETA/timeStep);
    if (mPositionCorrectionTechnique == BAUMGARTE_JOINTS)
    {
        mBTranslation.x = u.dot(mN1);
        mBTranslation.y = u.dot(mN2);
        mBTranslation *= biasFactor;
    }

    // Compute the inverse of the mass matrix K=JM^-1J^t for the 3 rotation
    // contraints (3x3 matrix)
    mInverseMassMatrixRotationConstraint = mI1 + mI2;
    if (mBody1->getType() == DYNAMIC || mBody2->getType() == DYNAMIC)
    {
        mInverseMassMatrixRotationConstraint = mInverseMassMatrixRotationConstraint.getInverse();
    }

    // Compute the bias "b" of the rotation constraint
    mBRotation.setToZero();
    if (mPositionCorrectionTechnique == BAUMGARTE_JOINTS)
    {
        Quaternion currentOrientationDifference = orientationBody2 * orientationBody1.getInverse();
        currentOrientationDifference.normalize();
        const Quaternion qError = currentOrientationDifference * mInitOrientationDifferenceInv;
        mBRotation = biasFactor * scalar(2.0) * qError.getVectorV();
    }

    // If the limits are enabled
    if (mIsLimitEnabled && (mIsLowerLimitViolated || mIsUpperLimitViolated))
    {

        // Compute the inverse of the mass matrix K=JM^-1J^t for the limits (1x1 matrix)
        mInverseMassMatrixLimit = Body1->mMassInverse + Body2->mMassInverse +
                                  mR1PlusUCrossSliderAxis.dot(mI1 * mR1PlusUCrossSliderAxis) +
                                  mR2CrossSliderAxis.dot(mI2 * mR2CrossSliderAxis);
        mInverseMassMatrixLimit = (mInverseMassMatrixLimit > 0.0) ?
                                  scalar(1.0) / mInverseMassMatrixLimit : scalar(0.0);

        // Compute the bias "b" of the lower limit constraint
        mBLowerLimit = 0.0;
        if (mPositionCorrectionTechnique == BAUMGARTE_JOINTS)
        {
            mBLowerLimit = biasFactor * lowerLimitError;
        }

        // Compute the bias "b" of the upper limit constraint
        mBUpperLimit = 0.0;
        if (mPositionCorrectionTechnique == BAUMGARTE_JOINTS)
        {
            mBUpperLimit = biasFactor * upperLimitError;
        }
    }

    // If the motor is enabled
    if (mIsMotorEnabled)
    {
        // Compute the inverse of mass matrix K=JM^-1J^t for the motor (1x1 matrix)
        mInverseMassMatrixMotor = Body1->mMassInverse + Body2->mMassInverse;
        mInverseMassMatrixMotor = (mInverseMassMatrixMotor > 0.0) ?
                    scalar(1.0) / mInverseMassMatrixMotor : scalar(0.0);
    }

    // If warm-starting is not enabled
    if (!isWarmStartingActive)
    {
        // Reset all the accumulated impulses
        mImpulseTranslation.setToZero();
        mImpulseRotation.setToZero();
        mImpulseLowerLimit = 0.0;
        mImpulseUpperLimit = 0.0;
        mImpulseMotor = 0.0;
    }
}

// Warm start the constraint (apply the previous impulse at the beginning of the step)
void rpSliderJoint::warmstart()
{

	// Get the velocities
	Vector3& v1 = Body1->mLinearVelocity;
	Vector3& v2 = Body2->mLinearVelocity;
	Vector3& w1 = Body1->mAngularVelocity;
	Vector3& w2 = Body2->mAngularVelocity;

    // Get the inverse mass and inverse inertia tensors of the bodies
    const scalar inverseMassBody1 = Body1->mMassInverse;
    const scalar inverseMassBody2 = Body2->mMassInverse;

    // Compute the impulse P=J^T * lambda for the lower and upper limits constraints of body 1
    scalar impulseLimits = mImpulseUpperLimit - mImpulseLowerLimit;
    Vector3 linearImpulseLimits = impulseLimits * mSliderAxisWorld;

    // Compute the impulse P=J^T * lambda for the motor constraint of body 1
    Vector3 impulseMotor = mImpulseMotor * mSliderAxisWorld;

    // Compute the impulse P=J^T * lambda for the 2 translation constraints of body 1
    Vector3 linearImpulseBody1 = -mN1 * mImpulseTranslation.x - mN2 * mImpulseTranslation.y;
    Vector3 angularImpulseBody1 = -mR1PlusUCrossN1 * mImpulseTranslation.x -
                                   mR1PlusUCrossN2 * mImpulseTranslation.y;

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 1
    angularImpulseBody1 += -mImpulseRotation;

    // Compute the impulse P=J^T * lambda for the lower and upper limits constraints of body 1
    linearImpulseBody1 += linearImpulseLimits;
    angularImpulseBody1 += impulseLimits * mR1PlusUCrossSliderAxis;

    // Compute the impulse P=J^T * lambda for the motor constraint of body 1
    linearImpulseBody1 += impulseMotor;

    // Apply the impulse to the body 1
    v1 += inverseMassBody1 * linearImpulseBody1;
    w1 += mI1 * angularImpulseBody1;

    // Compute the impulse P=J^T * lambda for the 2 translation constraints of body 2
    Vector3 linearImpulseBody2 = mN1 * mImpulseTranslation.x + mN2 * mImpulseTranslation.y;
    Vector3 angularImpulseBody2 = mR2CrossN1 * mImpulseTranslation.x +
                                  mR2CrossN2 * mImpulseTranslation.y;

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 2
    angularImpulseBody2 += mImpulseRotation;

    // Compute the impulse P=J^T * lambda for the lower and upper limits constraints of body 2
    linearImpulseBody2 += -linearImpulseLimits;
    angularImpulseBody2 += -impulseLimits * mR2CrossSliderAxis;

    // Compute the impulse P=J^T * lambda for the motor constraint of body 2
    linearImpulseBody2 += -impulseMotor;

    // Apply the impulse to the body 2
    v2 += inverseMassBody2 * linearImpulseBody2;
    w2 += mI2 * angularImpulseBody2;
}

// Solve the velocity constraint
void rpSliderJoint::solveVelocityConstraint()
{

	// Get the velocities
	Vector3& v1 = Body1->mLinearVelocity;
	Vector3& v2 = Body2->mLinearVelocity;
	Vector3& w1 = Body1->mAngularVelocity;
	Vector3& w2 = Body2->mAngularVelocity;

    // Get the inverse mass and inverse inertia tensors of the bodies
    scalar inverseMassBody1 = Body1->mMassInverse;
    scalar inverseMassBody2 = Body2->mMassInverse;

    // --------------- Translation Constraints --------------- //

    // Compute J*v for the 2 translation constraints
    const scalar el1 = -mN1.dot(v1) - w1.dot(mR1PlusUCrossN1) + mN1.dot(v2) + w2.dot(mR2CrossN1);
    const scalar el2 = -mN2.dot(v1) - w1.dot(mR1PlusUCrossN2) + mN2.dot(v2) + w2.dot(mR2CrossN2);
    const Vector2 JvTranslation(el1, el2);

    // Compute the Lagrange multiplier lambda for the 2 translation constraints
    Vector2 deltaLambda = mInverseMassMatrixTranslationConstraint * (-JvTranslation -mBTranslation);
    mImpulseTranslation += deltaLambda;

    // Compute the impulse P=J^T * lambda for the 2 translation constraints of body 1
    const Vector3 linearImpulseBody1 = -mN1 * deltaLambda.x - mN2 * deltaLambda.y;
    Vector3 angularImpulseBody1 = -mR1PlusUCrossN1 * deltaLambda.x -
                                   mR1PlusUCrossN2 * deltaLambda.y;

    // Apply the impulse to the body 1
    v1 += inverseMassBody1 * linearImpulseBody1;
    w1 += mI1 * angularImpulseBody1;

    // Compute the impulse P=J^T * lambda for the 2 translation constraints of body 2
    const Vector3 linearImpulseBody2 = mN1 * deltaLambda.x + mN2 * deltaLambda.y;
    Vector3 angularImpulseBody2 = mR2CrossN1 * deltaLambda.x + mR2CrossN2 * deltaLambda.y;

    // Apply the impulse to the body 2
    v2 += inverseMassBody2 * linearImpulseBody2;
    w2 += mI2 * angularImpulseBody2;

    // --------------- Rotation Constraints --------------- //

    // Compute J*v for the 3 rotation constraints
    const Vector3 JvRotation = w2 - w1;

    // Compute the Lagrange multiplier lambda for the 3 rotation constraints
    Vector3 deltaLambda2 = mInverseMassMatrixRotationConstraint * (-JvRotation - mBRotation);
    mImpulseRotation += deltaLambda2;

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 1
    angularImpulseBody1 = -deltaLambda2;

    // Apply the impulse to the body to body 1
    w1 += mI1 * angularImpulseBody1;

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 2
    angularImpulseBody2 = deltaLambda2;

    // Apply the impulse to the body 2
    w2 += mI2 * angularImpulseBody2;

    // --------------- Limits Constraints --------------- //

    if (mIsLimitEnabled)
    {

        // If the lower limit is violated
        if (mIsLowerLimitViolated)
        {
            // Compute J*v for the lower limit constraint
            const scalar JvLowerLimit = mSliderAxisWorld.dot(v2) + mR2CrossSliderAxis.dot(w2) -
                                        mSliderAxisWorld.dot(v1) - mR1PlusUCrossSliderAxis.dot(w1);

            // Compute the Lagrange multiplier lambda for the lower limit constraint
            scalar deltaLambdaLower = mInverseMassMatrixLimit * (-JvLowerLimit -mBLowerLimit);
            scalar lambdaTemp = mImpulseLowerLimit;
            mImpulseLowerLimit = Max(mImpulseLowerLimit + deltaLambdaLower, scalar(0.0));
            deltaLambdaLower = mImpulseLowerLimit - lambdaTemp;

            // Compute the impulse P=J^T * lambda for the lower limit constraint of body 1
            const Vector3 linearImpulseBody1 = -deltaLambdaLower * mSliderAxisWorld;
            const Vector3 angularImpulseBody1 = -deltaLambdaLower * mR1PlusUCrossSliderAxis;

            // Apply the impulse to the body 1
            v1 += inverseMassBody1 * linearImpulseBody1;
            w1 += mI1 * angularImpulseBody1;

            // Compute the impulse P=J^T * lambda for the lower limit constraint of body 2
            const Vector3 linearImpulseBody2 = deltaLambdaLower * mSliderAxisWorld;
            const Vector3 angularImpulseBody2 = deltaLambdaLower * mR2CrossSliderAxis;

            // Apply the impulse to the body 2
            v2 += inverseMassBody2 * linearImpulseBody2;
            w2 += mI2 * angularImpulseBody2;
        }

        // If the upper limit is violated
        if (mIsUpperLimitViolated)
        {

            // Compute J*v for the upper limit constraint
            const scalar JvUpperLimit = mSliderAxisWorld.dot(v1) + mR1PlusUCrossSliderAxis.dot(w1)
                                      - mSliderAxisWorld.dot(v2) - mR2CrossSliderAxis.dot(w2);

            // Compute the Lagrange multiplier lambda for the upper limit constraint
            scalar deltaLambdaUpper = mInverseMassMatrixLimit * (-JvUpperLimit -mBUpperLimit);
            scalar lambdaTemp = mImpulseUpperLimit;
            mImpulseUpperLimit = Max(mImpulseUpperLimit + deltaLambdaUpper, scalar(0.0));
            deltaLambdaUpper = mImpulseUpperLimit - lambdaTemp;

            // Compute the impulse P=J^T * lambda for the upper limit constraint of body 1
            const Vector3 linearImpulseBody1 = deltaLambdaUpper * mSliderAxisWorld;
            const Vector3 angularImpulseBody1 = deltaLambdaUpper * mR1PlusUCrossSliderAxis;

            // Apply the impulse to the body 1
            v1 += inverseMassBody1 * linearImpulseBody1;
            w1 += mI1 * angularImpulseBody1;

            // Compute the impulse P=J^T * lambda for the upper limit constraint of body 2
            const Vector3 linearImpulseBody2 = -deltaLambdaUpper * mSliderAxisWorld;
            const Vector3 angularImpulseBody2 = -deltaLambdaUpper * mR2CrossSliderAxis;

            // Apply the impulse to the body 2
            v2 += inverseMassBody2 * linearImpulseBody2;
            w2 += mI2 * angularImpulseBody2;
        }
    }

    // --------------- Motor --------------- //

    if (mIsMotorEnabled)
    {

        // Compute J*v for the motor
        const scalar JvMotor = mSliderAxisWorld.dot(v1) - mSliderAxisWorld.dot(v2);

        // Compute the Lagrange multiplier lambda for the motor
        const scalar maxMotorImpulse = mMaxMotorForce;
        scalar deltaLambdaMotor = mInverseMassMatrixMotor * (-JvMotor - mMotorSpeed);
        scalar lambdaTemp = mImpulseMotor;
        mImpulseMotor = clamp(mImpulseMotor + deltaLambdaMotor, -maxMotorImpulse, maxMotorImpulse);
        deltaLambdaMotor = mImpulseMotor - lambdaTemp;

        // Compute the impulse P=J^T * lambda for the motor of body 1
        const Vector3 linearImpulseBody1 = deltaLambdaMotor * mSliderAxisWorld;

        // Apply the impulse to the body 1
        v1 += inverseMassBody1 * linearImpulseBody1;

        // Compute the impulse P=J^T * lambda for the motor of body 2
        const Vector3 linearImpulseBody2 = -deltaLambdaMotor * mSliderAxisWorld;

        // Apply the impulse to the body 2
        v2 += inverseMassBody2 * linearImpulseBody2;
    }
}

// Solve the position constraint (for position error correction)
void rpSliderJoint::solvePositionConstraint()
{

    // If the error position correction technique is not the non-linear-gauss-seidel, we do
    // do not execute this method
    if (mPositionCorrectionTechnique != NON_LINEAR_GAUSS_SEIDEL) return;

    // Get the bodies positions and orientations
    // Get the bodies positions and orientations
    Vector3    x1 = Body1->mTransform.getPosition();//constraintSolverData.positions[mIndexBody1];
    Vector3    x2 = Body2->mTransform.getPosition();//constraintSolverData.positions[mIndexBody2];
    Quaternion q1 = Body1->mTransform.getOrientation();//constraintSolverData.orientations[mIndexBody1];
    Quaternion q2 = Body2->mTransform.getOrientation();//constraintSolverData.orientations[mIndexBody2];

    // Get the inverse mass and inverse inertia tensors of the bodies
    scalar inverseMassBody1 = Body1->mMassInverse;
    scalar inverseMassBody2 = Body2->mMassInverse;

    // Recompute the inertia tensor of bodies
    mI1 = Body1->getInertiaTensorInverseWorld();
    mI2 = Body2->getInertiaTensorInverseWorld();

    // Vector from body center to the anchor point
    mR1 = q1 * mLocalAnchorPointBody1;
    mR2 = q2 * mLocalAnchorPointBody2;

    // Compute the vector u (difference between anchor points)
    const Vector3 u = x2 + mR2 - x1 - mR1;

    // Compute the two orthogonal vectors to the slider axis in world-space
    mSliderAxisWorld = q1 * mSliderAxisBody1;
    mSliderAxisWorld.normalize();
    mN1 = mSliderAxisWorld.getOneUnitOrthogonalVector();
    mN2 = mSliderAxisWorld.cross(mN1);

    // Check if the limit constraints are violated or not
    scalar uDotSliderAxis = u.dot(mSliderAxisWorld);
    scalar lowerLimitError = uDotSliderAxis - mLowerLimit;
    scalar upperLimitError = mUpperLimit - uDotSliderAxis;
    mIsLowerLimitViolated = lowerLimitError <= 0;
    mIsUpperLimitViolated = upperLimitError <= 0;

    // Compute the cross products used in the Jacobians
    mR2CrossN1 = mR2.cross(mN1);
    mR2CrossN2 = mR2.cross(mN2);
    mR2CrossSliderAxis = mR2.cross(mSliderAxisWorld);
    const Vector3 r1PlusU = mR1 + u;
    mR1PlusUCrossN1 = (r1PlusU).cross(mN1);
    mR1PlusUCrossN2 = (r1PlusU).cross(mN2);
    mR1PlusUCrossSliderAxis = (r1PlusU).cross(mSliderAxisWorld);

    // --------------- Translation Constraints --------------- //

    // Recompute the inverse of the mass matrix K=JM^-1J^t for the 2 translation
    // constraints (2x2 matrix)
    scalar sumInverseMass = Body1->mMassInverse + Body2->mMassInverse;
    Vector3 I1R1PlusUCrossN1 = mI1 * mR1PlusUCrossN1;
    Vector3 I1R1PlusUCrossN2 = mI1 * mR1PlusUCrossN2;
    Vector3 I2R2CrossN1 = mI2 * mR2CrossN1;
    Vector3 I2R2CrossN2 = mI2 * mR2CrossN2;
    const scalar el11 = sumInverseMass + mR1PlusUCrossN1.dot(I1R1PlusUCrossN1) + mR2CrossN1.dot(I2R2CrossN1);
    const scalar el12 = mR1PlusUCrossN1.dot(I1R1PlusUCrossN2) + mR2CrossN1.dot(I2R2CrossN2);
    const scalar el21 = mR1PlusUCrossN2.dot(I1R1PlusUCrossN1) + mR2CrossN2.dot(I2R2CrossN1);
    const scalar el22 = sumInverseMass + mR1PlusUCrossN2.dot(I1R1PlusUCrossN2) + mR2CrossN2.dot(I2R2CrossN2);
    Matrix2x2 matrixKTranslation(el11, el12, el21, el22);
    mInverseMassMatrixTranslationConstraint.setToZero();
    if (mBody1->getType() == DYNAMIC || mBody2->getType() == DYNAMIC)
    {
        mInverseMassMatrixTranslationConstraint = matrixKTranslation.getInverse();
    }

    // Compute the position error for the 2 translation constraints
    const Vector2 translationError(u.dot(mN1), u.dot(mN2));

    // Compute the Lagrange multiplier lambda for the 2 translation constraints
    Vector2 lambdaTranslation = mInverseMassMatrixTranslationConstraint * (-translationError);

    // Compute the impulse P=J^T * lambda for the 2 translation constraints of body 1
    const Vector3 linearImpulseBody1 = -mN1 * lambdaTranslation.x - mN2 * lambdaTranslation.y;
    Vector3 angularImpulseBody1 = -mR1PlusUCrossN1 * lambdaTranslation.x -
                                        mR1PlusUCrossN2 * lambdaTranslation.y;

    // Apply the impulse to the body 1
    const Vector3 v1 = inverseMassBody1 * linearImpulseBody1;
    Vector3 w1 = mI1 * angularImpulseBody1;

    // Update the body position/orientation of body 1
    x1 += v1;
    q1 += Quaternion(0, w1) * q1 * scalar(0.5);
    q1.normalize();

    // Compute the impulse P=J^T * lambda for the 2 translation constraints of body 2
    const Vector3 linearImpulseBody2 = mN1 * lambdaTranslation.x + mN2 * lambdaTranslation.y;
    Vector3 angularImpulseBody2 = mR2CrossN1 * lambdaTranslation.x +
            mR2CrossN2 * lambdaTranslation.y;

    // Apply the impulse to the body 2
    const Vector3 v2 = inverseMassBody2 * linearImpulseBody2;
    Vector3 w2 = mI2 * angularImpulseBody2;

    // Update the body position/orientation of body 2
    x2 += v2;
    q2 += Quaternion(0, w2) * q2 * scalar(0.5);
    q2.normalize();

    // --------------- Rotation Constraints --------------- //

    // Compute the inverse of the mass matrix K=JM^-1J^t for the 3 rotation
    // contraints (3x3 matrix)
    mInverseMassMatrixRotationConstraint = mI1 + mI2;
    if (mBody1->getType() == DYNAMIC || mBody2->getType() == DYNAMIC)
    {
        mInverseMassMatrixRotationConstraint = mInverseMassMatrixRotationConstraint.getInverse();
    }

    // Compute the position error for the 3 rotation constraints
    Quaternion currentOrientationDifference = q2 * q1.getInverse();
    currentOrientationDifference.normalize();
    const Quaternion qError = currentOrientationDifference * mInitOrientationDifferenceInv;
    const Vector3 errorRotation = scalar(2.0) * qError.getVectorV();

    // Compute the Lagrange multiplier lambda for the 3 rotation constraints
    Vector3 lambdaRotation = mInverseMassMatrixRotationConstraint * (-errorRotation);

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 1
    angularImpulseBody1 = -lambdaRotation;

    // Apply the impulse to the body 1
    w1 = mI1 * angularImpulseBody1;

    // Update the body position/orientation of body 1
    q1 += Quaternion(0, w1) * q1 * scalar(0.5);
    q1.normalize();

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 2
    angularImpulseBody2 = lambdaRotation;

    // Apply the impulse to the body 2
    w2 = mI2 * angularImpulseBody2;

    // Update the body position/orientation of body 2
    q2 += Quaternion(0, w2) * q2 * scalar(0.5);
    q2.normalize();

    // --------------- Limits Constraints --------------- //

    if (mIsLimitEnabled)
    {

        if (mIsLowerLimitViolated || mIsUpperLimitViolated)
        {

            // Compute the inverse of the mass matrix K=JM^-1J^t for the limits (1x1 matrix)
            mInverseMassMatrixLimit = Body1->mMassInverse + Body2->mMassInverse +
                                    mR1PlusUCrossSliderAxis.dot(mI1 * mR1PlusUCrossSliderAxis) +
                                    mR2CrossSliderAxis.dot(mI2 * mR2CrossSliderAxis);
            mInverseMassMatrixLimit = (mInverseMassMatrixLimit > 0.0) ?
                                      scalar(1.0) / mInverseMassMatrixLimit : scalar(0.0);
        }

        // If the lower limit is violated
        if (mIsLowerLimitViolated)
        {

            // Compute the Lagrange multiplier lambda for the lower limit constraint
            scalar lambdaLowerLimit = mInverseMassMatrixLimit * (-lowerLimitError);

            // Compute the impulse P=J^T * lambda for the lower limit constraint of body 1
            const Vector3 linearImpulseBody1 = -lambdaLowerLimit * mSliderAxisWorld;
            const Vector3 angularImpulseBody1 = -lambdaLowerLimit * mR1PlusUCrossSliderAxis;

            // Apply the impulse to the body 1
            const Vector3 v1 = inverseMassBody1 * linearImpulseBody1;
            const Vector3 w1 = mI1 * angularImpulseBody1;

            // Update the body position/orientation of body 1
            x1 += v1;
            q1 += Quaternion(0, w1) * q1 * scalar(0.5);
            q1.normalize();

            // Compute the impulse P=J^T * lambda for the lower limit constraint of body 2
            const Vector3 linearImpulseBody2 = lambdaLowerLimit * mSliderAxisWorld;
            const Vector3 angularImpulseBody2 = lambdaLowerLimit * mR2CrossSliderAxis;

            // Apply the impulse to the body 2
            const Vector3 v2 = inverseMassBody2 * linearImpulseBody2;
            const Vector3 w2 = mI2 * angularImpulseBody2;

            // Update the body position/orientation of body 2
            x2 += v2;
            q2 += Quaternion(0, w2) * q2 * scalar(0.5);
            q2.normalize();
        }

        // If the upper limit is violated
        if (mIsUpperLimitViolated)
        {

            // Compute the Lagrange multiplier lambda for the upper limit constraint
            scalar lambdaUpperLimit = mInverseMassMatrixLimit * (-upperLimitError);

            // Compute the impulse P=J^T * lambda for the upper limit constraint of body 1
            const Vector3 linearImpulseBody1 = lambdaUpperLimit * mSliderAxisWorld;
            const Vector3 angularImpulseBody1 = lambdaUpperLimit * mR1PlusUCrossSliderAxis;

            // Apply the impulse to the body 1
            const Vector3 v1 = inverseMassBody1 * linearImpulseBody1;
            const Vector3 w1 = mI1 * angularImpulseBody1;

            // Update the body position/orientation of body 1
            x1 += v1;
            q1 += Quaternion(0, w1) * q1 * scalar(0.5);
            q1.normalize();

            // Compute the impulse P=J^T * lambda for the upper limit constraint of body 2
            const Vector3 linearImpulseBody2 = -lambdaUpperLimit * mSliderAxisWorld;
            const Vector3 angularImpulseBody2 = -lambdaUpperLimit * mR2CrossSliderAxis;

            // Apply the impulse to the body 2
            const Vector3 v2 = inverseMassBody2 * linearImpulseBody2;
            const Vector3 w2 = mI2 * angularImpulseBody2;

            // Update the body position/orientation of body 2
            x2 += v2;
            q2 += Quaternion(0, w2) * q2 * scalar(0.5);
            q2.normalize();
        }
    }

    Body1->setTransform(Transform(x1,q1));
    Body2->setTransform(Transform(x2,q2));
}

// Enable/Disable the limits of the joint
/**
 * @param isLimitEnabled True if you want to enable the joint limits and false
 *                       otherwise
 */
void rpSliderJoint::enableLimit(bool isLimitEnabled)
{

    if (isLimitEnabled != mIsLimitEnabled)
    {

        mIsLimitEnabled = isLimitEnabled;

        // Reset the limits
        resetLimits();
    }
}

// Enable/Disable the motor of the joint
/**
 * @param isMotorEnabled True if you want to enable the joint motor and false
 *                       otherwise
 */
void rpSliderJoint::enableMotor(bool isMotorEnabled)
{

    mIsMotorEnabled = isMotorEnabled;
    mImpulseMotor = 0.0;

    // Wake up the two bodies of the joint
    mBody1->setIsSleeping(false);
    mBody2->setIsSleeping(false);
}

// Return the current translation value of the joint
/**
 * @return The current translation distance of the joint (in meters)
 */
scalar rpSliderJoint::getTranslation() const
{

    // TODO : Check if we need to compare rigid body position or center of mass here

    // Get the bodies positions and orientations
    const Vector3& x1 = mBody1->getTransform().getPosition();
    const Vector3& x2 = mBody2->getTransform().getPosition();
    const Quaternion& q1 = mBody1->getTransform().getOrientation();
    const Quaternion& q2 = mBody2->getTransform().getOrientation();

    // Compute the two anchor points in world-space coordinates
    const Vector3 anchorBody1 = x1 + q1 * mLocalAnchorPointBody1;
    const Vector3 anchorBody2 = x2 + q2 * mLocalAnchorPointBody2;

    // Compute the vector u (difference between anchor points)
    const Vector3 u = anchorBody2 - anchorBody1;

    // Compute the slider axis in world-space
    Vector3 sliderAxisWorld = q1 * mSliderAxisBody1;
    sliderAxisWorld.normalize();

    // Compute and return the translation value
    return u.dot(sliderAxisWorld);
}

// Set the minimum translation limit
/**
 * @param lowerLimit The minimum translation limit of the joint (in meters)
 */
void rpSliderJoint::setMinTranslationLimit(scalar lowerLimit)
{

    assert(lowerLimit <= mUpperLimit);

    if (lowerLimit != mLowerLimit)
    {
        mLowerLimit = lowerLimit;

        // Reset the limits
        resetLimits();
    }
}

// Set the maximum translation limit
/**
 * @param lowerLimit The maximum translation limit of the joint (in meters)
 */
void rpSliderJoint::setMaxTranslationLimit(scalar upperLimit)
{

    assert(mLowerLimit <= upperLimit);

    if (upperLimit != mUpperLimit)
    {
        mUpperLimit = upperLimit;

        // Reset the limits
        resetLimits();
    }
}

// Reset the limits
void rpSliderJoint::resetLimits()
{

    // Reset the accumulated impulses for the limits
    mImpulseLowerLimit = 0.0;
    mImpulseUpperLimit = 0.0;

    // Wake up the two bodies of the joint
    mBody1->setIsSleeping(false);
    mBody2->setIsSleeping(false);
}

// Set the motor speed
/**
 * @param motorSpeed The speed of the joint motor (in meters per second)
 */
void rpSliderJoint::setMotorSpeed(scalar motorSpeed)
{

    if (motorSpeed != mMotorSpeed)
    {

        mMotorSpeed = motorSpeed;

        // Wake up the two bodies of the joint
        mBody1->setIsSleeping(false);
        mBody2->setIsSleeping(false);
    }
}

// Set the maximum motor force
/**
 * @param maxMotorForce The maximum force of the joint motor (in Newton x meters)
 */
void rpSliderJoint::setMaxMotorForce(scalar maxMotorForce)
{

    if (maxMotorForce != mMaxMotorForce)
    {

        assert(mMaxMotorForce >= 0.0);
        mMaxMotorForce = maxMotorForce;

        // Wake up the two bodies of the joint
        mBody1->setIsSleeping(false);
        mBody2->setIsSleeping(false);
    }
}

} /* namespace real_physics */
