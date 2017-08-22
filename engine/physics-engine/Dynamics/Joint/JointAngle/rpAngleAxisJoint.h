#ifndef RPANGLEAXISJOINT_H
#define RPANGLEAXISJOINT_H


#include "../../Joint/rpJoint.h"

namespace real_physics
{

struct AngleAxisJointInfo : public rpJointInfo
{

    Vector3 mLocalAxis;

    AngleAxisJointInfo(rpPhysicsBody* rigidBody1,
                       rpPhysicsBody* rigidBody2,
                       const Vector3& Axis )
        : rpJointInfo(rigidBody1, rigidBody2, JointType::AXISJOINT ) ,
          mLocalAxis(Axis)
    {
    }
};

class rpAngleAxisJoint : public rpJoint
{

    private:


        bool isSplitActive;
        bool isWarmStartingActive = true;

       //--------------------------//

        rpRigidPhysicsBody *Body1;
        rpRigidPhysicsBody *Body2;

       //--------------------------//

        Vector3   mAxis;

        Vector3   mLocalAxis1;
        Vector3   mLocalAxis2;

        Vector3   mLocalConstrAxis1;
        Vector3   mLocalConstrAxis2;

        Vector3   mWorldConstrAxis1;
        Vector3   mWorldConstrAxis2;

        Vector3   mAccumulatedImpulse;

        scalar    softnessOverDt;
        Matrix3x3 mEffectiveMass;

        Vector3   mBias;
        Vector3   mBias2;




        //-------------------- Methods --------------------//

        /// Private copy-constructor
        rpAngleAxisJoint(const rpAngleAxisJoint& constraint);

        /// Private assignment operator
        rpAngleAxisJoint& operator=(const rpAngleAxisJoint& constraint);




    public:


        rpAngleAxisJoint(const AngleAxisJointInfo& jointInfo)
        : rpJoint( jointInfo )
        {

            isWarmStartingActive = true;
            isSplitActive = true;

            /// Pair Body to collision solver
            Body1 = static_cast<rpRigidPhysicsBody*>(mBody1);
            Body2 = static_cast<rpRigidPhysicsBody*>(mBody2);


            biasFactor = 0.02f;
            softness = 0.001f; //0.05f;0.0f;

            mAccumulatedImpulse = Vector3::ZERO;
            mAxis = jointInfo.mLocalAxis;

            // Axis in body space
            mLocalAxis1 = mBody1->getTransform().getOrientation().getInverse() * mAxis;
            mLocalAxis2 = mBody2->getTransform().getOrientation().getInverse() * mAxis;

            mLocalConstrAxis1 = Vector3(0, 1, 0) ^ mLocalAxis1;
            if (mLocalConstrAxis1.length2() < 0.001f)
                mLocalConstrAxis1 = Vector3(1, 0, 0) ^ mLocalAxis1;

            mLocalConstrAxis2 = mLocalAxis1 ^ mLocalConstrAxis1;
            mLocalConstrAxis1.normalise();
            mLocalConstrAxis2.normalise();

        }


        /// Initialize before solving the rpJoint
        void initBeforeSolve( scalar tirmStep )
        {

            scalar dt = (1.f/1.f);
            /**/
            mEffectiveMass = Body1->getInertiaTensorWorld() +
                             Body2->getInertiaTensorWorld();

            softnessOverDt = softness / dt;

            mEffectiveMass[1][1] += softnessOverDt;
            mEffectiveMass[2][2] += softnessOverDt;
            mEffectiveMass[3][3] += softnessOverDt;

            mEffectiveMass = mEffectiveMass.getInverse();

            Matrix3x3 rot1 = Body1->getTransform().getBasis();
            Matrix3x3 rot2 = Body2->getTransform().getBasis();

            Vector3 worldAxis1 = rot1 * mLocalAxis1;
            Vector3 worldAxis2 = rot2 * mLocalAxis2;

            mWorldConstrAxis1 = rot1 * mLocalConstrAxis1;
            mWorldConstrAxis2 = rot2 * mLocalConstrAxis2;

            Vector3 error = worldAxis1.cross(worldAxis2);

            Vector3 errorAxis = Vector3(0, 0, 0);
            errorAxis.x = error.dot(mWorldConstrAxis1);
            errorAxis.y = error.dot(mWorldConstrAxis2);

            mBias  = Vector3(0, 0, 0);//errorAxis * biasFactor * (-1.0f / dt);
            mBias2 = errorAxis * (-1.0f / dt); // no bias
        }

        /// Warm start the rpJoint (apply the previous impulse at the beginning of the step)
        void warmstart()
        {
            Vector3 impulse;
            impulse.x = mWorldConstrAxis1.x * mAccumulatedImpulse.x + mWorldConstrAxis2.x * mAccumulatedImpulse.y;
            impulse.y = mWorldConstrAxis1.y * mAccumulatedImpulse.x + mWorldConstrAxis2.y * mAccumulatedImpulse.y;
            impulse.z = mWorldConstrAxis1.z * mAccumulatedImpulse.x + mWorldConstrAxis2.z * mAccumulatedImpulse.y;

            Body1->applyImpulseAngular( impulse );
            Body2->applyImpulseAngular(-impulse );
        }


        /// Solve the velocity constraint
        void solveVelocityConstraint()
        {
            Vector3 vd = Body1->mAngularVelocity -
                         Body2->mAngularVelocity;

            Vector3 jv = Vector3(0, 0, 0);
            jv.x = vd.dot(mWorldConstrAxis1);
            jv.y = vd.dot(mWorldConstrAxis2);

            Vector3 softnessVector = mAccumulatedImpulse * softnessOverDt;
            Vector3 lambda = mEffectiveMass * -(jv + mBias);

            mAccumulatedImpulse += lambda;

            Vector3 impulse;
            impulse.x = mWorldConstrAxis1.x * lambda.x + mWorldConstrAxis2.x * lambda.y;
            impulse.y = mWorldConstrAxis1.y * lambda.x + mWorldConstrAxis2.y * lambda.y;
            impulse.z = mWorldConstrAxis1.z * lambda.x + mWorldConstrAxis2.z * lambda.y;

            Body1->applyImpulseAngular( impulse );
            Body2->applyImpulseAngular(-impulse );

        }

        /// Solve the position constraint
        void solvePositionConstraint()
        {

            Vector3 vd = Body1->mSplitAngularVelocity -
                         Body2->mSplitAngularVelocity;

            Vector3 jv = Vector3(0, 0, 0);
            jv.x = vd.dot(mWorldConstrAxis1);
            jv.y = vd.dot(mWorldConstrAxis2);

            Vector3 lambda =  mEffectiveMass * -(jv + mBias2);

            Vector3 impulse;
            impulse.x = mWorldConstrAxis1.x * lambda.x + mWorldConstrAxis2.x * lambda.y;
            impulse.y = mWorldConstrAxis1.y * lambda.x + mWorldConstrAxis2.y * lambda.y;
            impulse.z = mWorldConstrAxis1.z * lambda.x + mWorldConstrAxis2.z * lambda.y;

            Body1->applySplitImpulseAngular(  impulse );
            Body2->applySplitImpulseAngular( -impulse );
        }
};


} /* namespace real_physics */


#endif // RPANGLEAXISJOINT_H
