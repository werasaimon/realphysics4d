/*
 * rpDistanceJoint.h
 *
 *  Created on: 8 янв. 2017 г.
 *      Author: wera
 */

#ifndef SOURCE_ENGIE_KINEMATICPHYSICS_CONSTRAINT_RPDISTANCEJOINT_H_
#define SOURCE_ENGIE_KINEMATICPHYSICS_CONSTRAINT_RPDISTANCEJOINT_H_

#include "rpJoint.h"
#include "../../LinearMaths/rpLinearMtah.h"

namespace real_physics
{



struct rpDistanceJointInfo : public rpJointInfo
{

    public :

        // -------------------- Attributes -------------------- //
        scalar distanceWorldSpace;

        rpDistanceJointInfo(rpPhysicsBody* rigidBody1,
                            rpPhysicsBody* rigidBody2,
                            const scalar& initDistanceWorldSpace = 0)
                       :rpJointInfo(rigidBody1, rigidBody2, DISTANCEJOINT),
                         distanceWorldSpace(initDistanceWorldSpace)
        {
            if(initDistanceWorldSpace == 0)
            {

            }
        }
};


class rpDistanceJoint : public rpJoint
{


	private:

		enum DistanceBehavior
		{
			LimitDistance, LimitMaximumDistance, LimitMinimumDistance,
		}behavior;


		Vector3 jacobian[4];

		Vector3 mR1World;
		Vector3 mR2World;

		scalar  mDistance;


		scalar  mBias;
		scalar  mLength;

		scalar  mAccumulatedImpulse;

		scalar softnessOverDt;
		scalar mEffectiveMass;


		rpRigidPhysicsBody *Body1;
		rpRigidPhysicsBody *Body2;

  public:

    rpDistanceJoint(const rpDistanceJointInfo& jointInfo) :
    rpJoint( jointInfo )
    {


        mDistance = jointInfo.distanceWorldSpace;


        mAccumulatedImpulse = 0.0f;
        mLength = 0;
        mBias = 0;


        this->biasFactor = 0.02f;
        this->softness = 0.0001f;

        Body1 = static_cast<rpRigidPhysicsBody*>(mBody1);
        Body2 = static_cast<rpRigidPhysicsBody*>(mBody2);


    }


    void initBeforeSolve( scalar timeStep );

    void warmstart();

    void solveVelocityConstraint();

    void solvePositionConstraint();


    /// Return the number of bytes used by the joint
    virtual size_t getSizeInBytes() const
    {
        return sizeof(rpDistanceJoint);
    }




};

} /* namespace real_physics */

#endif /* SOURCE_ENGIE_KINEMATICPHYSICS_CONSTRAINT_RPDISTANCEJOINT_H_ */
