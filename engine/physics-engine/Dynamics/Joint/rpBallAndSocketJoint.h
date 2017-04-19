/*
 * rpBallAndSocketJoint.h
 *
 *  Created on: 9 янв. 2017 г.
 *      Author: wera
 */

#ifndef SOURCE_ENGIE_KINEMATICPHYSICS_CONSTRAINT_RPBALLANDSOCKETJOINT_H_
#define SOURCE_ENGIE_KINEMATICPHYSICS_CONSTRAINT_RPBALLANDSOCKETJOINT_H_

#include "rpJoint.h"

namespace real_physics
{


// Structure BallAndSocketJointInfo
/**
 * This structure is used to gather the information needed to create a ball-and-socket
 * joint. This structure will be used to create the actual ball-and-socket joint.
 */
struct rpBallAndSocketJointInfo : public rpJointInfo
{

    public :

        // -------------------- Attributes -------------------- //

        /// Anchor point (in world-space coordinates)
        Vector3 anchorPointWorldSpace;

        /// Constructor
        /**
         * @param rigidBody1 Pointer to the first body of the joint
         * @param rigidBody2 Pointer to the second body of the joint
         * @param initAnchorPointWorldSpace The anchor point in world-space
         *                                  coordinates
         */
        rpBallAndSocketJointInfo(rpPhysicsBody* rigidBody1, rpPhysicsBody* rigidBody2,
                                 const Vector3& initAnchorPointWorldSpace)
                               :rpJointInfo(rigidBody1, rigidBody2, BALLSOCKETJOINT),
                               anchorPointWorldSpace(initAnchorPointWorldSpace)
        {

        }
};




// Class BallAndSocketJoint
/**
 * This class represents a ball-and-socket joint that allows arbitrary rotation
 * between two bodies. This joint has three degrees of freedom. It can be used to
 * create a chain of bodies for instance.
 */
class rpBallAndSocketJoint : public rpJoint
{

    private :


	    bool isSplitActive;
	    bool isWarmStartingActive = true;

	    rpRigidPhysicsBody *Body1;
	    rpRigidPhysicsBody *Body2;

        // -------------------- Constants -------------------- //

        // Beta value for the bias factor of position correction
        static const scalar BETA;

        // -------------------- Attributes -------------------- //

        /// Anchor point of body 1 (in local-space coordinates of body 1)
        Vector3 mLocalAnchorPointBody1;

        /// Anchor point of body 2 (in local-space coordinates of body 2)
        Vector3 mLocalAnchorPointBody2;

        /// Vector from center of body 2 to anchor point in world-space
        Vector3 mR1World;

        /// Vector from center of body 2 to anchor point in world-space
        Vector3 mR2World;

        /// Inertia tensor of body 1 (in world-space coordinates)
        Matrix3x3 mI1;

        /// Inertia tensor of body 2 (in world-space coordinates)
        Matrix3x3 mI2;

        /// Bias vector for the constraint
        Vector3 mBiasVector;



        /// Inverse mass matrix K=JM^-1J^-t of the constraint
        Matrix3x3 mInverseMassMatrix;

        /// Accumulated impulse
        Vector3 mImpulse;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        rpBallAndSocketJoint(const rpBallAndSocketJoint& constraint);

        /// Private assignment operator
        rpBallAndSocketJoint& operator=(const rpBallAndSocketJoint& constraint);

        /// Return the number of bytes used by the joint
        virtual size_t getSizeInBytes() const;

        /// Initialize before solving the constraint
        virtual void initBeforeSolve( scalar timeStep );

        /// Warm start the constraint (apply the previous impulse at the beginning of the step)
        virtual void warmstart();

        /// Solve the velocity constraint
        virtual void solveVelocityConstraint();

        /// Solve the position constraint (for position error correction)
        virtual void solvePositionConstraint();

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        rpBallAndSocketJoint(const rpBallAndSocketJointInfo& jointInfo);

        /// Destructor
        virtual ~rpBallAndSocketJoint();
};


// Return the number of bytes used by the joint
SIMD_INLINE size_t rpBallAndSocketJoint::getSizeInBytes() const
{
    return sizeof(rpBallAndSocketJoint);
}




} /* namespace real_physics */

#endif /* SOURCE_ENGIE_KINEMATICPHYSICS_CONSTRAINT_RPBALLANDSOCKETJOINT_H_ */
