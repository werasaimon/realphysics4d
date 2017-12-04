/*
 * rpJoint.h
 *
 *  Created on: 8 янв. 2017 г.
 *      Author: wera
 */

#ifndef SOURCE_ENGIE_PHYSICS_SOLVER_CONSTRAINTJOINT_RPJOINT_H_
#define SOURCE_ENGIE_PHYSICS_SOLVER_CONSTRAINTJOINT_RPJOINT_H_



#include "../../Body/Dynamics/rpPhysicsBody.h"
#include "../../Body/Dynamics/rpPhysicsRigidBody.h"

namespace real_physics
{



enum JointType { BALLSOCKETJOINT,
                 SLIDERJOINT,
                 HINGEJOINT ,
                 FIXEDJOINT ,
                 DISTANCEJOINT ,
                 AXISJOINT};


// Class declarations
struct ConstraintSolverData;
class  rpJoint;


struct rpJointInfo
{

    public:

        // -------------------- Attributes -------------------- //

        /// First rigid body of the rpJoint
        rpPhysicsBody* body1;

        /// Second rigid body of the rpJoint
        rpPhysicsBody* body2;

        /// Type of the rpJoint
        JointType type;

        /// Position correction technique used for the constraint (used for joints).
        /// By default, the BAUMGARTE technique is used
        JointsPositionCorrectionTechnique positionCorrectionTechnique;

        /// True if the two bodies of the joint are allowed to collide with each other
        bool isCollisionEnabled;

        /// Constructor
        rpJointInfo(JointType constraintType)
                      : body1(NULL), body2(NULL), type(constraintType),
                        positionCorrectionTechnique(NON_LINEAR_GAUSS_SEIDEL),
                        isCollisionEnabled(true)
        {
        }

        /// Constructor
        rpJointInfo(rpPhysicsBody* rigidBody1, rpPhysicsBody* rigidBody2, JointType constraintType)
                      : body1(rigidBody1), body2(rigidBody2), type(constraintType),
                        positionCorrectionTechnique(NON_LINEAR_GAUSS_SEIDEL),
                        isCollisionEnabled(true)
        {
        }

        /// Destructor
        virtual ~rpJointInfo() {}

};



// Class rpJoint
/**
 * This abstract class represents a rpJoint between two bodies.
 */
class rpJoint
{

  //  protected:

      public:


        uint mIndex = 0;
        // -------------------- Attributes -------------------- //

        /// Pointer to the first body of the rpJoint
        rpPhysicsBody* const mBody1;

        /// Pointer to the second body of the rpJoint
        rpPhysicsBody* const mBody2;

        /// Type of the rpJoint
        const JointType mType;

        /// Body 1 index in the velocity array to solve the constraint
        uint mIndexBody1;

        /// Body 2 index in the velocity array to solve the constraint
        uint mIndexBody2;

        /// Position correction technique used for the constraint (used for joints)
        JointsPositionCorrectionTechnique mPositionCorrectionTechnique;

        /// True if the two bodies of the constraint are allowed to collide with each other
        bool mIsCollisionEnabled;

        /// True if the rpJoint has already been added into an island
        bool mIsAlreadyInIsland;


        /****************************/
        scalar biasFactor;
        scalar softness; //0.05f;0.0f;
        /***************************/

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        rpJoint(const rpJoint& constraint);

        /// Private assignment operator
        rpJoint& operator=(const rpJoint& constraint);

        /// Return true if the rpJoint has already been added into an island
        bool isAlreadyInIsland() const;

        /// Return the number of bytes used by the rpJoint
        virtual size_t getSizeInBytes() const {}

        /// Initialize before solving the rpJoint
        virtual void initBeforeSolve( scalar tirmStep ) {}

        /// Warm start the rpJoint (apply the previous impulse at the beginning of the step)
        virtual void warmstart() {}

        /// Solve the velocity constraint
        virtual void solveVelocityConstraint() {}

        /// Solve the position constraint
        virtual void solvePositionConstraint() {}

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        rpJoint(const rpJointInfo& jointInfo);

        /// Destructor
        virtual ~rpJoint();

        /// Return the reference to the body 1
        rpPhysicsBody* getBody1() const;

        /// Return the reference to the body 2
        rpPhysicsBody* getBody2() const;

        /// Return true if the constraint is active
        bool isActive() const;

        /// Return the type of the constraint
        JointType getType() const;

        /// Return true if the collision between the two bodies of the rpJoint is enabled
        bool isCollisionEnabled() const;

        /// set false if the collision between the two bodies of the rpJoint is enabled
        void setIsCollisionEnabled(bool isCollisionEnabled);

        /// set stoffnes constraint-joint
        void setSoftness(const scalar &value);


        // -------------------- Friendship -------------------- //
        friend class rpDynamicsWorld;
        friend class rpIsland;
        friend class rpConstraintSolver;



};



// Return the reference to the body 1
/**
 * @return The first body involved in the rpJoint
 */
SIMD_INLINE rpPhysicsBody* rpJoint::getBody1() const
{
    return mBody1;
}

// Return the reference to the body 2
/**
 * @return The second body involved in the rpJoint
 */

SIMD_INLINE rpPhysicsBody* rpJoint::getBody2() const
{
    return mBody2;
}

// Return true if the rpJoint is active
/**
 * @return True if the rpJoint is active
 */
SIMD_INLINE bool rpJoint::isActive() const
{
    return (mBody1->isActive() &&
            mBody2->isActive());
}

// Return the type of the rpJoint
/**
 * @return The type of the rpJoint
 */
SIMD_INLINE JointType rpJoint::getType() const
{
    return mType;
}

// Return true if the collision between the two bodies of the rpJoint is enabled
/**
 * @return True if the collision is enabled between the two bodies of the rpJoint
 *              is enabled and false otherwise
 */
SIMD_INLINE bool rpJoint::isCollisionEnabled() const
{
    return mIsCollisionEnabled;
}


SIMD_INLINE void rpJoint::setIsCollisionEnabled(bool isCollisionEnabled)
{
    mIsCollisionEnabled = isCollisionEnabled;
}


// Return true if the rpJoint has already been added into an island
SIMD_INLINE bool rpJoint::isAlreadyInIsland() const
{
    return mIsAlreadyInIsland;
}





} /* namespace real_physics */

#endif /* SOURCE_ENGIE_PHYSICS_SOLVER_CONSTRAINTJOINT_RPJOINT_H_ */
