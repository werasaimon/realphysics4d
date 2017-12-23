/*
 * rpPhysicsRigidBody.cpp
 *
 *  Created on: 14 дек. 2016 г.
 *      Author: wera
 */

#include "rpPhysicsRigidBody.h"
#include "rpPhysicsObject.h"

#include <stddef.h>
#include <cassert>
#include <iostream>

#include "../../Collision/rpProxyShape.h"
#include "../../Collision/Shapes/rpCollisionShape.h"
#include "../../LinearMaths/mathematics.h"
#include "../../LinearMaths/rpLinearMtah.h"
#include "../../config.h"





using namespace std;





namespace real_physics
{


namespace
{
    /// Helper function for daping vector*s
    ////////////////////////////////////////////////////////////////
    static void Damping( Vector3& Velocity , const float& min_damping , const float& damping )
    {
        if( Velocity.length2()  < min_damping )
        {
            if( Velocity.length() > damping )
            {
                Velocity -= ( Velocity.getUnit() * damping);
            }
            else
            {
                Velocity  = Vector3::ZERO;
            }
        }
    }
    ////////////////////////////////////////////////////////////////

}


rpPhysicsRigidBody::rpPhysicsRigidBody(const Transform &transform, rpCollisionManager *CollideWorld, bodyindex id)
:rpPhysicsBody(transform, CollideWorld, id),
 mInitMass(scalar(1.0)),
 mCenterOfMassLocal(0, 0, 0),
 mCenterOfMassWorld(transform.getPosition4().getPos()),
 mIsGravityEnabled(true),
 mLinearDamping(scalar(0.004)),
 mAngularDamping(scalar(0.004)),
 mLinearVelocity(Vector3::ZERO),
 mAngularVelocity(Vector3::ZERO),
 mSplitLinearVelocity(Vector3::ZERO),
 mSplitAngularVelocity(Vector3::ZERO),
 mStepTime(0.0)
{
	/// body To type
	mTypePhysics = PhysicsBodyType::RIGID_BODY;

	/// world transform  initilize
	mWorldTransform = getTransform();

    /// Compute the inverse mass
    mMassInverse = (mInitMass > scalar(0))? scalar(1.0) / mInitMass : scalar(0);

    /// Compute the  energy to body
   // mTotalEnergy = pow(LIGHT_MAX_VELOCITY_C * mInitMass , 2.0);
}




///********************************************************
/// Information is taken from the book : http://www.gptelecom.ru/Articles/tensor.pdf
///********************************************************/
SIMD_INLINE void rpPhysicsRigidBody::Integrate( scalar _dt , ObserverSystem _observer )
{




    /// Time step integration
     mStepTime = _dt;



    if ( mInitMass == scalar(0.f) || _dt == scalar(0.f))
    {
        mExternalForce.setToZero();
        mExternalTorque.setToZero();

        mLinearVelocity.setToZero();
        mAngularVelocity.setToZero();

        return;
    }


    // If it is a static body
    if (mType == STATIC || mType == KINEMATIC)
    {
        // Reset the velocity to zero
        mLinearVelocity.setToZero();
        mAngularVelocity.setToZero();

    }




    /*********************************************
     *          Damping  velocity
     ********************************************/

    Damping( mLinearVelocity  , MINIMUM_FOR_DAPING , mLinearDamping  );
    Damping( mAngularVelocity , MINIMUM_FOR_DAPING , mAngularDamping );



    /*********************************************
     *          Integration forces
     ********************************************/

    scalar demission_linear  = ( 1.0 + Abs((mExternalForce  * _dt).dot(mLinearVelocity  * _dt)) / (LIGHT_MAX_VELOCITY_C * LIGHT_MAX_VELOCITY_C));
    scalar demission_angular = ( 1.0 + Abs((mExternalTorque * _dt).dot(mAngularVelocity * _dt)) / (LIGHT_MAX_VELOCITY_C * LIGHT_MAX_VELOCITY_C));

    mLinearVelocity  = (mLinearVelocity  + mExternalForce  * _dt) / demission_linear;
    mAngularVelocity = (mAngularVelocity + mExternalTorque * _dt) / demission_angular;




    /**********************************************
     *         Integrate lorentz evolution
     **********************************************/

     /// Translation Object
     Transform resulTransform = mWorldTransform;

     /// Loretz integration
     resulTransform = TransformUtil::RelativityIntegrateTransform( _observer ,  resulTransform , mLinearVelocity  , mAngularVelocity   , _dt  );

     /// Euler  integration
     resulTransform = TransformUtil::integrateTransform( resulTransform , mSplitLinearVelocity ,  mSplitAngularVelocity , _dt  );

    /// Lorentz boost matrix demission world position (distance demission  K => `K)
  //  mTransform.setPosition(mWorldTransform.getScale() * mTransform.getPosition());


    ///Update transformation
    setWorldTransform(resulTransform);
    UpdateMatrices();


    //-----------------------------------------------------------------------
    // Reset forces
    //-----------------------------------------------------------------------
    mExternalForce.setToZero();
    mExternalTorque.setToZero();

    mSplitLinearVelocity.setToZero();
    mSplitAngularVelocity.setToZero();

}



/// To remove a collision shape, you need to specify the pointer to the proxy
/// shape that has been returned when you have added the collision shape to the
/// body
/**
 * @param proxyShape The pointer of the proxy shape you want to remove
 */
SIMD_INLINE void rpPhysicsRigidBody::removeCollisionShape(const rpProxyShape *proxyShape)
{
    // Remove the collision shape
    rpCollisionBody::removeCollisionShapee(proxyShape);

    // Recompute the total mass, center of mass and inertia tensor
    recomputeMassInformation();
}


SIMD_INLINE void real_physics::rpPhysicsRigidBody::applyGravity(const Vector3& gravity)
{
	if( mMassInverse > 0 && !mIsSleeping && getType() == BodyType::DYNAMIC)
	{
		applyImpulseLinear(gravity * mInitMass);
    }
}



SIMD_INLINE void rpPhysicsRigidBody::updateBroadPhaseState() const
{
    rpPhysicsObject::updateBroadPhaseStatee( mStepTime * mLinearVelocity );
}





SIMD_INLINE void rpPhysicsRigidBody::setType(BodyType type)
{

	//if (mType == type) return;
	rpCollisionBody::setType(type);

	mType = type;

	// Recompute the total mass, center of mass and inertia tensor
	recomputeMassInformation();

	// If it is a static body
	if (mType == STATIC)
	{
		// Reset the velocity to zero
		mLinearVelocity.setToZero();
		mAngularVelocity.setToZero();
	}

	// If it is a static or a kinematic body
	if (mType == STATIC || mType == KINEMATIC)
	{
		// Reset the inverse mass and inverse inertia tensor to zero
		mMassInverse = scalar(0.0);
		mInertiaTensorLocal.setToZero();
		mInertiaTensorLocalInverse.setToZero();

	}
	else
	{  // If it is a dynamic body
		mMassInverse = scalar(1.0) / mInitMass;
		mInertiaTensorLocalInverse = mInertiaTensorLocal.getInverse();
	}

	// Awake the body
	setIsSleeping(false);


	UpdateMatrices();

	// Remove all the contacts with this body
    resetContactManifoldsList();

	// Ask the broad-phase to test again the collision shapes of the body for collision
	// detection (as if the body has moved)
    askForBroadPhaseCollisionCheck();

	// Reset the force and torque on the body
	mExternalForce.setToZero();
	mExternalTorque.setToZero();



}

SIMD_INLINE void rpPhysicsRigidBody::setIsSleeping(bool isSleeping)
{
    if (isSleeping)
    {
        // Absolutely Stop motion
        mLinearVelocity.setToZero();
        mAngularVelocity.setToZero();
        mExternalForce.setToZero();
        mExternalTorque.setToZero();
        mSplitLinearVelocity.setToZero();
        mSplitLinearVelocity.setToZero();
    }

    rpBody::setIsSleeping(isSleeping);
}



SIMD_INLINE void rpPhysicsRigidBody::recomputeMassInformation()
{
	mInitMass = scalar(0.0);
	mMassInverse = scalar(0.0);
	mInertiaTensorLocal.setToZero();
	mInertiaTensorLocalInverse.setToZero();
	mCenterOfMassLocal.setToZero();

	// If it is STATIC or KINEMATIC body
	if (mType == STATIC || mType == KINEMATIC)
	{
        mCenterOfMassWorld = mTransform.getPosition4().getPos();
		return;
	}

	assert(mType == DYNAMIC);

	// Compute the total mass of the body
	for (rpProxyShape* shape = mProxyCollisionShapes; shape != NULL; shape = shape->mNext)
	{
		mInitMass += shape->getMass();
        mCenterOfMassLocal += shape->getLocalToBodyTransform().getPosition4().getPos() * shape->getMass();
	}

	if (mInitMass > scalar(0.0))
	{
		mMassInverse = scalar(1.0) / mInitMass;
	}
	else
	{
		mInitMass = scalar(1.0);
		mMassInverse = scalar(1.0);
	}

	// Compute the center of mass
	const Vector3 oldCenterOfMass = mCenterOfMassWorld;
	mCenterOfMassLocal *= mMassInverse;
	mCenterOfMassWorld  =  mTransform * mCenterOfMassLocal;

	// Compute the total mass and inertia tensor using all the collision shapes
	for (rpProxyShape* shape = mProxyCollisionShapes; shape != NULL; shape = shape->mNext)
	{
		// Get the inertia tensor of the collision shape in its local-space
		Matrix3x3 inertiaTensor;
		shape->getCollisionShape()->computeLocalInertiaTensor(inertiaTensor, shape->getMass());

		// Convert the collision shape inertia tensor into the local-space of the body
        const Transform& shapeTransform = shape->getLocalToBodyTransform();
		Matrix3x3 rotationMatrix = shapeTransform.getBasis();
		inertiaTensor = rotationMatrix * inertiaTensor * rotationMatrix.getTranspose();

		// Use the parallel axis theorem to convert the inertia tensor w.r.t the collision shape
		// center into a inertia tensor w.r.t to the body origin.
        Vector3 offset = shapeTransform.getPosition4().getPos() - mCenterOfMassLocal;
		scalar offsetSquare = offset.lengthSquare();
		Matrix3x3 offsetMatrix;
		offsetMatrix[0].setAllValues(offsetSquare, scalar(0.0), scalar(0.0));
		offsetMatrix[1].setAllValues(scalar(0.0), offsetSquare, scalar(0.0));
		offsetMatrix[2].setAllValues(scalar(0.0), scalar(0.0), offsetSquare);
		offsetMatrix[0] += offset * (-offset.x);
		offsetMatrix[1] += offset * (-offset.y);
		offsetMatrix[2] += offset * (-offset.z);
		offsetMatrix *= shape->getMass();

		mInertiaTensorLocal += inertiaTensor + offsetMatrix;
	}

	// Compute the local inverse inertia tensor
	mInertiaTensorLocalInverse = mInertiaTensorLocal.getInverse();

	// Update the linear velocity of the center of mass
	mLinearVelocity += mAngularVelocity.cross(mCenterOfMassWorld - oldCenterOfMass);
}




SIMD_INLINE void rpPhysicsRigidBody::UpdateMatrices()
{
    mInertiaTensorWorldInverse = mTransform.getOrientation().getMatrix() * getInertiaTensorInverseLocal() *
                                 mTransform.getOrientation().getMatrix().getTranspose();

	mCenterOfMassWorld = mTransform * mCenterOfMassLocal;
}





//************************** apply Impulse ***********************//

SIMD_INLINE void rpPhysicsRigidBody::applySplitImpulse(const Vector3& impuls, const Vector3& point )
{
	// If it is not a dynamic body, we do nothing
	if (mType != DYNAMIC) return;

	// Awake the body if it was sleeping
    if (!mIsSleeping)
	{
        applySplitImpulseLinear(impuls);
        applySplitImpulseAngular((point - mCenterOfMassWorld).cross(impuls));
	}
}


//************************** apply Impulse ***********************//
SIMD_INLINE void rpPhysicsRigidBody::applySplitImpulseAngular(const Vector3&  impuls )
{
	// If it is not a dynamic body, we do nothing
	if (mType != DYNAMIC) return;

	// Awake the body if it was sleeping
    if (!mIsSleeping)
	{
        mSplitAngularVelocity += getInertiaTensorInverseWorld() * (impuls);
	}
}


//************************** apply Impulse ***********************//
SIMD_INLINE  void rpPhysicsRigidBody::applySplitImpulseLinear(const Vector3& impuls)
{
	// If it is not a dynamic body, we do nothing
	if (mType != DYNAMIC) return;

	// Awake the body if it was sleeping
    if (!mIsSleeping)
	{
        mSplitLinearVelocity += getInverseMass() * (impuls);
	}
}



/**************************************************************/

//************************** apply Impulse ***********************//
SIMD_INLINE void rpPhysicsRigidBody::applyImpulse(const Vector3& impuls , const Vector3& point)
{
	// If it is not a dynamic body, we do nothing
	if (mType != DYNAMIC) return;

	// Awake the body if it was sleeping
    if (!mIsSleeping)
	{

        //scalar gamma = gammaInvertFunction( mLinearVelocity - mAngularVelocity.cross(point - mCenterOfMassWorld));

        applyImpulseLinear( impuls );
        applyImpulseAngular((point - mCenterOfMassWorld).cross(impuls) );

	}
}


//************************** apply Impulse ***********************//
SIMD_INLINE void rpPhysicsRigidBody::applyImpulseAngular(const Vector3&  impuls )
{
	// If it is not a dynamic body, we do nothing
	if (mType != DYNAMIC) return;

	// Awake the body if it was sleeping
    if (!mIsSleeping)
	{
         Vector3 inv_impuls = getInertiaTensorInverseWorld() * (impuls);

         mAngularVelocity = (mAngularVelocity + inv_impuls) / ((1.0 + Abs((inv_impuls).dot(mAngularVelocity)) / (LIGHT_MAX_VELOCITY_C*LIGHT_MAX_VELOCITY_C)));
	}
}


//************************** apply Impulse ***********************//
SIMD_INLINE  void rpPhysicsRigidBody::applyImpulseLinear( const Vector3& impuls )
{

	// If it is not a dynamic body, we do nothing
	if (mType != DYNAMIC) return;

	// Awake the body if it was sleeping
    if (!mIsSleeping)
    {

        Vector3 inv_impuls = getInverseMass() * impuls;

        mLinearVelocity = (mLinearVelocity + inv_impuls) / ((1.0 + Abs((inv_impuls).dot(mLinearVelocity)) / (LIGHT_MAX_VELOCITY_C*LIGHT_MAX_VELOCITY_C)) );
	}
}


SIMD_INLINE void rpPhysicsRigidBody::applyForce(const Vector3& force, const Vector3& point)
{
	// If it is not a dynamic body, we do nothing
	if (mType != DYNAMIC) return;

	// Awake the body if it was sleeping
    if (!mIsSleeping)
	{
        // Add the force and torque
        applyForceToCenterOfMass(force);
        applyTorque((point - mCenterOfMassWorld).cross(force));
	}
}


SIMD_INLINE void rpPhysicsRigidBody::applyTorque(const Vector3& torque)
{
	// If it is not a dynamic body, we do nothing
	if (mType != DYNAMIC) return;

	// Awake the body if it was sleeping
    if (!mIsSleeping)
	{
        // Add the torque
        mExternalTorque += getInertiaTensorInverseWorld() * torque;
	}
}


SIMD_INLINE void rpPhysicsRigidBody::applyForceToCenterOfMass(const Vector3& force)
{
	// If it is not a dynamic body, we do nothing
	if (mType != DYNAMIC) return;

    // Awake the body if it was sleeping
    if (!mIsSleeping)
	{
        // Add the force
        mExternalForce += getInverseMass() * force;
	}


}

void rpPhysicsRigidBody::setLinearVelocity(const Vector3 &linearVelocity)
{
    mLinearVelocity = linearVelocity;
}

void rpPhysicsRigidBody::setAngularVelocity(const Vector3 &angularVelocity)
{
    mAngularVelocity = angularVelocity;
}


} /* namespace real_physics */


