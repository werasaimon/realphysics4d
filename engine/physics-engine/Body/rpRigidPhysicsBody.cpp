/*
 * rpRigidPhysicsBody.cpp
 *
 *  Created on: 14 дек. 2016 г.
 *      Author: wera
 */

#include "rpRigidPhysicsBody.h"
#include "rpPhysicsObject.h"

#include <stddef.h>
#include <cassert>
#include <iostream>

#include "../Collision/rpProxyShape.h"
#include "../Collision/Shapes/rpCollisionShape.h"
#include "../LinearMaths/mathematics.h"
#include "../LinearMaths/rpLinearMtah.h"
#include "../config.h"





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


rpRigidPhysicsBody::rpRigidPhysicsBody(const Transform& transform, rpCollisionManager *CollideWorld, bodyindex id)
:rpPhysicsBody(transform, CollideWorld, id),
 mInitMass(scalar(1.0)),
 mCenterOfMassLocal(0, 0, 0),
 mCenterOfMassWorld(transform.getPosition()),
 mIsGravityEnabled(true),
 mLinearDamping(scalar(0.004)),
 mAngularDamping(scalar(0.004)),
 mLinearVelocity(Vector3::ZERO),
 mAngularVelocity(Vector3::ZERO),
 mSplitLinearVelocity(Vector3::ZERO),
 mSplitAngularVelocity(Vector3::ZERO),
 mLinearFourVelocity4(0,0,0,0),
 mAngularFourVelocity4(0,0,0,0),
 mFourForce4(0,0,0,0),
 mFourTorque4(0,0,0,0),
 mFourPosition4(transform.getPosition(),0) ,
 mStepTime(0.0)
{
	/// body To type
	mTypePhysics = PhysicsBodyType::RIGID_BODY;

	/// world transform  initilize
	mWorldTransform = getTransform();

    /// Compute the inverse mass
    mMassInverse = (mInitMass > scalar(0))? scalar(1.0) / mInitMass : scalar(0);

    /// Compute the  energy to body
    mTotalEnergy = pow(LIGHT_MAX_VELOCITY_C * mInitMass , 2.0);
}




///********************************************************
/// Information is taken from the book : http://www.gptelecom.ru/Articles/tensor.pdf
///********************************************************/
SIMD_INLINE void rpRigidPhysicsBody::Integrate(scalar _dt)
{
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



        scalar gamma       =       gammaFunction(mLinearVelocity) * gammaFunction(mAngularVelocity);
        scalar gammaInvert = gammaInvertFunction(mLinearVelocity) * gammaInvertFunction(mAngularVelocity);





	    /*********************************************
	     *          Integration forces
	     ********************************************/
        /**
        scalar E = ( mTotalEnergy / _dt)  / (LIGHT_MAX_VELOCITY_C * LIGHT_MAX_VELOCITY_C) * gammaInvert;
        mFourForce4  = MinkowskiVector4(  mExternalForce  / (LIGHT_MAX_VELOCITY_C * gammaInvert) , E);
        mFourTorque4 = MinkowskiVector4(  mExternalTorque / (LIGHT_MAX_VELOCITY_C * gammaInvert) , E);

        mExternalForce  = (LIGHT_MAX_VELOCITY_C * mFourForce4.getVector3());
        mExternalTorque = (LIGHT_MAX_VELOCITY_C * mFourTorque4.getVector3());

        mLinearVelocity  += mExternalForce  * _dt;
        mAngularVelocity += mExternalTorque * _dt;
        /**/


        mTotalEnergy  = Pow(mInitMass * LIGHT_MAX_VELOCITY_C , scalar(2.0)) / gamma;

        scalar E = ( mTotalEnergy / _dt)  / (LIGHT_MAX_VELOCITY_C * LIGHT_MAX_VELOCITY_C) * gammaInvert;
        mFourForce4  = MinkowskiVector4(  mExternalForce  * gammaInvert , E);
        mFourTorque4 = MinkowskiVector4(  mExternalTorque * gammaInvert , E);


        mExternalForce  = (mFourForce4.getVector3());
        mExternalTorque = (mFourTorque4.getVector3());


        mLinearVelocity  += mExternalForce  * _dt;
        mAngularVelocity += mExternalTorque * _dt;



	    /*********************************************
	     *          Damping  velocity
         ********************************************/
        /**
	    if( mLinearVelocity.length2() < MINIMUM_FOR_DAPING )
	    {

            if( mLinearVelocity.length() > mLinearDamping )
	    	{
	    		mLinearVelocity -= ( mLinearVelocity.getUnit() * mLinearDamping);
	    	}
	    	else
	    	{
	    		mLinearVelocity = Vector3::ZERO;
	    	}

	    }


	    if( mAngularVelocity.length2()  < MINIMUM_FOR_DAPING )
	    {

            if( mAngularVelocity.length() > mAngularDamping )
	    	{
	    		mAngularVelocity -= ( mAngularVelocity.getUnit() * mAngularDamping);
	    	}
	    	else
	    	{
	    		mAngularVelocity  = Vector3::ZERO;
	    	}

        }
        /**/


        Damping( mLinearVelocity  , MINIMUM_FOR_DAPING , mLinearDamping  );
        Damping( mAngularVelocity , MINIMUM_FOR_DAPING , mAngularDamping );




	    /**********************************************
         *         Integrate lorentz evolution
         **********************************************/


        /**
	    ///Four-velocity on relativity 4D-space
        mLinearFourVelocity4  =  MinkowskiVector4( mLinearVelocity  / (LIGHT_MAX_VELOCITY_C * gammaInvert)  , gamma );
	    mAngularFourVelocity4 =  MinkowskiVector4( mAngularVelocity / (LIGHT_MAX_VELOCITY_C * gammaInvert)  , gamma );



        MinkowskiVector4 Vel_Test =  MinkowskiVector4( mLinearVelocity * (gamma)  ,  (LIGHT_MAX_VELOCITY_C * gammaInvert ) );

        cout<< "Vel :  " << Vel_Test.length() << "  Real: " << mLinearFourVelocity4.length() <<endl;

        /**/


        gamma       =       gammaFunction(mLinearVelocity) * gammaFunction(mAngularVelocity);
        gammaInvert = gammaInvertFunction(mLinearVelocity) * gammaInvertFunction(mAngularVelocity);



        ///Four-velocity on relativity 4D-space
        mLinearFourVelocity4  =  MinkowskiVector4( mLinearVelocity  * gammaInvert  , (LIGHT_MAX_VELOCITY_C * gammaInvert) );
        mAngularFourVelocity4 =  MinkowskiVector4( mAngularVelocity * gammaInvert  , (LIGHT_MAX_VELOCITY_C * gammaInvert) );


	    ///Real-velocity-dynamic on dimension of 3D-space
        mLinearVelocity  = (mLinearFourVelocity4.getVector3());
        mAngularVelocity = (mAngularFourVelocity4.getVector3());


	    /// Lorentz boost matrix for linear velocity
	    mRelativityMotion.updateDisplacementBoost(mLinearVelocity);


        ///Translation move Objects
        Transform resulTransform = TransformUtil::integrateTransform(mWorldTransform , mLinearVelocity      ,  mAngularVelocity      , _dt  );
                  resulTransform = TransformUtil::integrateTransform(resulTransform  , mSplitLinearVelocity ,  mSplitAngularVelocity , _dt  );


        /// Lorentz transllate position
        mFourPosition4 += mLinearFourVelocity4 * _dt;
        mFourPosition4 += MinkowskiVector4(mSplitLinearVelocity,0) * _dt;


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


///*****************************///
/// Change the frame of reference
/// to the new in the body one
/// ****************************///
void rpRigidPhysicsBody::changeToFrameOfReference( rpRigidPhysicsBody *rigidBody )
{


    /**
	Vector3 VL = (this->mLinearVelocity  - rigidBody->mLinearVelocity);
	Vector3 VA = (this->mAngularVelocity - rigidBody->mAngularVelocity);
    Vector3 R =  (this->mFourPosition4.getVector3() - rigidBody->mFourPosition4.getVector3());

	scalar gammaInvert = gammaInvertFunction(VA) * gammaInvertFunction(VL);

	MinkowskiVector4 ULinear  = mLinearFourVelocity4;
	MinkowskiVector4 UAngular = mAngularFourVelocity4;


    /**
	ULinear.setVector3( (ULinear.getVector3() - ((VL/LIGHT_MAX_VELOCITY_C) * ULinear.t)) / gammaInvert  );
	ULinear.t = (ULinear.t - (VL/LIGHT_MAX_VELOCITY_C).dot(R)) / gammaInvert ;

	UAngular.setVector3( (UAngular.getVector3() - ((VA/LIGHT_MAX_VELOCITY_C) * UAngular.t)) / gammaInvert  );
	UAngular.t = (UAngular.t - (VA/LIGHT_MAX_VELOCITY_C).dot(R)) / gammaInvert ;


	//mLinearFourVelocity4  = ULinear;
	//mAngularFourVelocity4 = UAngular;
	/**/



    /**
	MinkowskiVector4 position = mFourPosition4;
	position.setVector3( position.getVector3() - ((VL / LIGHT_MAX_VELOCITY_C) * position.t) / gammaInvert  );
	position.t = (position.t - (VL / LIGHT_MAX_VELOCITY_C).dot(R)) / gammaInvert ;

	//mWorldTransform.setPosition(position.getVector3());
	/**/



	//cout << "linear4: " << ULinear.lengthSquare() <<endl;
	//cout << " 4A-angular: " <<  UAngular.length() <<endl;

}


/// To remove a collision shape, you need to specify the pointer to the proxy
/// shape that has been returned when you have added the collision shape to the
/// body
/**
 * @param proxyShape The pointer of the proxy shape you want to remove
 */
SIMD_INLINE void rpRigidPhysicsBody::removeCollisionShape(const rpProxyShape *proxyShape)
{
    // Remove the collision shape
    rpCollisionBody::removeCollisionShapee(proxyShape);

    // Recompute the total mass, center of mass and inertia tensor
    recomputeMassInformation();
}


SIMD_INLINE void real_physics::rpRigidPhysicsBody::applyGravity(const Vector3& gravity)
{
	if( mMassInverse > 0 && !mIsSleeping && getType() == BodyType::DYNAMIC)
	{
		applyImpulseLinear(gravity * mInitMass);
    }
}



SIMD_INLINE void rpRigidPhysicsBody::updateBroadPhaseState() const
{
    rpPhysicsObject::updateBroadPhaseStatee( mStepTime * mLinearVelocity );
}





SIMD_INLINE void rpRigidPhysicsBody::setType(BodyType type)
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

SIMD_INLINE void rpRigidPhysicsBody::setIsSleeping(bool isSleeping)
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



SIMD_INLINE void rpRigidPhysicsBody::recomputeMassInformation()
{
	mInitMass = scalar(0.0);
	mMassInverse = scalar(0.0);
	mInertiaTensorLocal.setToZero();
	mInertiaTensorLocalInverse.setToZero();
	mCenterOfMassLocal.setToZero();

	// If it is STATIC or KINEMATIC body
	if (mType == STATIC || mType == KINEMATIC)
	{
		mCenterOfMassWorld = mTransform.getPosition();
		return;
	}

	assert(mType == DYNAMIC);

	// Compute the total mass of the body
	for (rpProxyShape* shape = mProxyCollisionShapes; shape != NULL; shape = shape->mNext)
	{

		mInitMass += shape->getMass();
		mCenterOfMassLocal += shape->getLocalToBodyTransform().getPosition() * shape->getMass();
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
		Vector3 offset = shapeTransform.getPosition() - mCenterOfMassLocal;
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




SIMD_INLINE void rpRigidPhysicsBody::UpdateMatrices()
{
    mInertiaTensorWorldInverse = mTransform.getBasis() * mInertiaTensorLocalInverse * mRelativityMotion.getLorentzMatrix() *
			                     mTransform.getBasis().getTranspose();

	mCenterOfMassWorld = mTransform * mCenterOfMassLocal;
}





//************************** apply Impulse ***********************//

SIMD_INLINE void rpRigidPhysicsBody::applySplitImpulse(const Vector3& impuls, const Vector3& point )
{
	// If it is not a dynamic body, we do nothing
	if (mType != DYNAMIC) return;

	// Awake the body if it was sleeping
	if (mIsSleeping)
	{
        /*setIsSleeping(false);*/ return;
	}

	applySplitImpulseLinear(impuls);
	applySplitImpulseAngular((point - mCenterOfMassWorld).cross(impuls));
}


//************************** apply Impulse ***********************//
SIMD_INLINE void rpRigidPhysicsBody::applySplitImpulseAngular(const Vector3&  impuls )
{
	// If it is not a dynamic body, we do nothing
	if (mType != DYNAMIC) return;

	// Awake the body if it was sleeping
	if (mIsSleeping)
	{
        /*setIsSleeping(false);*/ return;
	}

	mSplitAngularVelocity += getInertiaTensorInverseWorld() * (impuls);
}


//************************** apply Impulse ***********************//
SIMD_INLINE  void real_physics::rpRigidPhysicsBody::applySplitImpulseLinear(const Vector3& impuls)
{
	// If it is not a dynamic body, we do nothing
	if (mType != DYNAMIC) return;

	// Awake the body if it was sleeping
	if (mIsSleeping)
	{
        /*setIsSleeping(false);*/ return;
	}

	mSplitLinearVelocity += getInverseMass() * (impuls);
}



/**************************************************************/

//************************** apply Impulse ***********************//
SIMD_INLINE void rpRigidPhysicsBody::applyImpulse(const Vector3& impuls , const Vector3& point)
{
	// If it is not a dynamic body, we do nothing
	if (mType != DYNAMIC) return;

	// Awake the body if it was sleeping
	if (mIsSleeping)
	{
        /*setIsSleeping(false);*/ return;
	}

	applyImpulseLinear(impuls);
	applyImpulseAngular((point - mCenterOfMassWorld).cross(impuls));

}


//************************** apply Impulse ***********************//
SIMD_INLINE void rpRigidPhysicsBody::applyImpulseAngular(const Vector3&  impuls )
{
	// If it is not a dynamic body, we do nothing
	if (mType != DYNAMIC) return;

	// Awake the body if it was sleeping
	if (mIsSleeping)
	{
        /*setIsSleeping(false);*/ return;
	}

	mAngularVelocity += getInertiaTensorInverseWorld() * (impuls);// * gammaInvertFunction(mAngularVelocity);

}


//************************** apply Impulse ***********************//
SIMD_INLINE  void rpRigidPhysicsBody::applyImpulseLinear( const Vector3& impuls )
{

	// If it is not a dynamic body, we do nothing
	if (mType != DYNAMIC) return;

	// Awake the body if it was sleeping
	if (mIsSleeping)
	{
        /*setIsSleeping(false);*/ return;
	}

	mLinearVelocity += getInverseMass() * (impuls);// * gammaInvertFunction(mLinearVelocity);

}


SIMD_INLINE void rpRigidPhysicsBody::applyForce(const Vector3& force, const Vector3& point)
{
	// If it is not a dynamic body, we do nothing
	if (mType != DYNAMIC) return;

	// Awake the body if it was sleeping
	if (mIsSleeping)
	{
        /*setIsSleeping(false);*/ return;
	}

	// Add the force and torque
	applyForceToCenterOfMass(force);
	applyTorque((point - mCenterOfMassWorld).cross(force));

}


SIMD_INLINE void rpRigidPhysicsBody::applyTorque(const Vector3& torque)
{
	// If it is not a dynamic body, we do nothing
	if (mType != DYNAMIC) return;

	// Awake the body if it was sleeping
	if (mIsSleeping)
	{
        /*setIsSleeping(false);*/ return;
	}

	// Add the torque
	mExternalTorque += getInertiaTensorInverseWorld() * torque;
}


SIMD_INLINE void rpRigidPhysicsBody::applyForceToCenterOfMass(const Vector3& force)
{
	// If it is not a dynamic body, we do nothing
	if (mType != DYNAMIC) return;

	// Awake the body if it was sleeping
	if (mIsSleeping)
	{
        /*setIsSleeping(false);*/ return;
	}

	// Add the force
	mExternalForce += getInverseMass() * force;
}

void rpRigidPhysicsBody::setLinearVelocity(const Vector3 &linearVelocity)
{
    mLinearVelocity = linearVelocity;
}

void rpRigidPhysicsBody::setAngularVelocity(const Vector3 &angularVelocity)
{
    mAngularVelocity = angularVelocity;
}


} /* namespace real_physics */


