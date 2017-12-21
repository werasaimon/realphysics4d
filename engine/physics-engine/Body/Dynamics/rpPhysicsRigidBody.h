/*
 * rpRigidPhysicsBody.h
 *
 *  Created on: 14 дек. 2016 г.
 *      Author: wera
 */

#ifndef SOURCE_ENGIE_KINEMATICPHYSICS_BODY_RPRIGIDPHYSICSBODY_H_
#define SOURCE_ENGIE_KINEMATICPHYSICS_BODY_RPRIGIDPHYSICSBODY_H_

#include "../Material/rpPhysicsMaterial.h"
#include "rpPhysicsBody.h"

namespace real_physics
{




//**********************************************************************************************//

class rpPhysicsRigidBody: public rpPhysicsBody , public BlockAlloc<rpPhysicsRigidBody>
{


	private:


        scalar mStepTime;



        //-------------------- Attributes --------------------//

		/// Material properties of the rigid body
		rpPhysicsMaterial mMaterial;


		/// Intial mass of the body
		scalar mInitMass;





        /// Linear velocity damping factor
        scalar mLinearDamping;

        /// Angular velocity damping factor
        scalar mAngularDamping;




		/// Linear velocity of the body
		Vector3 mLinearVelocity;

		/// Angular velocity of the body
		Vector3 mAngularVelocity;




        /// Current external force on the body
        Vector3 mExternalForce;

        /// Current external torque on the body
        Vector3 mExternalTorque;




		/// Linear Split velocity  of the body
		Vector3 mSplitLinearVelocity;

		/// Angular Split velocity  of the body
		Vector3 mSplitAngularVelocity;




		/// Center of mass of the body in local-space coordinates.
		/// The center of mass can therefore be different from the body origin
		Vector3 mCenterOfMassLocal;

		/// Center of mass of the body in world-space coordinates
		Vector3 mCenterOfMassWorld;




		/// Local inertia tensor of the body (in local-space) with respect to the
		/// center of mass of the body
		Matrix3x3 mInertiaTensorLocal;

        /// Inverse of the inertia tensor of the body
		Matrix3x3 mInertiaTensorLocalInverse;

        /// Inertia tensor of the body
		Matrix3x3 mInertiaTensorWorldInverse;

		/// Inverse of the mass of the body
		scalar    mMassInverse;




		/// True if the gravity needs to be applied to this rigid body
		bool mIsGravityEnabled;




		/// apply pseudo impulse
		virtual void applySplitImpulse(const Vector3& impuls, const Vector3& point );
		virtual void applySplitImpulseAngular(const Vector3&  impuls );
		virtual void applySplitImpulseLinear(const Vector3&  impuls );


	public:


        rpPhysicsRigidBody(const Transform& transform, rpCollisionManager *CollideWorld, bodyindex id );



		/// apply force
		virtual void applyForce(const Vector3& Pivot, const Vector3& Impuls);
		virtual void applyTorque(const Vector3& Torque);
		virtual void applyForceToCenterOfMass(const Vector3& force);




		/// apply impulse
		virtual void applyImpulse(const Vector3& impuls , const Vector3& point);
	    virtual void applyImpulseAngular(const Vector3&  impuls );
		virtual void applyImpulseLinear(const Vector3&  impuls );

	    /************* methods ********************/

		/// Set the type of the body (static, kinematic or dynamic)
		void setType(BodyType type);


		/// Set the variable to know whether or not the body is sleeping
		void setIsSleeping(bool isSleeping);


        //// Integrate to Minkowskii method
        void Integrate(scalar _dt , ObserverSystem _observer);


        /// Remove a collision shape from the body
        void removeCollisionShape(const rpProxyShape* proxyShape );


		/// Recompute the center of mass, total mass and inertia tensor of the body using all
		/// the collision shapes attached to the body.
		void recomputeMassInformation();


		/// Update  matrices matrix invert inertia and position
        void UpdateMatrices();


		/// Update the transform of the body after a change of the center of mass
		void updateTransformWithCenterOfMass();


		/// Apply integrate velocity of gravity
	    void applyGravity( const Vector3& gravity );

        /*******************************************************/

        /// Update the broad-phase state for this body (because it has moved for instance)
        void updateBroadPhaseState() const;

		/*******************************************************/



	    /// Material get to Body
	    const rpPhysicsMaterial& getMaterial() const;

	    /// Material set to Body
	    void setMaterial(const rpPhysicsMaterial& material);


		/// Inertia to of mass
	    scalar getMass() const;
	    scalar getInverseMass() const;


	    /// Inertia tensor to Body
	    Matrix3x3  getInertiaTensorLocal() const;
	    Matrix3x3  getInertiaTensorWorld() const;
	    Matrix3x3  getInertiaTensorInverseWorld() const;


	    /// Velocity to body
	    Vector3 getAngularVelocity() const;
	    Vector3 getLinearVelocity() const;


	    /// Damping for to velocity
	    scalar getAngularDamping() const;
	    scalar getLinearDamping()  const;


        //---------------------  Set --------------------------- //


        void setLinearVelocity(const Vector3 &linearVelocity);
        void setAngularVelocity(const Vector3 &angularVelocity);


		// -------------------- Friendships -------------------- //
		friend class rpDynamicsWorld;
        friend class rpContactSolverSequentialImpulseObject;


		friend class rpDistanceJoint;
		friend class rpFixedJoint;
		friend class rpBallAndSocketJoint;
		friend class rpHingeJoint;
		friend class rpSliderJoint;
        friend class rpAngleAxisJoint;


};




SIMD_INLINE const rpPhysicsMaterial& rpPhysicsRigidBody::getMaterial() const
{
	return mMaterial;
}

SIMD_INLINE void rpPhysicsRigidBody::setMaterial(const rpPhysicsMaterial& material)
{
	mMaterial = material;
}


SIMD_INLINE scalar rpPhysicsRigidBody::getMass() const
{
	return mInitMass;
}

SIMD_INLINE scalar rpPhysicsRigidBody::getInverseMass() const
{
	return mMassInverse;
}


SIMD_INLINE Vector3 rpPhysicsRigidBody::getAngularVelocity() const
{
	return mAngularVelocity;
}

SIMD_INLINE Vector3 rpPhysicsRigidBody::getLinearVelocity() const
{
	return mLinearVelocity;
}


/// Update the transform of the body after a change of the center of mass
SIMD_INLINE void rpPhysicsRigidBody::updateTransformWithCenterOfMass()
{
	// Translate the body according to the translation of the center of mass position
	mTransform.setPosition(mCenterOfMassWorld - mTransform.getOrientation() * mCenterOfMassLocal);
}


// Return the local inertia tensor of the body (in local-space coordinates)
/**
 * @return The 3x3 inertia tensor matrix of the body (in local-space coordinates)
 */
SIMD_INLINE  Matrix3x3 rpPhysicsRigidBody::getInertiaTensorLocal() const
{
	return mInertiaTensorLocal;
}


// Return the inertia tensor in world coordinates.
/// The inertia tensor I_w in world coordinates is computed
/// with the local inertia tensor I_b in body coordinates
/// by I_w = R * I_b * R^T
/// where R is the rotation matrix (and R^T its transpose) of
/// the current orientation quaternion of the body
/**
 * @return The 3x3 inertia tensor matrix of the body in world-space coordinates
 */
SIMD_INLINE Matrix3x3 rpPhysicsRigidBody::getInertiaTensorWorld() const
{
	  // Compute and return the inertia tensor in world coordinates
	    return mTransform.getOrientation().getMatrix() * mInertiaTensorLocal *
	           mTransform.getOrientation().getMatrix().getTranspose();
}




SIMD_INLINE Matrix3x3 rpPhysicsRigidBody::getInertiaTensorInverseWorld() const
{
    // TODO : DO NOT RECOMPUTE THE MATRIX MULTIPLICATION EVERY TIME. WE NEED TO STORE THE
    //        INVERSE WORLD TENSOR IN THE CLASS AND UPLDATE IT WHEN THE ORIENTATION OF THE BODY CHANGES

    // Compute and return the inertia tensor in world coordinates
    return (mTransform.getOrientation().getMatrix() * mInertiaTensorLocalInverse *
    	    mTransform.getOrientation().getMatrix().getTranspose());
}


SIMD_INLINE scalar rpPhysicsRigidBody::getAngularDamping() const
{
	return mAngularDamping;
}

SIMD_INLINE scalar rpPhysicsRigidBody::getLinearDamping() const
{
	return mLinearDamping;
}




} /* namespace real_physics */

#endif /* SOURCE_ENGIE_KINEMATICPHYSICS_BODY_RPRIGIDPHYSICSBODY_H_ */
