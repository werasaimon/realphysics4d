/*
 * rpRigidPhysicsBody.h
 *
 *  Created on: 14 дек. 2016 г.
 *      Author: wera
 */

#ifndef SOURCE_ENGIE_DYNAMICS_BODY_RPPHYSICSBODY_H_
#define SOURCE_ENGIE_DYNAMICS_BODY_RPPHYSICSBODY_H_

#include "../../Dynamics/Body/rpPhysicsObject.h"

namespace real_physics
{


enum PhysicsBodyType { RIGID_BODY ,  VELERT_BODY };

class rpPhysicsBody: public rpPhysicsObject
{

   protected:

	PhysicsBodyType mTypePhysics;

   public:

             rpPhysicsBody( const Transform& transform, rpCollisionDetection *CollideWorld, bodyindex id );
	virtual ~rpPhysicsBody();


    /// integrate to Euler method
    virtual void Integrate(scalar _dt) {}

    /// earth gravity force = (9.8) for force gravity momentum
    virtual void applyGravity( const Vector3& gravity ) {}

	/// apply force
    virtual void applyForce(const Vector3& forca, const Vector3& pointPivot) {}
    virtual void applyTorque(const Vector3& Torque) {}
    virtual void applyForceToCenterOfMass(const Vector3& force) {}

	/// apply impulse
    virtual void applyImpulse(const Vector3& impuls , const Vector3& point) {}
    virtual void applyImpulseAngular(const Vector3&  impuls ) {}
    virtual void applyImpulseLinear(const Vector3&  impuls ) {}


   protected:


	/// apply pseudo impulse
    virtual void applySplitImpulse(const Vector3& impuls, const Vector3& point ) {}
    virtual void applySplitImpulseAngular(const Vector3&  impuls ) {}
    virtual void applySplitImpulseLinear(const Vector3&  impuls ) {}

};

} /* namespace real_physics */

#endif /* SOURCE_ENGIE_DYNAMICS_BODY_RPPHYSICSBODY_H_ */
