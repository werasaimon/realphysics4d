/*
 * rpRigidPhysicsBody.h
 *
 *  Created on: 14 дек. 2016 г.
 *      Author: wera
 */

#ifndef SOURCE_ENGIE_DYNAMICS_BODY_RPPHYSICSBODY_H_
#define SOURCE_ENGIE_DYNAMICS_BODY_RPPHYSICSBODY_H_


#include "rpPhysicsObject.h"

namespace real_physics
{

//// Class declarations
class   rpJoint;

///List element joint
typedef rpListElement<rpJoint> JointListElement;

/// type of body dynamics
enum PhysicsBodyType { RIGID_BODY ,
                       VELERT_BODY };



struct ObserverSystem
{
    Vector3 position     = Vector3(0,0,0);
    Vector3 ang_velocity = Vector3(0,0,0);
    Vector3 lin_velocity = Vector3(0,0,0);
};


class rpPhysicsBody: public rpPhysicsObject
{

   protected:

       /// Type structure of the body
       PhysicsBodyType mTypePhysics;


       /// First element of the linked list of joints involving this body
       JointListElement*  mJointsList;


       /// Private copy-constructor
       rpPhysicsBody(const rpPhysicsBody& body);

       /// Private assignment operator
       rpPhysicsBody& operator=(const rpPhysicsBody& body);


   public:

             rpPhysicsBody(const Transform& transform, rpCollisionManager *CollideWorld, bodyindex id );
	virtual ~rpPhysicsBody();


    /// integrate to Euler method
    virtual void Integrate( scalar _dt , ObserverSystem _observer ) {}

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

    //**********************************************//

    /// remove of the list joints
    virtual void removeJointFromJointsList( const rpJoint* joint);


    //**********************************************//

        //     virtual void UpdateMatrices(){}

   protected:


	/// apply pseudo impulse
    virtual void applySplitImpulse(const Vector3& impuls, const Vector3& point ) {}
    virtual void applySplitImpulseAngular(const Vector3&  impuls ) {}
    virtual void applySplitImpulseLinear(const Vector3&  impuls ) {}

   //--------------------------- frindship -----------------------------//

    friend class rpRigidPhysicsBody;
    friend class rpDynamicsWorld;

};

} /* namespace real_physics */

#endif /* SOURCE_ENGIE_DYNAMICS_BODY_RPPHYSICSBODY_H_ */
