#include "DynamicsWorld.h"


namespace utility_engine
{


    /// Constructor
    DynamicsWorld::DynamicsWorld(const Vector3 &gravity)
    : mDynamicsWorld( real_physics::Vector3( gravity.x , gravity.y , gravity.z ))
    {

    }

    /// Destructor
    DynamicsWorld::~DynamicsWorld()
    {
       destroy();
    }


    /// Create a rigid body into the physics world.
    UltimatePhysicsBody *DynamicsWorld::createRigidBody(const Matrix4 &transform)
    {
        return new UltimatePhysicsBody( mDynamicsWorld.createRigidBody(Matrix4ConvertToTransform(transform)) );
    }


    /// Destroy a rigid body and all the joints which it belongs
    void DynamicsWorld::destroyBody(UltimatePhysicsBody *rigidBody)
    {
        mDynamicsWorld.destroyBody( rigidBody->getPhysicsBody() );
        delete rigidBody;
    }


    /// Create a constraint-joint into the physics world.
    UltimateJoint *DynamicsWorld::createJoint(const real_physics::rpJointInfo &jointInfo)
    {
         return  new UltimateJoint(mDynamicsWorld.createJoint(jointInfo));
    }


    /// Destroy a constraint-joints
    void DynamicsWorld::destroyJoint(real_physics::rpJoint *joint)
    {
        mDynamicsWorld.destroyJoint(joint);
        delete joint;
    }


    /// Update real-time physics simulate
    void DynamicsWorld::update( float timeStep )
    {
        mDynamicsWorld.update( timeStep );
    }

    /// Update real-time physics simulate (Fixed TimeStep)
    void DynamicsWorld::updateFixedStep(float timeStep)
    {
        mDynamicsWorld.updateFixedTime(timeStep);
    }

    /// Realase and a delete memory
    void DynamicsWorld::destroy()
    {
        mDynamicsWorld.destroy();
    }



}
