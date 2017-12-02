#include "DynamicsWorld.h"


namespace utility_engine
{


    /// Constructor
    DynamicsWorld::DynamicsWorld(const Vector3 &gravity)
    {
        mDynamicsWorld = new real_physics::rpDynamicsWorld( real_physics::Vector3( gravity.x , gravity.y , gravity.z ));
    }


    /// Constructor
    DynamicsWorld::DynamicsWorld( real_physics::rpDynamicsWorld* world)
    : mDynamicsWorld(world)
    {
        assert(mDynamicsWorld != NULL);
    }


    /// Destructor
    DynamicsWorld::~DynamicsWorld()
    {
        destroy();
        delete mDynamicsWorld;
    }


    /// Create a rigid body into the physics world.
    UltimatePhysicsBody *DynamicsWorld::createRigidBody(const Matrix4 &transform)
    {
        UltimatePhysicsBody *rigidBody =  new UltimatePhysicsBody( mDynamicsWorld->createRigidBody(Matrix4ConvertToTransform(transform)) );
        mBodies.insert(rigidBody);
        return rigidBody;
    }


    /// Destroy a rigid body and all the joints which it belongs
    void DynamicsWorld::destroyBody(UltimatePhysicsBody *rigidBody)
    {
        mDynamicsWorld->destroyBody( rigidBody->getPhysicsBody() );
        mBodies.erase(rigidBody);
        delete rigidBody;
    }


    /// Create a constraint-joint into the physics world.
    UltimateJoint *DynamicsWorld::createJoint(const real_physics::rpJointInfo &jointInfo)
    {
         UltimateJoint *joint =  new UltimateJoint(mDynamicsWorld->createJoint(jointInfo));
         mJoints.insert(joint);
         return joint;
    }


    /// Destroy a constraint-joints
    void DynamicsWorld::destroyJoint(UltimateJoint *joint)
    {
        mDynamicsWorld->destroyJoint(joint->getJoint());
        mJoints.erase(joint);
        delete joint;
    }


    /// Update real-time physics simulate
    void DynamicsWorld::update(float timeStep )
    {
        assert( mDynamicsWorld != NULL );

        mDynamicsWorld->update( timeStep ,  mObserver );

        for(auto it = mBodies.begin(); it != mBodies.end(); ++it )
        {
            (*it)->update();
        }

    }

    /// Update real-time physics simulate (Fixed TimeStep)
    void DynamicsWorld::updateFixedStep(float timeStep  )
    {
       assert( mDynamicsWorld != NULL );

       mDynamicsWorld->updateFixedTime( timeStep , mObserver );

       for(auto it = mBodies.begin(); it != mBodies.end(); ++it )
       {
           (*it)->update();
       }
    }

    /// Realase and a delete memory
    void DynamicsWorld::destroy()
    {

        for(auto it = mBodies.begin(); it != mBodies.end(); ++it )
        {
            delete (*it);
        }

        for(auto it = mJoints.begin(); it != mJoints.end(); ++it )
        {
            delete (*it);
        }

        mBodies.clear();
        mJoints.clear();

        mDynamicsWorld->destroy();


    }


    real_physics::rpDynamicsWorld *DynamicsWorld::getDynamicsWorld() const
    {
        return mDynamicsWorld;
    }

    void DynamicsWorld::setObserver(const real_physics::ObserverSystem &observer)
    {
        mObserver = observer;
    }



}
