#ifndef DYNAMICSWORLD_H
#define DYNAMICSWORLD_H


#include "../../physics-engine/physics.h"

#include "../maths/Vector3.h"
#include "../maths/Matrix4.h"

#include "../Mesh/Mesh.h"

#include "Body/UltimatePhysicsBody.h"
#include "Joint/UltimateJoint.h"




namespace utility_engine
{


    class DynamicsWorld
    {
       private:

            //------------------------ Attribute ------------------------//
            /// Dynamics physics world
            real_physics::rpDynamicsWorld    *mDynamicsWorld;
            std::set<UltimatePhysicsBody*>    mBodies;
            std::set<UltimateJoint*>          mJoints;

            real_physics::ObserverSystem      mObserver;

            //---------------------- Constructor ------------------------//
            /// Private copy-constructor
            DynamicsWorld(const DynamicsWorld& world);

            /// Private assignment operator
            DynamicsWorld& operator=(const DynamicsWorld& world);



       public:


            DynamicsWorld( const Vector3& gravity );

            DynamicsWorld( real_physics::rpDynamicsWorld* world );

            /// Destructor
           ~DynamicsWorld();



            //------------------- value -------------------//
            real_physics::rpDynamicsWorld *getDynamicsWorld() const;

            void setObserver(const real_physics::ObserverSystem &observer);


            //------------------------- Method --------------------------//

            /// Create a rigid body into the physics world.
            UltimatePhysicsBody* createRigidBody(const Matrix4& transform);

            /// Destroy a rigid body and all the joints which it belongs
            void destroyBody(UltimatePhysicsBody* rigidBody);


            /// Create a constraint-joint into the physics world.
            UltimateJoint* createJoint(const real_physics::rpJointInfo &jointInfo);

            /// Destroy a constraint-joints
            void destroyJoint(UltimateJoint *joint);

            /// Update real-time physics simulate
            void update(float timeStep );

            /// Update real-time physics simulate (Fixed-TimeStep)
            void updateFixedStep(float timeStep );

            /// Realase and a delete memory
            void destroy();


    };


}
#endif // DYNAMICSWORLD_H
