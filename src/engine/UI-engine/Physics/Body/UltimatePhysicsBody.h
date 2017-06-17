#ifndef ULTIMATEPHYSICSBODY_H
#define ULTIMATEPHYSICSBODY_H

#include "../../maths/Vector2.h"
#include "../../maths/Vector3.h"
#include "../../maths/Vector4.h"
#include "../../maths/Matrix3.h"
#include "../../maths/Matrix4.h"

#include "../../../physics-engine/physics.h"
#include "GroupMesh.h"

namespace utility_engine
{

/// Enumeration for the type of a body
/// STATIC : A static body has infinite mass, zero velocity but the position can be
///          changed manually. A static body does not collide with other static or kinematic bodies.
/// KINEMATIC : A kinematic body has infinite mass, the velocity can be changed manually and its
///             position is computed by the physics engine. A kinematic body does not collide with
///             other static or kinematic bodies.
/// DYNAMIC : A dynamic body has non-zero mass, non-zero velocity determined by forces and its
///           position is determined by the physics engine. A dynamic body can collide with other
///           dynamic, static or kinematic bodies.

    /// Ultimatium physics body
    class UltimatePhysicsBody
    {


       private:

            //------------------- Attribute ------------------------//
            GroupMesh                    mGroupMesh;
            real_physics::rpPhysicsBody *mPhysicsBody;



       public:

            UltimatePhysicsBody( real_physics::rpPhysicsBody *_PhysicsBody );
           ~UltimatePhysicsBody();


            //--------------------------------- Add collisoon geometry shape convex-hull ------------------------------------------------//


            void addCollisionGeometry_ConvexHull( Mesh *mesh , float massa )
            {
                real_physics::rpModelConvexHull*   initHull = new real_physics::rpModelConvexHull(MeshConvertToVertexes(mesh));
                real_physics::rpConvexHullShape* convexHull = new real_physics::rpConvexHullShape(initHull);

                addCollisionGeometry( convexHull , mesh->getTransformMatrix() , massa , mesh );
            }


            void addCollisionGeometry_ConvexHull( Mesh *mesh , const Matrix4& transform , float massa )
            {
                real_physics::rpModelConvexHull*   initHull = new real_physics::rpModelConvexHull(MeshConvertToVertexes(mesh));
                real_physics::rpConvexHullShape* convexHull = new real_physics::rpConvexHullShape(initHull);

                addCollisionGeometry( convexHull , transform , massa , mesh );
            }

            //---------------------------------- Add collision geometry shape shpere  ----------------------------------------------------//

            void addCollisionGeometry_Sphere( Mesh *mesh , float radius , float massa )
            {
                real_physics::rpCollisionShape *shape = new real_physics::rpSphereShape(radius);
                addCollisionGeometry( shape  , mesh->getTransformMatrix() , massa , mesh );
            }


            void addCollisionGeometry_Sphere( Mesh *mesh , const Matrix4& transform  , float radius , float massa )
            {
                real_physics::rpCollisionShape *shape = new real_physics::rpSphereShape(radius);
                addCollisionGeometry( shape  , transform , massa , mesh );
            }

             //--------------------------------- Add collision geometry shape Box halfSize ----------------------------------//


            void addCollisionGeometry_Box( Mesh *mesh , const Vector3& halfSize , float massa )
            {
                real_physics::rpCollisionShape *shape = new real_physics::rpBoxShape( real_physics::Vector3( halfSize.x , halfSize.y  , halfSize.z) );
                addCollisionGeometry( shape  , mesh->getTransformMatrix() , massa , mesh );
            }


            void addCollisionGeometry_Box( Mesh *mesh , const Matrix4& transform  , const Vector3& halfSize , float massa )
            {
                 real_physics::rpCollisionShape *shape = new real_physics::rpBoxShape( real_physics::Vector3( halfSize.x , halfSize.y  , halfSize.z) );
                addCollisionGeometry( shape  , transform , massa , mesh );
            }




            //--------------------- Method -------------------------//

            /// Add collision geometry for physics simulate real-time
            void addCollisionGeometry( real_physics::rpCollisionShape *shape  , const Matrix4& transform  , float massa ,  Mesh *mesh = NULL )
            {
                assert( shape && massa >= 1.0 );
                if( shape != NULL )  mPhysicsBody->addCollisionShape( shape , massa , Matrix4ConvertToTransform(transform) );
                if(  mesh != NULL )  mGroupMesh.addInitMesh( mesh , transform );
            }

            /// Initilization  body type : ( DYNAMIC , STATIC , KINEMATIC )
            void setType( real_physics::BodyType type );

            /// Apply gravity
            void applyGravity( const Vector3& gravity );


            /// Apply force pivot point
            void applyForce( const Vector3& force , const Vector3& point );
            /// Apply torque
            void applyTorque( const Vector3& torque );
            /// Apply force to center Of mass
            void applyForceToCenterOfMass( const Vector3& force );


            /// Apply impulse pivot point
            void applyImpulse( const Vector3& momentImpuls , const Vector3& point );
            /// Apply impulse linear cenetr of mass
            void applyImpulseLinear( const Vector3& momentImpuls );
            /// Apply impulse anglar
            void applyImpulseAngular( const Vector3& momentImpuls );


            /// Update real-time
            void update();




            //---------------------- Value -------------------------//

            /// Physics body
            real_physics::rpPhysicsBody *getPhysicsBody();


            /// Type body
            enum BodyType { STATIC , KINEMATIC , DYNAMIC };
    };

}

#endif // ULTIMATEPHYSICSBODY_H
