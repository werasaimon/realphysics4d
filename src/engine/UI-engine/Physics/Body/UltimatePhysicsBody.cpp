#include "UltimatePhysicsBody.h"


namespace utility_engine
{




UltimatePhysicsBody::UltimatePhysicsBody( real_physics::rpPhysicsBody *_PhysicsBody)
 : mPhysicsBody(_PhysicsBody)
{

}

UltimatePhysicsBody::~UltimatePhysicsBody()
{
//    if(mPhysicsBody != NULL)
//    {
//        delete mPhysicsBody;
//        mPhysicsBody = NULL;
//    }

    mGroupMesh.destroy();
}



//------------------------------ Method ------------------------------------//

/// Initilization  body type : ( DYNAMIC , STATIC , KINEMATIC )
void UltimatePhysicsBody::setType(real_physics::BodyType type)
{
   mPhysicsBody->setType( type );
}


/// Apply gravity
void UltimatePhysicsBody::applyGravity(const Vector3& gravity )
{
    mPhysicsBody->applyGravity( real_physics::Vector3(gravity.x , gravity.y , gravity.z) );
}





/// Apply force pivot point
void  UltimatePhysicsBody::applyForce( const Vector3& force , const Vector3& point )
{
    mPhysicsBody->applyForce( real_physics::Vector3( force.x , force.y , force.z ) ,
                              real_physics::Vector3( point.x , point.y , point.z ));
}

/// Apply torque
void  UltimatePhysicsBody::applyTorque( const Vector3& torque )
{
    mPhysicsBody->applyTorque( real_physics::Vector3( torque.x , torque.y , torque.z ) );
}

/// Apply force to center Of mass
void  UltimatePhysicsBody::applyForceToCenterOfMass( const Vector3& force )
{
    mPhysicsBody->applyForceToCenterOfMass( real_physics::Vector3( force.x , force.y , force.z ) );
}





/// Apply impulse pivot point
void  UltimatePhysicsBody::applyImpulse( const Vector3& momentImpuls , const Vector3& point )
{
    mPhysicsBody->applyImpulse( real_physics::Vector3(momentImpuls.x , momentImpuls.y , momentImpuls.z) ,
                                real_physics::Vector3(point.x , point.y , point.z ));
}

/// Apply impulse linear cenetr of mass
void  UltimatePhysicsBody::applyImpulseLinear( const Vector3& momentImpuls )
{
    mPhysicsBody->applyImpulseLinear( real_physics::Vector3(momentImpuls.x , momentImpuls.y , momentImpuls.z) );
}

/// Apply impulse anglar
void  UltimatePhysicsBody::applyImpulseAngular( const Vector3& momentImpuls )
{
    mPhysicsBody->applyImpulseAngular( real_physics::Vector3(momentImpuls.x , momentImpuls.y , momentImpuls.z) );
}




/// Update real-time
void UltimatePhysicsBody::update()
{
   mGroupMesh.updateTransform( TransformConvertToMatrix4(mPhysicsBody->getTransform()) );
}

//------------------------------- Value ------------------------------------//


/// Physics body
real_physics::rpPhysicsBody *UltimatePhysicsBody::getPhysicsBody()
{
    return mPhysicsBody;
}


}
