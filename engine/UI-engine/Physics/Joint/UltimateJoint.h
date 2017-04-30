#ifndef ULTIMATEJOINT_H
#define ULTIMATEJOINT_H


#include "../../../physics-engine/physics.h"
#include "../Body/UltimatePhysicsBody.h"


namespace utility_engine
{



    struct UltimateJointInfo
    {


        static real_physics::rpDistanceJointInfo DistanceJointInfo( UltimatePhysicsBody* body1 ,
                                                                    UltimatePhysicsBody* body2 ,
                                                                    float& initDistance )
        {
            return real_physics::rpDistanceJointInfo( body1->getPhysicsBody() , body2->getPhysicsBody() , initDistance );
        }




        static real_physics::rpBallAndSocketJointInfo  BallAndSocketJointInfo( UltimatePhysicsBody* body1 ,
                                                                               UltimatePhysicsBody* body2 ,
                                                                               const Vector3& initAnchorPointWorldSpace )
        {
            real_physics::Vector3 anchor( initAnchorPointWorldSpace.x ,
                                          initAnchorPointWorldSpace.y ,
                                          initAnchorPointWorldSpace.z );

            return real_physics::rpBallAndSocketJointInfo( body1->getPhysicsBody() , body2->getPhysicsBody() , anchor );
        }



        static real_physics::rpFixedJointInfo  FixedJointInfo( UltimatePhysicsBody* body1 ,
                                                               UltimatePhysicsBody* body2 ,
                                                               const Vector3& initAnchorPointWorldSpace )
        {
            real_physics::Vector3 anchor( initAnchorPointWorldSpace.x ,
                                          initAnchorPointWorldSpace.y ,
                                          initAnchorPointWorldSpace.z );

            return real_physics::rpFixedJointInfo( body1->getPhysicsBody() , body2->getPhysicsBody() , anchor );
        }


        static real_physics::rpHingeJointInfo  HingeJointInfo( UltimatePhysicsBody* body1 ,
                                                               UltimatePhysicsBody* body2 ,
                                                               const Vector3& initAnchorPointWorldSpace ,
                                                               const Vector3& initAxisWorldSpace )
        {
            real_physics::Vector3 anchor( initAnchorPointWorldSpace.x ,
                                          initAnchorPointWorldSpace.y ,
                                          initAnchorPointWorldSpace.z );

            real_physics::Vector3 axis( initAxisWorldSpace.x ,
                                        initAxisWorldSpace.y ,
                                        initAxisWorldSpace.z );

            return real_physics::rpHingeJointInfo( body1->getPhysicsBody() , body2->getPhysicsBody() , anchor , axis );
        }



        static real_physics::rpHingeJointInfo  HingeJointInfo( UltimatePhysicsBody* body1 ,
                                                               UltimatePhysicsBody* body2 ,
                                                               const Vector3& initAnchorPointWorldSpace ,
                                                               const Vector3& initAxisWorldSpace ,
                                                               float initMinAngleLimit, float initMaxAngleLimit)
        {
            real_physics::Vector3 anchor( initAnchorPointWorldSpace.x ,
                                          initAnchorPointWorldSpace.y ,
                                          initAnchorPointWorldSpace.z );

            real_physics::Vector3 axis( initAxisWorldSpace.x ,
                                        initAxisWorldSpace.y ,
                                        initAxisWorldSpace.z );

            return real_physics::rpHingeJointInfo( body1->getPhysicsBody() , body2->getPhysicsBody() , anchor , axis  ,
                                                   initMinAngleLimit , initMaxAngleLimit );
        }



        static real_physics::rpHingeJointInfo  HingeJointInfo( UltimatePhysicsBody* body1 ,
                                                               UltimatePhysicsBody* body2 ,
                                                               const Vector3& initAnchorPointWorldSpace ,
                                                               const Vector3& initAxisWorldSpace ,
                                                               float initMinAngleLimit, float initMaxAngleLimit ,
                                                               float initMotorSpeed   , float initMaxMotorTorque)
        {
            real_physics::Vector3 anchor( initAnchorPointWorldSpace.x ,
                                          initAnchorPointWorldSpace.y ,
                                          initAnchorPointWorldSpace.z );

            real_physics::Vector3 axis( initAxisWorldSpace.x ,
                                        initAxisWorldSpace.y ,
                                        initAxisWorldSpace.z );

            return real_physics::rpHingeJointInfo( body1->getPhysicsBody() , body2->getPhysicsBody(), anchor , axis  ,
                                                   initMinAngleLimit , initMaxAngleLimit ,
                                                   initMotorSpeed , initMaxMotorTorque);
        }





        static real_physics::rpSliderJointInfo  SliderJointInfo( UltimatePhysicsBody* body1 ,
                                                                 UltimatePhysicsBody* body2 ,
                                                                 const Vector3& initAnchorPointWorldSpace ,
                                                                 const Vector3& initAxisWorldSpace )
        {
            real_physics::Vector3 anchor( initAnchorPointWorldSpace.x ,
                                          initAnchorPointWorldSpace.y ,
                                          initAnchorPointWorldSpace.z );

            real_physics::Vector3 axis( initAxisWorldSpace.x ,
                                        initAxisWorldSpace.y ,
                                        initAxisWorldSpace.z );

            return real_physics::rpSliderJointInfo( body1->getPhysicsBody() , body2->getPhysicsBody(), anchor , axis );
        }


        static real_physics::rpSliderJointInfo  SliderJointInfo( UltimatePhysicsBody* body1 ,
                                                                 UltimatePhysicsBody* body2 ,
                                                                 const Vector3& initAnchorPointWorldSpace ,
                                                                 const Vector3& initAxisWorldSpace ,
                                                                 float initMinTranslationLimit, float initMaxTranslationLimit)
        {
            real_physics::Vector3 anchor( initAnchorPointWorldSpace.x ,
                                          initAnchorPointWorldSpace.y ,
                                          initAnchorPointWorldSpace.z );

            real_physics::Vector3 axis( initAxisWorldSpace.x ,
                                        initAxisWorldSpace.y ,
                                        initAxisWorldSpace.z );

            return real_physics::rpSliderJointInfo( body1->getPhysicsBody() , body2->getPhysicsBody(), anchor , axis ,
                                                    initMinTranslationLimit , initMaxTranslationLimit );
        }


        static real_physics::rpSliderJointInfo  SliderJointInfo( UltimatePhysicsBody* body1 ,
                                                                 UltimatePhysicsBody* body2 ,
                                                                 const Vector3& initAnchorPointWorldSpace ,
                                                                 const Vector3& initAxisWorldSpace ,
                                                                 float initMinTranslationLimit , float initMaxTranslationLimit ,
                                                                 float initMotorSpeed          , float initMaxMotorForce)
        {
            real_physics::Vector3 anchor( initAnchorPointWorldSpace.x ,
                                          initAnchorPointWorldSpace.y ,
                                          initAnchorPointWorldSpace.z );

            real_physics::Vector3 axis( initAxisWorldSpace.x ,
                                        initAxisWorldSpace.y ,
                                        initAxisWorldSpace.z );

           return real_physics::rpSliderJointInfo( body1->getPhysicsBody() , body2->getPhysicsBody(), anchor , axis ,
                                                   initMinTranslationLimit , initMaxTranslationLimit ,
                                                   initMotorSpeed , initMaxMotorForce);
        }


    };



    class UltimateJoint
    {
       private:

        //--------------- Attribute ----------------------//

         real_physics::rpJoint *mJoint;


       public:


         UltimateJoint(){}

         /// Constructor joint
         UltimateJoint(real_physics::rpJoint *_joint);


         /// Return joint
         real_physics::rpJoint *getJoint() const;


         /// Return true if the collision between the two bodies of the rpJoint is enabled
         bool isCollisionEnabled() const;

         /// set false if the collision between the two bodies of the rpJoint is enabled
         void setIsCollisionEnabled(bool isCollisionEnabled);
    };


}
#endif // ULTIMATEJOINT_H
