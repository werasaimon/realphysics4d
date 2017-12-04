#include "loadlibaryluaphysicsengine.h"
#include "../physics-engine/physics.h"


LoadLibaryLuaPhysicsEngine::LoadLibaryLuaPhysicsEngine(lua_State *_VirtualMashinLua)
    : LoadLibaryLua(_VirtualMashinLua)
{
    assert(_VirtualMashinLua);
}

void LoadLibaryLuaPhysicsEngine::LoadLibary()
{

    //---------------------------------------- Physics-Engine -------------------------------------------------------------//

    /// Vector2D
    importToScope(luabind::namespace_("physics")
                  [
                     luabind::class_<real_physics::Vector2>("vector2")
                    // constructor
                    .def(luabind::constructor<>())
                    .def(luabind::constructor<real_physics::scalar,real_physics::scalar>())
                    // method
                    .def("lenght"    , &real_physics::Vector2::length)
                    .def("lenght2"   , &real_physics::Vector2::lengthSquare)
                    .def("normalize" , &real_physics::Vector2::getUnit)
                    .def("dot"       , &real_physics::Vector2::dot)
                    .def("cross"     , &real_physics::Vector2::cross)
                    // operator
                    .def(luabind::const_self  +  real_physics::Vector2())
                    .def(luabind::const_self  -  real_physics::Vector2())
                    .def(luabind::const_self  *  real_physics::Vector2())
                    .def(luabind::const_self  /  real_physics::Vector2())
                    .def(luabind::const_self ==  real_physics::Vector2())
                    .def(luabind::const_self  *  real_physics::scalar())
                    .def(luabind::const_self  /  real_physics::scalar())
                    // value
                    .def_readwrite("x", &real_physics::Vector2::x)
                    .def_readwrite("y", &real_physics::Vector2::y)
                  ]);


    /// Vector3D
    importToScope(luabind::namespace_("physics")
                [
                         luabind::class_<real_physics::Vector3>("vector3")
                         // constructor
                        .def(luabind::constructor<>())
                        .def(luabind::constructor<real_physics::scalar,real_physics::scalar,real_physics::scalar>())
                         // method
                        .def("lenght"    , &real_physics::Vector3::length)
                        .def("lenght2"   , &real_physics::Vector3::length2)
                        .def("angle"     , &real_physics::Vector3::AngleBetweenVectors)
                        .def("normalize" , &real_physics::Vector3::getUnit)
                        .def("dot"       , &real_physics::Vector3::dot)
                        .def("cross"     , &real_physics::Vector3::cross)
                         // operator
                        .def(luabind::const_self  +  real_physics::Vector3())
                        .def(luabind::const_self  -  real_physics::Vector3())
                        .def(luabind::const_self  *  real_physics::Vector3())
                        .def(luabind::const_self  /  real_physics::Vector3())
                        .def(luabind::const_self ==  real_physics::Vector3())
                        .def(luabind::const_self  *  real_physics::scalar())
                        .def(luabind::const_self  /  real_physics::scalar())
                         // value
                        .def_readwrite("x", &real_physics::Vector3::x)
                        .def_readwrite("y", &real_physics::Vector3::y)
                        .def_readwrite("z", &real_physics::Vector3::z)
                ]);


    /// Matrix2x2
    importToScope(luabind::namespace_("physics")
                  [
                    luabind::class_<real_physics::Matrix2x2>("matrix2x2")
                   // constructor
                   .def(luabind::constructor<>())
                   .def(luabind::constructor<real_physics::scalar,real_physics::scalar,
                                             real_physics::scalar,real_physics::scalar>())
                   // method
                   .def( "identity"    , &real_physics::Matrix2x2::setToIdentity)
                   .def( "determinant" , &real_physics::Matrix2x2::getDeterminant)
                   .def( "inverse"     , &real_physics::Matrix2x2::getInverse)
                   .def( "transpose"   , &real_physics::Matrix2x2::getTranspose)
                   // operator
                   .def(luabind::const_self  +  real_physics::Matrix2x2())
                   .def(luabind::const_self  -  real_physics::Matrix2x2())
                   .def(luabind::const_self  *  real_physics::Matrix2x2())
                   .def(luabind::const_self  *  real_physics::scalar())

                  ]);



    /// Matrix3x3
    importToScope(luabind::namespace_("physics")
                        [
                                  luabind::class_<real_physics::Matrix3x3>("matrix3x3")
                                  // constructor
                                  .def(luabind::constructor<>())
                                  //.def(luabind::constructor<real_physics::Quaternion>())
                                  .def(luabind::constructor<real_physics::scalar,real_physics::scalar,real_physics::scalar,
                                                            real_physics::scalar,real_physics::scalar,real_physics::scalar,
                                                            real_physics::scalar,real_physics::scalar,real_physics::scalar>())
                                  // method
                                  .def( "identity"    , &real_physics::Matrix3x3::setToIdentity)
                                  .def( "determinant" , &real_physics::Matrix3x3::getDeterminant)
                                  .def( "inverse"     , &real_physics::Matrix3x3::getInverse)
                                  .def( "transpose"   , &real_physics::Matrix3x3::getTranspose)
                                  // operator
                                  .def(luabind::const_self  +  real_physics::Matrix3x3())
                                  .def(luabind::const_self  -  real_physics::Matrix3x3())
                                  .def(luabind::const_self  *  real_physics::Matrix3x3())
                                  .def(luabind::const_self  *  real_physics::scalar())
                        ]);


    /// Matrix3x3
    importToScope(luabind::namespace_("physics")
                  [
                         luabind::class_<real_physics::Matrix4x4>("matrix4x4")
                        // constructor
                        .def(luabind::constructor<>())
                        // method
                        .def( "identity"   , &real_physics::Matrix4x4::setToIdentity)
                  ]);


    /// Quaternion
    importToScope(luabind::namespace_("physics")
                        [
                          luabind::class_<real_physics::Quaternion>("quaternion")
                         // constructor
                         .def(luabind::constructor<>())
                         .def(luabind::constructor<real_physics::Vector3,real_physics::scalar>())
                         // method
                         .def("v"   , &real_physics::Quaternion::getVectorV)
                         .def("r"   , &real_physics::Quaternion::getAngle)
                         .def("mat3", &real_physics::Quaternion::getMatrix)
                       ]);


    /// Transform
    importToScope( luabind::namespace_("physics")
                       [
                          luabind::class_<real_physics::Transform>("transform")
                          // constructor
                          .def(luabind::constructor<const real_physics::Vector3& ,const real_physics::Quaternion&>())
                          .def( "position"   ,  &real_physics::Transform::getPosition    )
                          .def( "quaternion" ,  &real_physics::Transform::getOrientation )
                          .def( "basis"      ,  &real_physics::Transform::getBasis       )

                       ]);



    //---------------------------------------- Physics-Engine -------------------------------------------------------------//


    /// Collision shape virtual
    importToScope( luabind::namespace_("physics")
                         [
                            luabind::class_<real_physics::rpCollisionShape>("shape")
                         ]);



    /// Collision shape box
    importToScope( luabind::namespace_("physics")
                         [
                            luabind::class_< real_physics::rpBoxShape ,  luabind::bases<real_physics::rpCollisionShape> >("shape_box")
                            // constructor
                            .def(luabind::constructor<const real_physics::Vector3&>())
                            .def(luabind::constructor<const real_physics::Vector3& , real_physics::scalar>())
                         ]);



    /// Collsion shape shpere
    importToScope( luabind::namespace_("physics")
                         [
                            luabind::class_< real_physics::rpSphereShape ,  luabind::bases<real_physics::rpCollisionShape> >("shape_sphere")
                            // constructor
                            .def(luabind::constructor<real_physics::scalar>())
                         ]);


    /// Convex-Hull Geometry
    importToScope( luabind::namespace_("physics")
                         [
                             luabind::class_<real_physics::rpModelConvexHull>("hull")
                             // constructor
                            .def(luabind::constructor<const real_physics::Vector3*, real_physics::uint>())
                         ]);


    /// Collsion shape convex-hull
    importToScope( luabind::namespace_("physics")
                         [
                             luabind::class_<real_physics::rpConvexHullShape , luabind::bases<real_physics::rpCollisionShape>>("shape_hull")
                             // constructor
                             .def(luabind::constructor<real_physics::rpModelConvexHull*>())

                         ]);



    /// Meneger systems collisions
    importToScope( luabind::namespace_("physics")
                         [
                            luabind::class_<real_physics::rpCollisionManager>("collid_meneger")
                             // constructor
                            .def(luabind::constructor<>())
                         ]);


    ///Abstract on body
    importToScope( luabind::namespace_("physics")
                         [
                            luabind::class_<real_physics::rpBody>("body")
                            // constructor
                           .def(luabind::constructor<real_physics::bodyindex>())
                         ]);


    ///Collision on body
    importToScope( luabind::namespace_("physics")
                         [
                                luabind::class_<real_physics::rpCollisionBody , luabind::bases<real_physics::rpBody>>("collid_body")
                                // constructor
                                .def(luabind::constructor<const real_physics::Transform& , real_physics::rpCollisionManager* , real_physics::bodyindex>())
                                .def( "transform" , &real_physics::rpCollisionBody::getTransform )
                                .enum_("BodyType")
                                [
                                  luabind::value("static"    ,real_physics::BodyType::STATIC),
                                  luabind::value("dynamic"   ,real_physics::BodyType::DYNAMIC),
                                  luabind::value("kinematic" ,real_physics::BodyType::KINEMATIC)
                                ]
                         ]);



    importToScope( luabind::namespace_("physics")
                         [
                               luabind::class_<real_physics::rpCollisionWorld>("collisions_world")
                               // constructor
                               .def(luabind::constructor<>())
                         ]);

    //---------------------------------------- Physics-Engine -------------------------------------------------------------//

    ///Physics material
    importToScope( luabind::namespace_("physics")
                         [
                                luabind::class_<real_physics::rpPhysicsMaterial>("material")
                                // constructor
                                .def(luabind::constructor<>())
                         ]);


    ///Physics on object
    importToScope(  luabind::namespace_("physics")
                          [
                              luabind::class_<real_physics::rpPhysicsObject , luabind::bases<real_physics::rpCollisionBody> >("physics_object")
                              // constructor
                              .def(luabind::constructor<const real_physics::Transform& , real_physics::rpCollisionManager* , real_physics::bodyindex>())
                              .enum_("BodyType")
                              [
                                luabind::value("static"    ,real_physics::BodyType::STATIC),
                                luabind::value("dynamic"   ,real_physics::BodyType::DYNAMIC),
                                luabind::value("kinematic" ,real_physics::BodyType::KINEMATIC)
                              ]
                         ]);



    ///Physics on body
    importToScope( luabind::namespace_("physics")
                         [
                            luabind::class_<real_physics::rpPhysicsBody , luabind::bases<real_physics::rpPhysicsObject , real_physics::rpCollisionBody> >("physics_body")
                            // constructor
                            .def(luabind::constructor<const real_physics::Transform& , real_physics::rpCollisionManager* , real_physics::bodyindex>())
                            .def( "integrate"           ,  &real_physics::rpPhysicsBody::Integrate)
                            .def( "applyGravity"        ,  &real_physics::rpPhysicsBody::applyGravity)
                            .def( "applyImpulseLinear"  ,  &real_physics::rpPhysicsBody::applyImpulseLinear)
                            .def( "applyImpulseAngular" ,  &real_physics::rpPhysicsBody::applyImpulseAngular)
                            .def( "applyImpulse"        ,  &real_physics::rpPhysicsBody::applyImpulse)
                            .def( "applyForce"          ,  &real_physics::rpPhysicsBody::applyForceToCenterOfMass)
                            .def( "applyForce"          ,  &real_physics::rpPhysicsBody::applyForce)
                            .def( "applyTorque"         ,  &real_physics::rpPhysicsBody::applyTorque)
                         ]);


    ///Physics on rigid-body
    importToScope( luabind::namespace_("physics")
                         [
                            luabind::class_<real_physics::rpPhysicsRigidBody , luabind::bases<real_physics::rpPhysicsBody,real_physics::rpPhysicsObject,real_physics::rpCollisionBody>>("rigid_body")
                            // constructor
                            .def(luabind::constructor<const real_physics::Transform& , real_physics::rpCollisionManager* , real_physics::bodyindex>())
                         ]);




    //------------------------------------------ Joints Info -----------------------------------------------------//

    importToScope( luabind::namespace_("physics")
                         [
                               luabind::class_<real_physics::rpJointInfo>("joint_info")
                              // constructor
                              .def(luabind::constructor<real_physics::JointType>())
                              .def(luabind::constructor<real_physics::rpPhysicsBody* , real_physics::rpPhysicsBody* , real_physics::JointType>())
                              .enum_("JointType")
                              [
                                luabind::value("ball"     ,real_physics::JointType::BALLSOCKETJOINT),
                                luabind::value("slider"   ,real_physics::JointType::SLIDERJOINT),
                                luabind::value("hinge"    ,real_physics::JointType::HINGEJOINT),
                                luabind::value("fixed"    ,real_physics::JointType::FIXEDJOINT),
                                luabind::value("distance" ,real_physics::JointType::DISTANCEJOINT)
                              ]
                         ]);



    importToScope( luabind::namespace_("physics")
                         [
                            luabind::class_<real_physics::rpDistanceJointInfo , luabind::bases<real_physics::rpJointInfo> >("dist_info")
                            // constructor
                           .def(luabind::constructor<real_physics::rpPhysicsBody* , real_physics::rpPhysicsBody* , const real_physics::scalar&>())
                         ]);



    importToScope( luabind::namespace_("physics")
                         [
                             luabind::class_<real_physics::rpBallAndSocketJointInfo , luabind::bases<real_physics::rpJointInfo> >("ball_info")
                             // constructor
                             .def(luabind::constructor<real_physics::rpPhysicsBody* , real_physics::rpPhysicsBody* , const real_physics::Vector3&>())
                         ]);



    importToScope( luabind::namespace_("physics")
                         [
                             luabind::class_<real_physics::rpHingeJointInfo , luabind::bases<real_physics::rpJointInfo> >("hinge_info")
                             // constructor
                             .def(luabind::constructor<real_physics::rpPhysicsBody* , real_physics::rpPhysicsBody* ,
                                                       const real_physics::Vector3& , const real_physics::Vector3&>())

                             .def(luabind::constructor<real_physics::rpPhysicsBody* , real_physics::rpPhysicsBody* ,
                                                       const real_physics::Vector3& , const real_physics::Vector3& ,
                                                       real_physics::scalar , real_physics::scalar>())

                             .def(luabind::constructor<real_physics::rpPhysicsBody* , real_physics::rpPhysicsBody* ,
                                                       const real_physics::Vector3& , const real_physics::Vector3& ,
                                                       real_physics::scalar , real_physics::scalar ,
                                                       real_physics::scalar , real_physics::scalar>())
                         ]);



    importToScope( luabind::namespace_("physics")
                         [
                             luabind::class_<real_physics::rpSliderJointInfo , luabind::bases<real_physics::rpJointInfo> >("slider_info")
                             // constructor
                             .def(luabind::constructor<real_physics::rpPhysicsBody* , real_physics::rpPhysicsBody* ,
                                                       const real_physics::Vector3& , const real_physics::Vector3&>())

                             .def(luabind::constructor<real_physics::rpPhysicsBody* , real_physics::rpPhysicsBody* ,
                                                       const real_physics::Vector3& , const real_physics::Vector3& ,
                                                              real_physics::scalar  ,       real_physics::scalar>())
                         ]);


    importToScope( luabind::namespace_("physics")
                         [
                             luabind::class_<real_physics::rpFixedJointInfo , luabind::bases<real_physics::rpJointInfo> >("fixed_info")
                            // constructor
                            .def(luabind::constructor<real_physics::rpPhysicsBody* , real_physics::rpPhysicsBody* , const real_physics::Vector3&>())

                         ]);


    //------------------------------------------ Joints -----------------------------------------------------//

    importToScope(  luabind::namespace_("physics")
                          [
                             luabind::class_<real_physics::rpJoint>("joint")
                             // constructor
                             .def(luabind::constructor<const real_physics::rpJointInfo&>())
                          ]);





    importToScope(  luabind::namespace_("physics")
                     [
                         luabind::class_<real_physics::rpDistanceJoint , luabind::bases<real_physics::rpJoint> >("dist_joint")
                         // constructor
                         .def(luabind::constructor<const real_physics::rpDistanceJointInfo&>())
                     ]);


    importToScope(  luabind::namespace_("physics")
                    [
                       luabind::class_<real_physics::rpBallAndSocketJoint , luabind::bases<real_physics::rpJoint> >("ball_joint")
                       // constructor
                       .def(luabind::constructor<const real_physics::rpBallAndSocketJointInfo&>())
                    ]);



    importToScope(  luabind::namespace_("physics")
                    [
                        luabind::class_<real_physics::rpHingeJoint , luabind::bases<real_physics::rpJoint> >("hinge_joint")
                        // constructor
                        .def(luabind::constructor<const real_physics::rpHingeJointInfo&>())
                    ]);



    importToScope(  luabind::namespace_("physics")
                    [
                        luabind::class_<real_physics::rpSliderJoint , luabind::bases<real_physics::rpJoint> >("slider_joint")
                        // constructor
                        .def(luabind::constructor<const real_physics::rpSliderJointInfo&>())
                    ]);



    importToScope(  luabind::namespace_("physics")
                    [
                        luabind::class_<real_physics::rpFixedJoint , luabind::bases<real_physics::rpJoint> >("fixed_joint")
                        // constructor
                        .def(luabind::constructor<const real_physics::rpFixedJointInfo&>())
                    ]);

      //------------------------------------------------- physics world ------------------------------------------------------------//

    importToScope(  luabind::namespace_("physics")
                    [
                       luabind::class_< real_physics::rpDynamicsWorld ,luabind::bases<real_physics::rpCollisionWorld>>("dynamics_world")
                       // constructor
                       .def(luabind::constructor<const real_physics::Vector3&>())
                    ]);


}





