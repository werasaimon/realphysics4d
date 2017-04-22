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

    /// Vector3D
    importToScope(luabind::namespace_("physics")
                [
                         luabind::class_<real_physics::Vector3>("vector3")
                         // constructor
                        .def(luabind::constructor<>())
                        .def(luabind::constructor<real_physics::scalar,real_physics::scalar,real_physics::scalar>())
                         // method
                        .def("lenght" , &real_physics::Vector3::length)
                        .def("lenght2", &real_physics::Vector3::length2)
                        .def("angle"  , &real_physics::Vector3::AngleBetweenVectors)
                         // operator
                        .def(luabind::const_self  +  real_physics::Vector3())
                        .def(luabind::const_self  -  real_physics::Vector3())
                        .def(luabind::const_self  *  real_physics::Vector3())
                        .def(luabind::const_self  /  real_physics::Vector3())
                        .def(luabind::const_self ==  real_physics::Vector3())
                         // value
                        .def_readwrite("x", &real_physics::Vector3::x)
                        .def_readwrite("y", &real_physics::Vector3::y)
                        .def_readwrite("z", &real_physics::Vector3::z)
                ]);


    /// Matrix3x3
    importToScope(luabind::namespace_("physics")
                        [
                                  luabind::class_<real_physics::Matrix3x3>("matrix3")
                                  // constructor
                                  .def(luabind::constructor<>())
                                  //.def(luabind::constructor<real_physics::Quaternion>())
                                  .def(luabind::constructor<real_physics::scalar,real_physics::scalar,real_physics::scalar,
                                                            real_physics::scalar,real_physics::scalar,real_physics::scalar,
                                                            real_physics::scalar,real_physics::scalar,real_physics::scalar>())
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
                            luabind::class_< real_physics::rpSphereShape ,  luabind::bases<real_physics::rpCollisionShape> >("shape-sphere")
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
                             luabind::class_<real_physics::rpConvexHullShape , luabind::bases<real_physics::rpCollisionShape>>("shape-hull")
                             // constructor
                             .def(luabind::constructor<real_physics::rpModelConvexHull*>())

                         ]);



    /// Meneger systems collisions
    importToScope( luabind::namespace_("physics")
                         [
                            luabind::class_<real_physics::rpCollisionDetection>("collid-meneger")
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
                                luabind::class_<real_physics::rpCollisionBody ,luabind::bases<real_physics::rpBody>>("collid-body")
                                // constructor
                                .def(luabind::constructor<const real_physics::Transform& , real_physics::rpCollisionDetection* , real_physics::bodyindex>())
                                .enum_("BodyType")
                                [
                                  luabind::value("static"    ,real_physics::BodyType::STATIC),
                                  luabind::value("dynamic"   ,real_physics::BodyType::DYNAMIC),
                                  luabind::value("kinematic" ,real_physics::BodyType::KINEMATIC)
                                ]
                         ]);



    importToScope( luabind::namespace_("physics")
                         [
                               luabind::class_<real_physics::rpCollisionWorld>("collisions-world")
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
                              luabind::class_<real_physics::rpPhysicsObject , luabind::bases<real_physics::rpCollisionBody> >("physics-object")
                              // constructor
                              .def(luabind::constructor<const real_physics::Transform& , real_physics::rpCollisionDetection* , real_physics::bodyindex>())
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
                            luabind::class_<real_physics::rpPhysicsBody , luabind::bases<real_physics::rpPhysicsObject> >("physics-body")
                            // constructor
                            .def(luabind::constructor<const real_physics::Transform& , real_physics::rpCollisionDetection* , real_physics::bodyindex>())
                         ]);


    ///Physics on rigid-body
    importToScope( luabind::namespace_("physics")
                         [
                            luabind::class_<real_physics::rpRigidPhysicsBody , luabind::bases<real_physics::rpPhysicsBody> >("rigid-body")
                            // constructor
                            .def(luabind::constructor<const real_physics::Transform& , real_physics::rpCollisionDetection* , real_physics::bodyindex>())
                         ]);





    importToScope( luabind::namespace_("physics")
                         [
                               luabind::class_<real_physics::rpJointInfo>("joint-info")
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
                            luabind::class_<real_physics::rpDistanceJointInfo , luabind::bases<real_physics::rpJointInfo> >("dist-info")
                            // constructor
                           .def(luabind::constructor<real_physics::rpPhysicsBody* , real_physics::rpPhysicsBody* , const real_physics::scalar&>())
                         ]);



    importToScope( luabind::namespace_("physics")
                         [
                             luabind::class_<real_physics::rpBallAndSocketJointInfo , luabind::bases<real_physics::rpJointInfo> >("ball-info")
                             // constructor
                             .def(luabind::constructor<real_physics::rpPhysicsBody* , real_physics::rpPhysicsBody* , const real_physics::Vector3&>())
                         ]);




    importToScope(  luabind::namespace_("physics")
                          [
                             luabind::class_<real_physics::rpJoint>("joint")
                             // constructor
                             .def(luabind::constructor<const real_physics::rpJointInfo&>())
                          ]);





    importToScope(  luabind::namespace_("physics")
                          [
                             luabind::class_<real_physics::rpDistanceJoint , luabind::bases<real_physics::rpJoint> >("dist-joint")
                             // constructor
                             .def(luabind::constructor<const real_physics::rpDistanceJointInfo&>())
                          ]);


    importToScope(  luabind::namespace_("physics")
                          [
                             luabind::class_<real_physics::rpBallAndSocketJoint , luabind::bases<real_physics::rpJoint> >("ball-joint")
                              // constructor
                             .def(luabind::constructor<const real_physics::rpBallAndSocketJointInfo&>())
                          ]);



    importToScope(  luabind::namespace_("physics")
                          [
                            luabind::class_< real_physics::rpDynamicsWorld ,luabind::bases<real_physics::rpCollisionWorld>>("dynamics-world")
                            // constructor
                            .def(luabind::constructor<const real_physics::Vector3&>())
                          ]);


}
