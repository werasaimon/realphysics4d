#include "loadlibaryluauiengine.h"
#include "../UI-engine/engine.h"


LoadLibaryLuaUIEngine::LoadLibaryLuaUIEngine( lua_State *_VirtualMashinLua )
  : LoadLibaryLua( _VirtualMashinLua )
{
    assert(_VirtualMashinLua);
}

void LoadLibaryLuaUIEngine::LoadLibary()
{


    //--------------------------------- UI-Engine -------------------------------------//
    importToScope( luabind::class_<utility_engine::Vector2>("vector2")
                       // constructor
                        .def(luabind::constructor<>())
                        .def(luabind::constructor<float,float>())
                       // method
                       .def("lenght"     , &utility_engine::Vector2::length)
                       .def("lenght2"    , &utility_engine::Vector2::lengthSquared)
                       .def("normalize"  , &utility_engine::Vector2::normalize)
                       .def("dot"        , &utility_engine::Vector2::dot)
                       .def("cross"      , &utility_engine::Vector2::cross)
                       // operator
                       .def(luabind::const_self  +  utility_engine::Vector2())
                       .def(luabind::const_self  -  utility_engine::Vector2())
                       .def(luabind::const_self  *  utility_engine::Vector2())
                       .def(luabind::const_self ==  utility_engine::Vector2())
                       .def(luabind::const_self  /  float())
                       .def(luabind::const_self  *  float())
                       // value
                       .def_readwrite("x", &utility_engine::Vector2::x)
                       .def_readwrite("y", &utility_engine::Vector2::y));


    importToScope( luabind::class_<utility_engine::Vector3>("vector3")
                         // constructor
                         .def(luabind::constructor<>())
                         .def(luabind::constructor<float,float,float>())
                         .def(luabind::constructor<const real_physics::Vector3&>())
                         // method
                         .def("lenght"     , &utility_engine::Vector3::length)
                         .def("lenght2"    , &utility_engine::Vector3::lengthSquared)
                         .def("normalize"  , &utility_engine::Vector3::normalize)
                         .def("dot"        , &utility_engine::Vector3::dot)
                         .def("cross"      , &utility_engine::Vector3::cross)
                         // operator
                         .def(luabind::const_self  +  utility_engine::Vector3())
                         .def(luabind::const_self  -  utility_engine::Vector3())
                         .def(luabind::const_self  *  utility_engine::Vector3())
                         .def(luabind::const_self  /  utility_engine::Vector3())
                         .def(luabind::const_self ==  utility_engine::Vector3())
                         .def(luabind::const_self  *  float())
                         .def(luabind::const_self  /  float())
                         // value
                         .def_readwrite("x", &utility_engine::Vector3::x)
                         .def_readwrite("y", &utility_engine::Vector3::y)
                         .def_readwrite("z", &utility_engine::Vector3::z));



    importToScope( luabind::class_<utility_engine::Vector4>("vector4")
                         // constructor
                         .def(luabind::constructor<>())
                         .def(luabind::constructor<float,float,float,float>())
                        // method
                         .def("lenght"     , &utility_engine::Vector4::length)
                         .def("lenght2"    , &utility_engine::Vector4::lengthSquared));


    /// Colors
    importToScope( luabind::class_<utility_engine::Color>("color4")
                         // constructor
                         .def(luabind::constructor<float,float,float,float>()));




    /// Matrix4x4
    importToScope( luabind::class_<utility_engine::Matrix4>("matrix4")
                          // constructor
                          .def(luabind::constructor<>())
//                          .def(luabind::constructor<float,float,float,
//                                                    float,float,float,
//                                                    float,float,float>())
                         // method
                         .def("translate" , &utility_engine::Matrix4::translationMatrix)
                         .def("rotate"    , &utility_engine::Matrix4::rotationMatrix)
                         .def("identity"  , &utility_engine::Matrix4::setToIdentity)
                         .def("inverse"   , &utility_engine::Matrix4::getInverse)
                         .def("transpose" , &utility_engine::Matrix4::getTranspose)
                         // operator
                        .def(luabind::const_self  +  utility_engine::Matrix4())
                        .def(luabind::const_self  -  utility_engine::Matrix4())
                        .def(luabind::const_self  *  utility_engine::Matrix4())
                        .def(luabind::const_self  *  utility_engine::Vector3())
                        .def(luabind::const_self  *  utility_engine::Vector4())
                        .def(luabind::const_self  *  float()));



    /// Object3d orintation space
    importToScope(  luabind::class_<utility_engine::Object3D>("object3D")
                          // constructor
                          .def(luabind::constructor<>())
                          // method
                          .def("identity"  , &utility_engine::Object3D::setToIdentity)
                          .def("setMatrix" , &utility_engine::Object3D::setTransformMatrix)
                          .def("getMatrix" , &utility_engine::Object3D::getTransformMatrix)
                          .def("translate" , &utility_engine::Object3D::translateWorld)
                          .def("rotate"    , &utility_engine::Object3D::rotateWorld)
                          .def("origin"    , &utility_engine::Object3D::getOrigin)
                          .def("position"  , &utility_engine::Object3D::getOrigin));


    /// Object camera eya look
    //importToScope(  luabind::class_<utility_engine::CCameraEya , luabind::bases< utility_engine::Object3D>>("camera") );

    /// Camera Look
    importToScope(  luabind::class_< utility_engine::Camera , luabind::bases< utility_engine::Object3D >>("camera")
                          .def(luabind::constructor<>())
                          // method
                          .def("lookAt"     ,  &utility_engine::Camera::LookAt)
                          .def("project"    ,  &utility_engine::Camera::ProjectionMatrix)
                          .def("project"    ,  (utility_engine::Matrix4(utility_engine::Camera::*)())&utility_engine::Camera::getProjectionMatrix)
                          .def("modelView"  ,  (utility_engine::Matrix4(utility_engine::Camera::*)())&utility_engine::Camera::getViewMatrix));



    /// Mesh model
    importToScope(  luabind::class_<utility_engine::Mesh , luabind::bases< utility_engine::Object3D>>("mesh")
                          .def("draw"     , &utility_engine::Mesh::Draw)
                          .def("draw"     , &utility_engine::Mesh::DrawOpenGL)
                          .def("texture"  , &utility_engine::Mesh::setTexture)
                          .def("vColor"   , &utility_engine::Mesh::setColor)
                          .def("vColor"   , &utility_engine::Mesh::setColorToAllVertices));


    /// mesh model plane
    importToScope(  luabind::class_<utility_engine::MeshPlane , luabind::bases< utility_engine::Mesh , utility_engine::Object3D>>("mesh_plane")
                          // constructor
                          .def( luabind::constructor<float,float>() ) );

    /// mesh model box
    importToScope(  luabind::class_<utility_engine::MeshBox , luabind::bases< utility_engine::Mesh , utility_engine::Object3D>>("mesh_box")
                          // constructor
                          .def( luabind::constructor<const utility_engine::Vector3>())
                          .def("halfSize" , &utility_engine::MeshBox::halfSize));


    /// mesh model 3DS-file (3DS-MAX)
    importToScope(  luabind::class_<utility_engine::MeshReadFile3DS , luabind::bases< utility_engine::Mesh , utility_engine::Object3D>>("mesh_3ds")
                          // constructor
                          .def( luabind::constructor<const char*>() ) );




    /// Texture 2D geometry
    importToScope(  luabind::class_<utility_engine::Texture2D>("texture2D")
                         .def(luabind::constructor<>()) );



    //-----------------------------  UI-Engine : Physics ------------------------------------//

     importToScope(  luabind::class_<utility_engine::DynamicsWorld>("dynamics_world")
                           .def(luabind::constructor<const utility_engine::Vector3&>())
                           .def( "RigidBody" , &utility_engine::DynamicsWorld::createRigidBody )
                           .def( "Joint"     , &utility_engine::DynamicsWorld::createJoint )
                           .def( "destroy"   , &utility_engine::DynamicsWorld::destroyBody )
                           .def( "destroy"   , &utility_engine::DynamicsWorld::destroyJoint )
                           .def( "destroy"   , &utility_engine::DynamicsWorld::destroy )
                           .def( "update"    , &utility_engine::DynamicsWorld::update ));



    importToScope(  luabind::class_<utility_engine::GroupMesh>("group_mesh")
                          // constructor
                          .def( luabind::constructor<>())
                          .def("add", (void(utility_engine::GroupMesh::*)(utility_engine::Mesh* , const utility_engine::Matrix4& ))  &utility_engine::GroupMesh::addInitMesh));


    importToScope(  luabind::class_<utility_engine::UltimatePhysicsBody>("ultimate_physics")
                          // constructor
                          .def( luabind::constructor<real_physics::rpPhysicsBody*>())
                          .def("type"        ,   &utility_engine::UltimatePhysicsBody::setType)
                          .def("addCollide"  ,   &utility_engine::UltimatePhysicsBody::addCollisionGeometry)
                          .def("addHull"     ,   (void(utility_engine::UltimatePhysicsBody::*)(utility_engine::Mesh* , float)) &utility_engine::UltimatePhysicsBody::addCollisionGeometry_ConvexHull)
                          .def("addHull"     ,   (void(utility_engine::UltimatePhysicsBody::*)(utility_engine::Mesh* , const utility_engine::Matrix4& , float)) &utility_engine::UltimatePhysicsBody::addCollisionGeometry_ConvexHull)
                          .def("addSphere"   ,   (void(utility_engine::UltimatePhysicsBody::*)(utility_engine::Mesh* , float , float )) &utility_engine::UltimatePhysicsBody::addCollisionGeometry_Sphere)
                          .def("addSphere"   ,   (void(utility_engine::UltimatePhysicsBody::*)(utility_engine::Mesh* , const utility_engine::Matrix4& , float , float )) &utility_engine::UltimatePhysicsBody::addCollisionGeometry_Sphere)
                          .def("addBox"      ,   (void(utility_engine::UltimatePhysicsBody::*)(utility_engine::Mesh* , const utility_engine::Vector3& , float )) &utility_engine::UltimatePhysicsBody::addCollisionGeometry_Box)
                          .def("addBox"      ,   (void(utility_engine::UltimatePhysicsBody::*)(utility_engine::Mesh* , const utility_engine::Matrix4& , const utility_engine::Vector3& , float )) &utility_engine::UltimatePhysicsBody::addCollisionGeometry_Box)

                          .def("applyImpuls"        ,   &utility_engine::UltimatePhysicsBody::applyImpulse)
                          .def("applyImpulsAngular" ,   &utility_engine::UltimatePhysicsBody::applyImpulseAngular)
                          .def("applyImpulsLinear"  ,   &utility_engine::UltimatePhysicsBody::applyImpulseLinear)
                          .def("applyForceToCenter" ,   &utility_engine::UltimatePhysicsBody::applyForceToCenterOfMass)
                          .def("applyForce"         ,   &utility_engine::UltimatePhysicsBody::applyForce)
                          .def("applyTorque"        ,   &utility_engine::UltimatePhysicsBody::applyTorque)

                          .def("update"     ,   &utility_engine::UltimatePhysicsBody::update)

                          .enum_("BodyType")
                          [
                            luabind::value("static"    , utility_engine::UltimatePhysicsBody::BodyType::STATIC    ),
                            luabind::value("dynamic"   , utility_engine::UltimatePhysicsBody::BodyType::DYNAMIC   ),
                            luabind::value("kinematic" , utility_engine::UltimatePhysicsBody::BodyType::KINEMATIC )
                          ]);




    ///---------------------------------- Ultimatium joint function ----------------------------------------------///
    /// \brief importToScope
    ///
    importToScope(  luabind::def( "DistanceJointInfo"      , &utility_engine::UltimateJointInfo::DistanceJointInfo) );
    importToScope(  luabind::def( "BallAndSocketJointInfo" , &utility_engine::UltimateJointInfo::BallAndSocketJointInfo) );
    importToScope(  luabind::def( "FixedJointInfo"         , &utility_engine::UltimateJointInfo::FixedJointInfo) );
    importToScope(  luabind::def( "HingeJointInfo"         , (real_physics::rpHingeJointInfo(*)( utility_engine::UltimatePhysicsBody* ,
                                                                                                       utility_engine::UltimatePhysicsBody* ,
                                                                                                       const utility_engine::Vector3&,
                                                                                                       const utility_engine::Vector3&))&utility_engine::UltimateJointInfo::HingeJointInfo) );
   // importToScope(  luabind::def( "SliderJointInfo"        , &utility_engine::UltimateJointInfo::SliderJointInfo) );
}

