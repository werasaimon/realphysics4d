#include "lua_integration.h"


#include "../physics-engine/physics.h"
#include "../UI-engine/engine.h"


lua_integration::lua_integration()
: mVirtualMashinLua(NULL)
{

}


void lua_integration::initialization()
{

    mVirtualMashinLua = luaL_newstate();

    luaL_openlibs(mVirtualMashinLua);

    luaopen_base(mVirtualMashinLua);
    luaopen_string(mVirtualMashinLua);
    luaopen_table(mVirtualMashinLua);
    luaopen_math(mVirtualMashinLua);
    luaopen_debug(mVirtualMashinLua);
    luaopen_io(mVirtualMashinLua);

    luabind::open(mVirtualMashinLua);

}


void lua_integration::closet()
{
    lua_close(mVirtualMashinLua);
}

void lua_integration::parserClassesLibrary()
{


     //---------------------------------------- Physics-Engine -------------------------------------------------------------//

    importToMethodScope( luabind::class_<bool>("ubool"));
    importToMethodScope( luabind::class_<int>("uint"));
    importToMethodScope( luabind::class_<float>("ufloat"));


    /// Vector3D         
    importToMethodScope(luabind::namespace_("physics")
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
    importToMethodScope(luabind::namespace_("physics")
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
    importToMethodScope(luabind::namespace_("physics")
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
    importToMethodScope( luabind::namespace_("physics")
                         [
                            luabind::class_<real_physics::Transform>("transform")
                            // constructor
                           .def(luabind::constructor<const real_physics::Vector3& ,const real_physics::Quaternion&>())
                         ]);



    //---------------------------------------- Physics-Engine -------------------------------------------------------------//


    /// Collision shape virtual
    importToMethodScope(  luabind::class_<real_physics::rpCollisionShape>("shape"));



    /// Collision shape box
    importToMethodScope(  luabind::class_< real_physics::rpBoxShape ,  luabind::bases<real_physics::rpCollisionShape> >("shape_box")
                          // constructor
                          .def(luabind::constructor<const real_physics::Vector3&>())
                          .def(luabind::constructor<const real_physics::Vector3& , real_physics::scalar>()));



    /// Collsion shape shpere
    importToMethodScope( luabind::class_< real_physics::rpSphereShape ,  luabind::bases<real_physics::rpCollisionShape> >("shape-sphere")
                         // constructor
                         .def(luabind::constructor<real_physics::scalar>()));


    /// Convex-Hull Geometry
    importToMethodScope(  luabind::class_<real_physics::rpModelConvexHull>("hull")
                          // constructor
                         .def(luabind::constructor<const real_physics::Vector3*, real_physics::uint>()));


    /// Collsion shape convex-hull
    importToMethodScope(  luabind::class_<real_physics::rpConvexHullShape , luabind::bases<real_physics::rpCollisionShape>>("shape-hull")
                          // constructor
                         .def(luabind::constructor<real_physics::rpModelConvexHull*>()) );



    /// Meneger systems collisions
    importToMethodScope(  luabind::class_<real_physics::rpCollisionDetection>("collid-meneger")
                          // constructor
                         .def(luabind::constructor<>()) );


    ///Abstract on body
    importToMethodScope(  luabind::class_<real_physics::rpBody>("body")
                          // constructor
                          .def(luabind::constructor<real_physics::bodyindex>()) );

    ///Collision on body
    importToMethodScope(  luabind::class_<real_physics::rpCollisionBody ,luabind::bases<real_physics::rpBody>>("collid-body")
                          // constructor
                          .def(luabind::constructor<const real_physics::Transform& , real_physics::rpCollisionDetection* , real_physics::bodyindex>())
                          .enum_("BodyType")
                          [
                            luabind::value("static"    ,real_physics::BodyType::STATIC),
                            luabind::value("dynamic"   ,real_physics::BodyType::DYNAMIC),
                            luabind::value("kinematic" ,real_physics::BodyType::KINEMATIC)
                          ]);



    importToMethodScope(  luabind::class_<real_physics::rpCollisionWorld>("collisions-world")
                          // constructor
                          .def(luabind::constructor<>()) );

    //---------------------------------------- Physics-Engine -------------------------------------------------------------//



    ///Physics material
    importToMethodScope(  luabind::class_<real_physics::rpPhysicsMaterial>("material")
                          // constructor
                          .def(luabind::constructor<>()) );


    ///Physics on object
    importToMethodScope(  luabind::class_<real_physics::rpPhysicsObject , luabind::bases<real_physics::rpCollisionBody> >("physics-object")
                          // constructor
                          .def(luabind::constructor<const real_physics::Transform& , real_physics::rpCollisionDetection* , real_physics::bodyindex>())
                          .enum_("BodyType")
                          [
                            luabind::value("static"    ,real_physics::BodyType::STATIC),
                            luabind::value("dynamic"   ,real_physics::BodyType::DYNAMIC),
                            luabind::value("kinematic" ,real_physics::BodyType::KINEMATIC)
                          ]);



    ///Physics on body
    importToMethodScope(  luabind::class_<real_physics::rpPhysicsBody , luabind::bases<real_physics::rpPhysicsObject> >("physics-body")
                          // constructor
                          .def(luabind::constructor<const real_physics::Transform& , real_physics::rpCollisionDetection* , real_physics::bodyindex>()) );


    ///Physics on rigid-body
    importToMethodScope(  luabind::class_<real_physics::rpRigidPhysicsBody , luabind::bases<real_physics::rpPhysicsBody> >("rigid-body")
                          // constructor
                          .def(luabind::constructor<const real_physics::Transform& , real_physics::rpCollisionDetection* , real_physics::bodyindex>()) );





    importToMethodScope(  luabind::class_<real_physics::rpJointInfo>("joint-info")
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
                          ]);



    importToMethodScope(  luabind::class_<real_physics::rpDistanceJointInfo , luabind::bases<real_physics::rpJointInfo> >("dist-info")
                          // constructor
                          .def(luabind::constructor<real_physics::rpPhysicsBody* , real_physics::rpPhysicsBody* , const real_physics::scalar&>()) );



    importToMethodScope(  luabind::class_<real_physics::rpBallAndSocketJointInfo , luabind::bases<real_physics::rpJointInfo> >("ball-info")
                          // constructor
                          .def(luabind::constructor<real_physics::rpPhysicsBody* , real_physics::rpPhysicsBody* , const real_physics::Vector3&>()) );




    importToMethodScope(  luabind::class_<real_physics::rpJoint>("joint")
                          // constructor
                          .def(luabind::constructor<const real_physics::rpJointInfo&>()) );





    importToMethodScope(  luabind::class_<real_physics::rpDistanceJoint , luabind::bases<real_physics::rpJoint> >("dist-joint")
                          // constructor
                          .def(luabind::constructor<const real_physics::rpDistanceJointInfo&>()) );


    importToMethodScope(  luabind::class_<real_physics::rpBallAndSocketJoint , luabind::bases<real_physics::rpJoint> >("ball-joint")
                          // constructor
                          .def(luabind::constructor<const real_physics::rpBallAndSocketJointInfo&>()) );



    importToMethodScope(  luabind::class_< real_physics::rpDynamicsWorld ,luabind::bases<real_physics::rpCollisionWorld>>("dynamics-world")
                          // constructor
                          .def(luabind::constructor<const real_physics::Vector3&>()) );




    //--------------------------------- UI-Engine -------------------------------------//


    importToMethodScope( luabind::class_<utility_engine::Vector2>("vector2")
                         // constructor
                         .def(luabind::constructor<>())
                         .def(luabind::constructor<float,float>())
                         // method
                         .def("lenght"     , &utility_engine::Vector2::length)
                         .def("lenght2"    , &utility_engine::Vector2::lengthSquared)
                         .def("normalize"  , &utility_engine::Vector2::normalize));



    importToMethodScope( luabind::class_<utility_engine::Vector3>("vector3")
                         // constructor
                         .def(luabind::constructor<>())
                         .def(luabind::constructor<float,float,float>())
                         .def(luabind::constructor<const real_physics::Vector3&>())
                         // method
                         .def("lenght"     , &utility_engine::Vector3::length)
                         .def("lenght2"    , &utility_engine::Vector3::lengthSquared)
                         .def("dot"        , &utility_engine::Vector3::dot)
                         .def("cross"      , &utility_engine::Vector3::cross)
                         .def("normalize"  , &utility_engine::Vector3::normalize)
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


    /// Matrix4x4
    importToMethodScope( luabind::class_<utility_engine::Matrix4>("matrix4")
                          // constructor
                          .def(luabind::constructor<>())
                          .def(luabind::constructor<float,float,float,
                                                    float,float,float,
                                                    float,float,float>())
                         // method
                         .def("identity" , &utility_engine::Matrix4::setToIdentity)
                         .def("translate" , &utility_engine::Matrix4::translationMatrix)
                         // operator
                        .def(luabind::const_self  +  utility_engine::Matrix4())
                        .def(luabind::const_self  -  utility_engine::Matrix4()));




    /// Object3d orintation space
    importToMethodScope(  luabind::class_<utility_engine::Object3D>("object3D")
                          // constructor
                          .def(luabind::constructor<>())
                          // method
                          .def("identity"  , &utility_engine::Object3D::setToIdentity)
                          .def("matrix"    , &utility_engine::Object3D::setTransformMatrix)
                          .def("matrix"    , &utility_engine::Object3D::getTransformMatrix)
                          .def("translate" , &utility_engine::Object3D::translateWorld)
                          .def("rotate"    , &utility_engine::Object3D::rotateWorld));


    /// Object camera eya look
    //importToMethodScope(  luabind::class_<utility_engine::CCameraEya , luabind::bases< utility_engine::Object3D>>("camera") );

    /// Camera Look
    importToMethodScope(  luabind::class_< utility_engine::Camera , luabind::bases< utility_engine::Object3D >>("camera")
                          .def(luabind::constructor<>())
                          // method
                          .def("lookAt"     ,  &utility_engine::Camera::LookAt)
                          .def("project"    ,  &utility_engine::Camera::ProjectionMatrix)
                          .def("project"    ,  (utility_engine::Matrix4(utility_engine::Camera::*)())&utility_engine::Camera::getProjectionMatrix)
                          .def("modelView"  ,  (utility_engine::Matrix4(utility_engine::Camera::*)())&utility_engine::Camera::getViewMatrix));



    /// Mesh model
    importToMethodScope(  luabind::class_<utility_engine::Mesh , luabind::bases< utility_engine::Object3D>>("mesh")
                          .def("draw"   , &utility_engine::Mesh::Draw)
                          .def("draw"   , &utility_engine::Mesh::DrawOpenGL)
                          .def("texture", &utility_engine::Mesh::setTexture));


    /// mesh model 3DS-file (3DS-MAX)
    importToMethodScope(  luabind::class_<utility_engine::MeshReadFile3DS , luabind::bases< utility_engine::Mesh , utility_engine::Object3D>>("mesh_3ds")
                          // constructor
                          .def( luabind::constructor<const char*>() ) );



    /// mesh model box
    importToMethodScope(  luabind::class_<utility_engine::MeshBox , luabind::bases< utility_engine::Mesh , utility_engine::Object3D>>("mesh_box")
                          // constructor
                          .def( luabind::constructor<const utility_engine::Vector3>() ) );



    /// mesh model plane
    importToMethodScope(  luabind::class_<utility_engine::MeshPlane , luabind::bases< utility_engine::Mesh , utility_engine::Object3D>>("mesh_plane")
                          // constructor
                          .def( luabind::constructor<float,float>() ) );



    /// Texture 2D geometry
    importToMethodScope(  luabind::class_<utility_engine::Texture2D>("texture2D")
                         .def(luabind::constructor<>()) );



    //-----------------------------  UI-Engine : Physics ------------------------------------//

     importToMethodScope(  luabind::class_<utility_engine::DynamicsWorld>("dynamicsWorld")
                           .def(luabind::constructor<const utility_engine::Vector3&>())
                           .def( "RigidBody" , &utility_engine::DynamicsWorld::createRigidBody )
                           .def( "Joint"     , &utility_engine::DynamicsWorld::createJoint )
                           .def( "destroy"   , &utility_engine::DynamicsWorld::destroyBody )
                           .def( "destroy"   , &utility_engine::DynamicsWorld::destroyJoint )
                           .def( "destroy"   , &utility_engine::DynamicsWorld::destroy )
                           .def( "update"    , &utility_engine::DynamicsWorld::update ));



    importToMethodScope(  luabind::class_<utility_engine::GroupMesh>("group_mesh")
                          // constructor
                          .def( luabind::constructor<>())
                          .def("add", (void(utility_engine::GroupMesh::*)(utility_engine::Mesh* , const utility_engine::Matrix4& ))  &utility_engine::GroupMesh::addInitMesh));


    importToMethodScope(  luabind::class_<utility_engine::UltimatePhysicsBody>("ultimate_physics")
                          // constructor
                          .def( luabind::constructor<real_physics::rpPhysicsBody*>())
                          .def("type"      ,   &utility_engine::UltimatePhysicsBody::setType)
                          .def("addCollide" ,   &utility_engine::UltimatePhysicsBody::addCollisionGeometry)
                          .def("addHull"   ,   (void(utility_engine::UltimatePhysicsBody::*)(utility_engine::Mesh* , float)) &utility_engine::UltimatePhysicsBody::addCollisionGeometry_ConvexHull)
                          .def("addHull"   ,   (void(utility_engine::UltimatePhysicsBody::*)(utility_engine::Mesh* , const utility_engine::Matrix4& , float)) &utility_engine::UltimatePhysicsBody::addCollisionGeometry_ConvexHull)
                          .def("update"    ,   &utility_engine::UltimatePhysicsBody::update)
                          .enum_("BodyType")
                          [
                            luabind::value("static"    , utility_engine::UltimatePhysicsBody::BodyType::STATIC    ),
                            luabind::value("dynamic"   , utility_engine::UltimatePhysicsBody::BodyType::DYNAMIC   ),
                            luabind::value("kinematic" , utility_engine::UltimatePhysicsBody::BodyType::KINEMATIC )
                          ]);




    ///---------------------------------- Ultimatium joint function ----------------------------------------------///
    /// \brief importToMethodScope
    ///
    importToMethodScope(  luabind::def( "DistanceJointInfo"      , &utility_engine::UltimateJointInfo::DistanceJointInfo) );
    importToMethodScope(  luabind::def( "BallAndSocketJointInfo" , &utility_engine::UltimateJointInfo::BallAndSocketJointInfo) );
    importToMethodScope(  luabind::def( "FixedJointInfo"         , &utility_engine::UltimateJointInfo::FixedJointInfo) );
    importToMethodScope(  luabind::def( "HingeJointInfo"         , (real_physics::rpHingeJointInfo(*)( utility_engine::UltimatePhysicsBody* ,
                                                                                                       utility_engine::UltimatePhysicsBody* ,
                                                                                                       const utility_engine::Vector3&,
                                                                                                       const utility_engine::Vector3&))&utility_engine::UltimateJointInfo::HingeJointInfo) );
   // importToMethodScope(  luabind::def( "SliderJointInfo"        , &utility_engine::UltimateJointInfo::SliderJointInfo) );




    ///------------------------------------- shader-Qt --------------------------------------///

    ///Qt shader
    importToMethodScope(  luabind::class_<QOpenGLShaderProgram>("qshaderProgram")
                          .def(luabind::constructor<>())
                          .def("link"    , &QOpenGLShaderProgram::link)
                          .def("bind"    , &QOpenGLShaderProgram::bind)
                          .def("release" , &QOpenGLShaderProgram::release));


    ///Program shader
    importToMethodScope(  luabind::class_<utility_engine::GLShaderProgram , luabind::bases<QOpenGLShaderProgram> >("shaderProgram")
                          .def(luabind::constructor<>())
                          .def("addSourceFile" , &utility_engine::GLShaderProgram::addSourceFile)
                          .def("UniformValue"  , (void(utility_engine::GLShaderProgram::*)(const char* , int))&utility_engine::GLShaderProgram::UniformValue)
                          .def("UniformValue"  , (void(utility_engine::GLShaderProgram::*)(const char* , const float&))&utility_engine::GLShaderProgram::UniformValue)
                          .def("UniformValue"  , (void(utility_engine::GLShaderProgram::*)(const char* , const utility_engine::Vector3& ))&utility_engine::GLShaderProgram::UniformValue)
                          .def("UniformValue"  , (void(utility_engine::GLShaderProgram::*)(const char* , const utility_engine::Matrix4& ))&utility_engine::GLShaderProgram::UniformValue)
                          .enum_("ShaderType")
                          [
                             luabind::value("vertex"     , utility_engine::GLShaderProgram::ShaderType::Vertex   ),
                             luabind::value("fragment"   , utility_engine::GLShaderProgram::ShaderType::Fragment ),
                             luabind::value("geometry"   , utility_engine::GLShaderProgram::ShaderType::Geometry )
                          ]);



}




void lua_integration::importToMethodScope(luabind::scope _importValue)
{
  luabind::module(mVirtualMashinLua)[_importValue];
}

/// Run script-lua
void lua_integration::runString(const char *_str)
{
   luaL_dostring(mVirtualMashinLua , _str);
}

/// Run file-lua
void lua_integration::runFile(const char *_fileName)
{
   luaL_dofile(mVirtualMashinLua, _fileName);
}

///virtual mashine-lua
lua_State *lua_integration::getVirtualMashinLua() const
{
    return mVirtualMashinLua;
}
