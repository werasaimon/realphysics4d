#include "loadlibaryluashader.h"
#include "../UI-engine/Shader/Shader.h"

LoadLibaryLuaShader::LoadLibaryLuaShader(lua_State *_VirtualMashinLua)
    : LoadLibaryLua(_VirtualMashinLua)
{
    assert(_VirtualMashinLua);
}

void LoadLibaryLuaShader::LoadLibary()
{

    ///Qt shader
    importToScope(  luabind::class_<QOpenGLShaderProgram>("qshaderProgram")
                          .def(luabind::constructor<>())
                          .def("link"    , &QOpenGLShaderProgram::link)
                          .def("bind"    , &QOpenGLShaderProgram::bind)
                          .def("release" , &QOpenGLShaderProgram::release));


    ///Program shader
    importToScope(  luabind::class_<utility_engine::GLShaderProgram , luabind::bases<QOpenGLShaderProgram> >("shaderProgram")
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

