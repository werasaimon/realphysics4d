#include "loadlibaryluaopengl.h"
#include "utilopengl.h"

LoadLibaryLuaOpenGL::LoadLibaryLuaOpenGL(lua_State *_VirtualMashinLua)
 : LoadLibaryLua( _VirtualMashinLua )
{
    assert(mVirtualMashinLua);
}

void LoadLibaryLuaOpenGL::LoadLibary()
{

#ifdef __ANDROID__
#elif defined(WIN32) || defined(__linux__)
    importToScope( luabind::namespace_("GL")[luabind::def( "glProjection" , &UtilOpenGL::glProject_)]);
    importToScope( luabind::namespace_("GL")[luabind::def( "glModelView"  , &UtilOpenGL::glModelView_)]);
#endif
    importToScope( luabind::namespace_("GL")[luabind::def( "glIdentity"   , &UtilOpenGL::glLoadIdentity_)]);
    importToScope( luabind::namespace_("GL")[luabind::def( "glClear"      , &UtilOpenGL::glClear_)]);
    importToScope( luabind::namespace_("GL")[luabind::def( "glViewport"   , &UtilOpenGL::glViewport_) ]);


    luabind::globals(mVirtualMashinLua)["GL_COLOR_BUFFER_BIT"]  = (int)(GL_COLOR_BUFFER_BIT);
    luabind::globals(mVirtualMashinLua)["GL_DEPTH_BUFFER_BIT"]  = (int)(GL_DEPTH_BUFFER_BIT);


}
