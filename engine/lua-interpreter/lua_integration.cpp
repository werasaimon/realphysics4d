#include "lua_integration.h"


#include "../physics-engine/physics.h"
#include "../UI-engine/engine.h"


lua_integration::lua_integration()
: mVirtualMashinLua(NULL)
{

}

///********************************///
/// \brief lua_integration::initialization
///Load libary to scope
///
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

/// Closet lua-mashine
void lua_integration::closet()
{
    lua_close(mVirtualMashinLua);
}

/// Import to scope in lua-mashine
void lua_integration::importToScope(luabind::scope _importValue)
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

