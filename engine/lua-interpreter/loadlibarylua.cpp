#include "loadlibarylua.h"


LoadLibaryLua::LoadLibaryLua(lua_State *_VirtualMashinLua)
    : mVirtualMashinLua(_VirtualMashinLua)
{

}

void LoadLibaryLua::importToScope(luabind::scope _importValue)
{
    luabind::module(mVirtualMashinLua)[_importValue];
}
