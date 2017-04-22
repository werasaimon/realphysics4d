#include "loadlibaryluavalue.h"


LoadLibaryLuaValue::LoadLibaryLuaValue( lua_State *_VirtualMashinLua )
    : LoadLibaryLua( _VirtualMashinLua )
{
    assert(_VirtualMashinLua);
}

void LoadLibaryLuaValue::LoadLibary()
{

    importToScope( luabind::class_<bool>("ubool"));
    importToScope( luabind::class_<int>("uint"));
    importToScope( luabind::class_<float>("ufloat"));

}
