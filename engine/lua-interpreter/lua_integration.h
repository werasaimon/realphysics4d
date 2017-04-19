#ifndef LUA_INTEGRATION_H
#define LUA_INTEGRATION_H

extern "C"
{
    #include "lua/lua.h"
    #include "lua/lualib.h"
    #include "lua/lauxlib.h"
}

#include "luabind/luabind.hpp"
#include "luabind/operator.hpp"
#include "luabind/class.hpp"
#include "luabind/function.hpp"


class lua_integration
{

    public:

        lua_integration();

        void initialization();
        void closet();
        void parserClassesLibrary();


        void importToMethodScope(luabind::scope _importValue);

        void runString(const char *_str);
        void runFile(const char *_fileName);

        lua_State *getVirtualMashinLua() const;

private:

        lua_State *mVirtualMashinLua;

};

#endif // LUA_INTEGRATION_H
