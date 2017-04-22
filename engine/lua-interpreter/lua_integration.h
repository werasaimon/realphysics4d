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


    private:

        //---------- Attribute ----------//

        lua_State *mVirtualMashinLua;


    public:



        lua_integration();


       //------------ Method ------------//

        void initialization();
        void closet();

        void importToScope(luabind::scope _importValue);

        void runString(const char *_str);
        void runFile(const char *_fileName);



        lua_State *getVirtualMashinLua() const;



};

#endif // LUA_INTEGRATION_H
