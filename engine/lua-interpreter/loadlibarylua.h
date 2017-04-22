#ifndef LOADLIBARYLUA_H
#define LOADLIBARYLUA_H

#include "lua_integration.h"
///*****************************///
/// loader SDK to scope
///*****************************///
class LoadLibaryLua
{
   protected:

    //---------- Attribute ----------//
      lua_State *mVirtualMashinLua;

    //------------ Method ------------//
     void importToScope(luabind::scope _importValue);

   public:

     explicit LoadLibaryLua(lua_State *_VirtualMashinLua);

    //------------ Method ------------//
     virtual void LoadLibary() = 0;


};

#endif // LOADLIBARYLUA_H
