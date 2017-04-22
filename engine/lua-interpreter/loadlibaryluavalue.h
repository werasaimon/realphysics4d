#ifndef LOADLIBARYLUAVALUE_H
#define LOADLIBARYLUAVALUE_H

#include "loadlibarylua.h"


class LoadLibaryLuaValue : public LoadLibaryLua
{

  public:

      LoadLibaryLuaValue( lua_State *_VirtualMashinLua );


    //------------ Method ------------//

      void LoadLibary();
};

#endif // LOADLIBARYLUAVALUE_H
