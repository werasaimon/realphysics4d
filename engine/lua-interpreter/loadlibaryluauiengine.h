#ifndef LOADLIBARYLUAUIENGINE_H
#define LOADLIBARYLUAUIENGINE_H

#include "loadlibarylua.h"

class LoadLibaryLuaUIEngine : public LoadLibaryLua
{
   public:

      LoadLibaryLuaUIEngine( lua_State *_VirtualMashinLua );


      //------------ Method ------------//
       virtual void LoadLibary();
};

#endif // LOADLIBARYLUAUIENGINE_H
