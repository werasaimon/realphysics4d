#ifndef LOADLIBARYLUAOPENGL_H
#define LOADLIBARYLUAOPENGL_H

#include "loadlibarylua.h"

class LoadLibaryLuaOpenGL : public LoadLibaryLua
{
   public:

     LoadLibaryLuaOpenGL( lua_State *_VirtualMashinLua );

     //------------ Method ------------//
      virtual void LoadLibary();
};

#endif // LOADLIBARYLUAOPENGL_H
