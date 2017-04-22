#ifndef LOADLIBARYLUASHADER_H
#define LOADLIBARYLUASHADER_H

#include "loadlibarylua.h"

class LoadLibaryLuaShader : public LoadLibaryLua
{
   public:

      LoadLibaryLuaShader( lua_State *_VirtualMashinLua );


   //------------ Method ------------//
      void LoadLibary();
};

#endif // LOADLIBARYLUASHADER_H
