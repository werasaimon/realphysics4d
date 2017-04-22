#ifndef LOADLIBARYLUAPHYSICSENGINE_H
#define LOADLIBARYLUAPHYSICSENGINE_H

#include "loadlibarylua.h"

class LoadLibaryLuaPhysicsEngine : public LoadLibaryLua
{

   public:

      LoadLibaryLuaPhysicsEngine( lua_State *_VirtualMashinLua );


      //------------ Method ------------//
      void LoadLibary();
};

#endif // LOADLIBARYLUAPHYSICSENGINE_H
