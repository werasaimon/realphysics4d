#ifndef UNITSCENELUAINTERPRETATIONSDK_H
#define UNITSCENELUAINTERPRETATIONSDK_H


#include "utilopengl.h"

#include "UnitScene.h"
#include "engine/lua-interpreter/lua_integration.h"
#include "engine/UI-engine/engine.h"

#include <QFile>


using namespace engine;

namespace
{
  struct mouse
  {
      mouse(float x , float y)
      : m_x(x) , m_y(y)
      {

      }

      float m_y;
      float m_x;
  };



  bool CopyFileResources( QFile &mFile, const char* fileName )
  {
      QFile FileRead(fileName);

      bool success = true;
      success &=  FileRead.open( QFile::ReadOnly );
      success &= mFile.open( QFile::WriteOnly | QFile::Truncate );
      success &= mFile.write(  FileRead.readAll() ) >= 0;
      FileRead.close();
      mFile.close();

      return success;
  }

  ///--------------- GL_Function ------------------///

  Texture2D loadTexture(const char* _filename)
  {
      // Load cube.png image
      QOpenGLTexture *texture = new QOpenGLTexture(QImage(_filename).mirrored());

      // Set nearest filtering mode for texture minification
      texture->setMinificationFilter(QOpenGLTexture::Nearest);

      // Set bilinear filtering mode for texture magnification
      texture->setMagnificationFilter(QOpenGLTexture::Linear);

      // Wrap texture coordinates by repeating
      // f.ex. texture coordinate (1.1, 1.2) is same as (0.1, 0.2)
      texture->setWrapMode(QOpenGLTexture::Repeat);

      // Reaturn texture2D
      return Texture2D(texture->textureId());
  }
}

class UnitSceneLuaInterpretationSDK : public UnitScene
{

    private:

     //-------------------- Attribute --------------------//


    /// Lua interptation mashine
    lua_integration mLuaMashine;

    /// Resize window
    float width;
    float height;

    /// Initilization lua-interpritator
    bool initLua();
    bool initGL();

    public:

      UnitSceneLuaInterpretationSDK();


      bool initialization();

      void render(float FrameTime);
      void update();
      void resize( float w , float h );

      void mouseMove( float x , float y);
      void mousePress( float x , float y);
      void mouseWheel( float delta );

      void keyboard(int key );
      void destroy();

      //---------------------------------------------------//

      void runScriptLua( const char* _str );

};

#endif // UNITSCENELUAINTERPRETATIONSDK_H
