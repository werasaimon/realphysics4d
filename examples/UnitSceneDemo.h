#ifndef UNITSCENEDEMO_H
#define UNITSCENEDEMO_H


#include "UnitScene.h"
#include "engine/UI-engine/engine.h"

#include <QFile>
#include <QVector>

namespace
{

    bool CopyFileFromToResources( QFile &mFile, const char* fileName )
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


//    real_physics::rpJointInfo BallJointt( utility_engine::UltimatePhysicsBody* _physics1 , utility_engine::UltimatePhysicsBody *_physics2 , utility_engine::Vector3 anchor )
//    {
//        real_physics::rpPhysicsBody *phys1;
//        real_physics::rpPhysicsBody *phys2;

//        return  real_physics::rpBallAndSocketJointInfo( phys1 , phys2 , anchor );
//    }


}

using namespace engine;


class UnitSceneDemo : public UnitScene
{
   private:



    //-------------------- Attribute --------------------//

     /// Shaders shines
     GLShaderProgram    mProgramShader;

     /// Array mesh-geometry
     std::vector<Mesh*> mMeshes;

     /// Look camera
     Camera mCamera;

     /// camera value
     Vector3 mEye;
     Vector3 mCenter;
     Vector3 mUp;


     /// width and height size window
     float width;
     float height;


     ///----------------- Physics -----------------///
     DynamicsWorld                    *mDynamicsWorld;
     std::vector<UltimatePhysicsBody*> mPhysicsBodies;


     ///------- time step real-time ---------------///
     float mTimeStep;




     ///-------------------------///
      bool initShader();
      bool initGeometry();
      bool initPhysics();
     ///-------------------------///

   public:

     UnitSceneDemo();


     bool initialization();
     void render(float FrameTime);
     void update();
     void resize( float w , float h );
     void mouseMove(float x , float y , int button);
     void mousePress( float x , float y , int button );
     void mouseReleasePress( float x , float y , int button );
     void mouseWheel( float delta );
     void keyboard(int key );
     void destroy();

};

#endif // UNITSCENEDEMO_H
