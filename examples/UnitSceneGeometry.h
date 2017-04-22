#ifndef UNITSCENEGEOMETRY_H
#define UNITSCENEGEOMETRY_H

#include "UnitScene.h"
#include "engine/UI-engine/engine.h"

using namespace engine;

class UnitSceneGeometry : public UnitScene
{
   private:

        /// Look camera
        Camera  mCamera;

        /// camera value
        Vector3 mEye;
        Vector3 mCenter;
        Vector3 mUp;


        int mMouseRightButton;


        /// width and height size window
        float width;
        float height;

        //--------------- array --------------//

        /// Array mesh-geometry
        std::vector<Mesh*> mMeshes;


        void initGeometry();

   public:

      UnitSceneGeometry();



     bool initialization();
     void render(float FrameTime);
     void update();
     void resize(     float w , float h);
     void mouseMove(float x , float y   , int button );
     void mousePress( float x , float y , int button );
     void mouseReleasePress( float x , float y , int button );
     void mouseWheel( float delta );
     void keyboard(int key );
     void destroy();


};

#endif // UNITSCENEGEOMETRY_H
