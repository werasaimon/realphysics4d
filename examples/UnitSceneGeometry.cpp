#include "UnitSceneGeometry.h"
#include <QFile>

namespace
{

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



    float oldX = 0;
    float oldY = 0;

}

//---------------------------------------//

UnitSceneGeometry::UnitSceneGeometry()
{

}



void UnitSceneGeometry::initGeometry()
{
    width  = 600;
    height = 400;

    float aspect = width / height;
    float zNear  = 3.0;
    float zFar   = 1024;
    float fov    = 45.0;

    mCamera.ProjectionMatrix( fov , aspect , zNear , zFar );

    mEye    =  Vector3(0,0,-40);
    mCenter =  Vector3(0,0,0);
    mUp     =  Vector3(0,1,0);


    //---------------------------- loading texture --------------------------------//
    // Load cube.png image
    QOpenGLTexture *texture = new QOpenGLTexture(QImage(":/Files/cube.jpg").mirrored());

    // Set nearest filtering mode for texture minification
    texture->setMinificationFilter(QOpenGLTexture::Nearest);

    // Set bilinear filtering mode for texture magnification
    texture->setMagnificationFilter(QOpenGLTexture::Linear);

    // Wrap texture coordinates by repeating
    // f.ex. texture coordinate (1.1, 1.2) is same as (0.1, 0.2)
    texture->setWrapMode(QOpenGLTexture::Repeat);


     Texture2D   texture2D(texture->textureId());
    //-----------------------------------------------------------------------------//



     /**/
     /// Add mesh-model in array meshes
     const char fileName2[] = "plane.3DS";
     QFile mFile2(fileName2);
     CopyFileResources( mFile2 , ":/Files/plane.3DS" );

     for( int i =0 ; i < 10; ++i )
     {

         Mesh *meshModel = new MeshBox( Vector3(4,4,4) );

         Vector3 position =   Vector3::Y * i * 12.0 + Vector3::X * 1 * 2.5;
         //position += Vector3::Z * -0.0;
         //position += Vector3::X * -0.0;

         //meshModel->setToIdentity();
         meshModel->translateWorld( position );
        // meshModel->setTexture(texture2D);
         meshModel->setColorToAllVertices(Color(1,0,0,1));

         mMeshes.push_back(meshModel);
     }
     /**/


     mFile2.remove();

}


//---------------------------------------//

bool UnitSceneGeometry::initialization()
{
    initGeometry();
}



void UnitSceneGeometry::render(float FrameTime)
{

    //------------------ Render --------------------//

     glViewport(0, 0, width , height );

     mCamera.LookAt( mEye , mCenter  , mUp );

     glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
     glLoadIdentity();


     glMatrixMode(GL_PROJECTION);
     glLoadMatrixf(mCamera.getProjectionMatrix().getTranspose().dataBlock());

     glMatrixMode(GL_MODELVIEW);
     glLoadMatrixf(mCamera.getViewMatrix().getTranspose().dataBlock());


     ///------------ draw ---------------///

     for(unsigned int i=0; i < mMeshes.size(); ++i)
     {
            mMeshes[i]->Draw();
         // mMeshes[i]->DrawOpenGL(&mProgramShader);
     }




}

void UnitSceneGeometry::update()
{

}

void UnitSceneGeometry::resize(float w, float h)
{
    width  = w;
    height = h;

    float aspect = width / height;
    float zNear  = 3.0;
    float zFar   = 1024;
    float fov    = 45.0;

    mCamera.ProjectionMatrix( fov , aspect , zNear , zFar );
}


void UnitSceneGeometry::mouseMove(float x, float y)
{


    Matrix4 M;
    M.setToIdentity();
    M = Matrix4::rotationMatrix( Vector3::Y , (x - oldX) * 0.01 ) * M;
    M = Matrix4::rotationMatrix( Vector3::X , (y - oldY) * 0.01 ) * M;

    mEye = M * mEye;

    oldX = x;
    oldY = y;

}

void UnitSceneGeometry::mousePress(float x, float y)
{
    oldX = x;
    oldY = y;
}

void UnitSceneGeometry::mouseWheel(float delta)
{
    mEye += (Vector3(0,0,0) - mEye).normalize() * delta * 0.095;
    delta = 0;
}

void UnitSceneGeometry::keyboard(int key)
{

}

void UnitSceneGeometry::destroy()
{

}
