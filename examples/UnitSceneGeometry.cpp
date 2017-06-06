#include "UnitSceneGeometry.h"
#include <QFile>


#include "engine/engine.h"

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


    bool _bb_ = true;

}

//---------------------------------------//

UnitSceneGeometry::UnitSceneGeometry()
{

}



void UnitSceneGeometry::initCamera()
{
    _bb_ = true;

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
}

void UnitSceneGeometry::initGeometry()
{

    world = new DynamicsWorld( Vector3::Y * -30.0 );


    //***********************************************//

    Mesh *model0 = new MeshBox( Vector3(250,4,250) );
    model0->translateWorld( Vector3::Y * -10.0);
    model0->setColorToAllVertices(Color(1,1,1,1));
    mMeshes.push_back(model0);


    UltimatePhysicsBody *body0 = world->createRigidBody( model0->getTransformMatrix() );

    model0->setToIdentity();
    body0->addCollisionGeometry_ConvexHull( model0 , model0->getTransformMatrix() , 20 );
    body0->setType(real_physics::BodyType::STATIC);




    const int NbSize = 50;
    UltimatePhysicsBody *bodies[NbSize];
    for( int i = 0; i < NbSize; ++i)
    {

        //************************************************//
        Mesh *model1 = new MeshBox( Vector3(5,4,5) );
        model1->translateWorld( Vector3::Y * 5.0 * i /*+ Vector3::X * 2.0 * sin(i)*/ );
        model1->setColorToAllVertices(Color(1,0,0,1));
        mMeshes.push_back(model1);

        UltimatePhysicsBody *body1 = world->createRigidBody( model1->getTransformMatrix() );
        model1->setToIdentity();
        body1->addCollisionGeometry_ConvexHull( model1 , model1->getTransformMatrix() , 15 );
        body1->setType(real_physics::BodyType::DYNAMIC);

        bodies[i] = body1;

    }


    for( int i = 1; i < NbSize; ++i )
    {
        const real_physics::Vector3 v1 = bodies[i-1]->getPhysicsBody()->getTransform().getPosition();
        const real_physics::Vector3 v2 = bodies[i-0]->getPhysicsBody()->getTransform().getPosition();
        const real_physics::Vector3 anchor = (v1 + v2) * 0.5;
        real_physics::rpBallAndSocketJointInfo infoJoint( bodies[i-1]->getPhysicsBody() , bodies[i-0]->getPhysicsBody() , anchor );
        UltimateJoint *joint = world->createJoint(infoJoint);
    }


}


//---------------------------------------//

bool UnitSceneGeometry::initialization()
{
    initCamera();
    initGeometry();
}



void UnitSceneGeometry::render(float FrameTime)
{

    //------------------ Render --------------------//

     glViewport(0, 0, width , height );

     mCamera.LookAt( mEye , mCenter  , mUp );

     glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
     //glLoadIdentity();


//     glMatrixMode(GL_PROJECTION);
//     glLoadMatrixf(mCamera.getProjectionMatrix().getTranspose().dataBlock());

//     glMatrixMode(GL_MODELVIEW);
//     glLoadMatrixf(mCamera.getViewMatrix().getTranspose().dataBlock());


//     ///------------ draw ---------------///

//     for(unsigned int i=0; i < mMeshes.size(); ++i)
//     {
//            mMeshes[i]->Draw();
//         // mMeshes[i]->DrawOpenGL(&mProgramShader);
//     }





}

void UnitSceneGeometry::update()
{
      if(!_bb_) return;


      if( world != NULL )
      {
          world->update( (1.0/60.0) );
      }

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


void UnitSceneGeometry::mouseMove(float x , float y , int button)
{

    float speedX = (x - oldX);
    float speedY = (y - oldY);
    oldX = x;
    oldY = y;

    Matrix4 M;
    M.setToIdentity();


    if( mMouseRightButton == Qt::MouseButton::RightButton )
    {

        M = Matrix4::rotationMatrix( Vector3::Y , speedX * 0.01 ) * M;
        M = Matrix4::rotationMatrix( Vector3::X , speedY * 0.01 ) * M;

        mEye = M * mEye;
    }


    //    if( mMouseRightButton == Qt::MouseButton::LeftButton )
    //    {

    //         Vector3 lookDir =  (mCenter - mEye).normalize();
    //         Vector3 leftDir = -(lookDir.cross(Vector3::Y)).normalize();
    //         Vector3 upDir   =  (Vector3::Y);

    //         M = Matrix4::translationMatrix( leftDir * speedX * 0.01 ) * M;
    //         M = Matrix4::translationMatrix(   upDir * speedY * 0.01 ) * M;

    //         mEye    = M * mEye;
    //         mCenter = M * mCenter;
    //    }



}

void UnitSceneGeometry::mousePress(float x, float y, int button)
{
    oldX = x;
    oldY = y;


    if( button == Qt::MouseButton::RightButton )
    {
        mMouseRightButton &= Qt::MouseButton::RightButton;
    }
    else if( button == Qt::MouseButton::LeftButton )
    {
        mMouseRightButton &= Qt::MouseButton::LeftButton;
    }
}

void UnitSceneGeometry::mouseReleasePress(float x, float y, int button)
{

    if( button == Qt::MouseButton::RightButton )
    {
        mMouseRightButton |= ~Qt::MouseButton::RightButton;
    }
    else if( button == Qt::MouseButton::LeftButton )
    {
        mMouseRightButton |= ~Qt::MouseButton::LeftButton;
    }
}

void UnitSceneGeometry::mouseWheel(float delta)
{
    mEye += (mCenter - mEye).normalize() * delta * 0.005;
    delta = 0;
}

void UnitSceneGeometry::keyboard(int key)
{



    if( key == Qt::Key_G )
    {
        cout<< " G " <<endl;
        _bb_ = true;
        initGeometry();

    }

    if( key == Qt::Key_F )
    {
        cout<< " F " <<endl;
        _bb_ = false;

        delete world;

        for( int i = 0; i < mMeshes.size(); ++i )
        {
            delete  mMeshes[i];
        }
        mMeshes.clear();

    }

}

void UnitSceneGeometry::destroy()
{
 //   _bb_ = false;
 //   delete world;
}
