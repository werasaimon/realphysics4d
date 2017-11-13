#include "UnitStudyRelativityScene.h"


namespace
{
    float oldX = 0;
    float oldY = 0;

    bool _bb_ = true;

     ///////////////////////////////////////////////////////////////////////////////////////////////////////////


    float    angle = 0;
    Vector3  axis(0,0,0);

    Matrix4 LambdaRot( float O , Vector3 R )
    {

        float S = 1.f - cos(O);

        /**
        Matrix4 M( 1 , 0 , 0 , 0 ,
                   0 , R.x*R.x + (1.f - R.x*R.x) * cos(O) , R.x*R.y*S - R.z*sin(O)           , R.x*R.z*S + R.y*sin(O) ,
                   0 , R.x*R.y*S + R.z*sin(O)             , R.y*R.y + (1.f - R.y*R.y)*cos(O) , R.y*R.z*S - R.x*sin(O) ,
                   0 , R.x*R.z*S - R.y*sin(O)             , R.y*R.z*S + R.x*sin(O)           , R.z*R.z + (1.f - R.z*R.z)*cos(O));


        /**/
         Matrix4 M(R.x*R.x + (1.f - R.x*R.x) * cos(O) , R.x*R.y*S - R.z*sin(O)           , R.x*R.z*S + R.y*sin(O)           , 0 ,
                   R.x*R.y*S + R.z*sin(O)             , R.y*R.y + (1.f - R.y*R.y)*cos(O) , R.y*R.z*S - R.x*sin(O)           , 0 ,
                   R.x*R.z*S - R.y*sin(O)             , R.y*R.z*S + R.x*sin(O)           , R.z*R.z + (1.f - R.z*R.z)*cos(O) , 0 ,
                                                                                                                  0 , 0 , 0 , 1);
        /**/

         return M;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////


    Vector3 vel(0,0,0);

    Matrix4 LambdaBoost( float v ,  Vector3 b)
    {
        float Y = pow( (1 - v*v) , -0.5f );
        float c = 100;

        /**
        Matrix4 M(0,0,0,0,
                  0,0,0,0,
                  0,0, Y,((v*b.x)/c)*Y,
                  0,0, ((v*b.x)/c)*Y,Y);

        /**/
        Matrix4 M( 1,0,0,v*b.z*Y,
                   0,1,0,v*b.y*Y,
                   0,0,1,v*b.x*Y,
                   v*b.z*Y , v*b.y*Y , v*b.x*Y , 1);
        /**

        Matrix4 M( Y       , v*b.x*Y , v*b.y*Y , v*b.z*Y ,
                   v*b.x*Y , 1 + (Y - 1)*b.x*b.x , (Y - 1)*b.x*b.y , (Y - 1)*b.x*b.z ,
                   v*b.y*Y , (Y - 1)*b.y*b.x , 1 + (Y - 1)*b.y*b.y , (Y - 1)*b.y*b.z ,
                   v*b.z*Y , (Y - 1)*b.z*b.x , (Y - 1)*b.z*b.y     , 1 + (Y - 1)*b.z*b.z );
        /**

            Matrix4 M(1.f + (Y - 1.f)*b.x*b.x , (Y - 1.f)*b.x*b.y     , (Y - 1.f)*b.x*b.z     , -v*b.x*Y,
                     (Y - 1.f)*b.y*b.x      , 1.f + (Y - 1.f)*b.y*b.y , (Y - 1.f)*b.y*b.z     , -v*b.y*Y,
                     (Y - 1.f)*b.z*b.x      , (Y - 1.f)*b.z*b.y     , 1.f + (Y - 1.f)*b.z*b.z , -v*b.z*Y ,
                      -v*b.x*Y , -v*b.y*Y , -v*b.z*Y , Y);
        /**

        float U = (Y - 1.f);

        Matrix4 M(1.f + U*b.x*b.x , U*b.x*b.y       , U*b.x*b.z       , -v*b.x*Y,
                        U*b.y*b.x , 1.f + U*b.y*b.y , U*b.y*b.z       , -v*b.y*Y,
                        U*b.z*b.x , U*b.z*b.y       , 1.f + U*b.z*b.z , -v*b.z*Y,
                       -v*b.x*Y , -v*b.y*Y , -v*b.z*Y , Y);

        /**/
        return M;
    }

     ///////////////////////////////////////////////////////////////////////////////////////////////////////////
}

UnitStudyRelativityScene::UnitStudyRelativityScene()
{

}

bool UnitStudyRelativityScene::initialization()
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

    //------------------------------------------------//

        Mesh *model0 = new MeshBox( Vector3(5,5,5) );
        //model0->translateWorld( Vector3::Y * -10.0);
        model0->setColorToAllVertices(Color(1,1,1,1));
        mMesh = model0;

    //------------------------------------------------//

}

void UnitStudyRelativityScene::render(float FrameTime)
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

     mMesh->Draw();
}

void UnitStudyRelativityScene::update()
{

}

void UnitStudyRelativityScene::resize(float w, float h)
{
    width  = w;
    height = h;

    float aspect = width / height;
    float zNear  = 3.0;
    float zFar   = 1024;
    float fov    = 45.0;

    mCamera.ProjectionMatrix( fov , aspect , zNear , zFar );
}

void UnitStudyRelativityScene::mouseMove(float x, float y, int button)
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

}

void UnitStudyRelativityScene::mousePress(float x, float y, int button)
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

void UnitStudyRelativityScene::mouseReleasePress(float x, float y, int button)
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

void UnitStudyRelativityScene::mouseWheel(float delta)
{
    mEye += (mCenter - mEye).normalize() * delta * 0.005;
    delta = 0;
}

void UnitStudyRelativityScene::keyboard(int key)
{

    if( key == Qt::Key_Z )
    {
        cout<<  "ZZZ" <<endl;

        angle += 0.1f;
        mMesh->setTransformMatrix( LambdaRot( angle , (Vector3::X+Vector3::Y+Vector3::Z).normalize()) );

        /**
        mMesh->setTransformMatrix( LambdaRot( angle , (Vector3::X).normalize()) *
                                   LambdaRot( angle , (Vector3::Y).normalize()) *
                                   LambdaRot( angle , (Vector3::Z).normalize()));
        /**/
    }

    if( key == Qt::Key_X )
    {
        cout<< "X_X_X" <<endl;

        vel = Vector3::X * 0.1f;
        mMesh->setTransformMatrix( mMesh->getTransformMatrix() * LambdaBoost(vel.length() , vel.normalize()) );
    }

}

void UnitStudyRelativityScene::destroy()
{

}
