#include "UnitSceneDemo.h"



UnitSceneDemo::UnitSceneDemo()
{

}

///-------------------------------///
bool UnitSceneDemo::initShader()
{

    /************************* Init-Shaders ******************************/
    // Compile vertex shader
    if (!mProgramShader.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/vshader2.glsl"))
    {
        return false;
    }

    // Compile fragment shader
    if (!mProgramShader.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/fshader2.glsl"))
    {
        return false;
    }


    // Link shader pipeline
    if (!mProgramShader.link())
    {
        return false;
    }


    // Bind shader pipeline for use
    if (!mProgramShader.bind())
    {
        return false;
    }

    mProgramShader.release();


}


Matrix4 M1;
Matrix4 M2;


bool UnitSceneDemo::initGeometry()
{

    width  = 600;
    height = 400;

    float aspect = width / height;
    float zNear  = 3.0;
    float zFar   = 1024;
    float fov    = 45.0;

    mCamera.ProjectionMatrix( fov , aspect , zNear , zFar );

    mEye    =  Vector3(0,100,-190);
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
     if(!CopyFileFromToResources( mFile2 , ":/Files/plane.3DS" )) return false;

     for( int i =0 ; i < 15; ++i )
     {

         Mesh *meshModel = new MeshReadFile3DS(fileName2);

         Vector3 position =   Vector3::Y * i * 15.0 + Vector3::X * 1 * 2.5;
         //position += Vector3::Z * -0.0;
         //position += Vector3::X * -0.0;

         //meshModel->setToIdentity();
         meshModel->translateWorld( position );
         meshModel->setTexture(texture2D);

         mMeshes.push_back(meshModel);
     }
     /**/

     mFile2.remove();


}

bool UnitSceneDemo::initPhysics()
{

    mTimeStep = 1.0 / 60.0;
    mDynamicsWorld = new DynamicsWorld( Vector3::Y * -30.0 );



    //---------------------------------------------------//

    /**/

    Matrix4 M;
    M= M.translationMatrix(Vector3::Y * -10.0);
    UltimatePhysicsBody* physicss = mDynamicsWorld->createRigidBody(M);
    physicss->setType( real_physics::BodyType::STATIC );

    real_physics::Vector3 halfSizeBox( 250.0 , 10.0 , 250.0);
    M.setToIdentity();
    physicss->addCollisionGeometry( new real_physics::rpBoxShape(halfSizeBox)  , M , 10.0 );
    mPhysicsBodies.push_back(physicss);





    for( int i =0 ; i < 15; ++i )
    {

        Vector3 position =   Vector3::Y * (i+1) * 15.0 + Vector3::X * 1 * 2.5;
        Matrix4 MM =  MM.translationMatrix( position );

        UltimatePhysicsBody* physics1 = mDynamicsWorld->createRigidBody(MM);
        physics1->setType( real_physics::BodyType::DYNAMIC );

        mMeshes[i]->setToIdentity();
        physics1->addCollisionGeometry_ConvexHull( mMeshes[i] , 5.0 );
        mPhysicsBodies.push_back( physics1 );

    }

   /**/



}



///-------------------------------///

bool UnitSceneDemo::initialization()
{
   NullAllKey();

   initShader();
   initGeometry();
   initPhysics();
}


void UnitSceneDemo::render(float FrameTime)
{

  glViewport(0, 0, width , height );


  /**/
    //------------------ Render --------------------//

   mProgramShader.bind();

   mCamera.LookAt( mEye , mCenter  , mUp );


   mProgramShader.UniformValue("ProjectionMatrix"  , mCamera.getProjectionMatrix());
   mProgramShader.UniformValue("ViewMatrix"        , mCamera.getViewMatrix());


       //    glLoadIdentity();
       //    glMatrixMode(GL_PROJECTION);
       //    glLoadMatrixf(projection.data());

       //    glMatrixMode(GL_MODELVIEW);
       //    glLoadMatrixf(viewMatrix.data());



   glEnable(GL_DEPTH_TEST);
   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
   glDisable(GL_CULL_FACE);


   glFrontFace(GL_CW);
   glCullFace(GL_FRONT);
   glEnable(GL_CULL_FACE);


   for(unsigned int i=0; i < mMeshes.size(); ++i)
   {
        mProgramShader.UniformValue( "ModelMatrix" , mMeshes[i]->getTransformMatrix() );
        mMeshes[i]->DrawOpenGL(&mProgramShader);
   }



   glEnable(GL_CULL_FACE);
   glDisable(GL_DEPTH_TEST);


   mProgramShader.release();

   /**/


        //   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        //   glLoadIdentity();


        //   glMatrixMode(GL_PROJECTION);
        //   glLoadMatrixf(mCamera.getProjectionMatrix().dataBlock());

        //   glMatrixMode(GL_MODELVIEW);
        //   glLoadMatrixf(mCamera.getViewMatrix().dataBlock());


        //   for(unsigned int i=0; i < mMeshes.size(); ++i)
        //   {
        //        mMeshes[i]->Draw();
        //   }

}


void UnitSceneDemo::update()
{


    mDynamicsWorld->update( mTimeStep );

    for (int i = 0; i < mPhysicsBodies.size(); ++i)
    {
       mPhysicsBodies[i]->update();
    }


    //-------- keyboard -------//

    if( mKeys[Qt::Key_V] )
    {
        cout<< "suka" <<endl;
        mEye += Vector3(0,0,0.1);

    }

}


void UnitSceneDemo::resize(float w , float h )
{
    width  = w;
    height = h;

    float aspect = width / height;
    float zNear  = 3.0;
    float zFar   = 1024;
    float fov    = 45.0;

    mCamera.ProjectionMatrix( fov , aspect , zNear , zFar );
}


float oldX = 0;
float oldY = 0;
void UnitSceneDemo::mouseMove(float x, float y , int button)
{

    M1.setToIdentity();
    M2.setToIdentity();
    M1 = Matrix4::rotationMatrix2( Vector3::Y , (x - oldX) * 0.01 ) * M1;
    M1 = Matrix4::rotationMatrix2( Vector3::X , (y - oldY) * 0.01 ) * M1;
    mEye = M1 * mEye;
    oldX = x;
    oldY = y;

}

void UnitSceneDemo::mousePress(float x, float y, int button)
{
    oldX = x;
    oldY = y;
}

void UnitSceneDemo::mouseReleasePress(float x, float y, int button)
{

}

void UnitSceneDemo::mouseWheel(float delta)
{
    mEye += (Vector3(0,0,0) - mEye).normalize() * delta * 0.05;
    delta = 0;
}




void UnitSceneDemo::keyboard( int key )
{


    if( key == Qt::Key_V )
    {

        cout<< "initilization" <<endl;
        initGeometry();
        initPhysics();
    }


    if( key == Qt::Key_B )
    {

        cout<< "destroy" <<endl;
        destroy();


    }

}

void UnitSceneDemo::destroy()
{



    for (int i = 0; i < mMeshes.size(); ++i)
    {
        delete mMeshes[i];
    }

    for (int i = 0; i < mPhysicsBodies.size(); ++i)
    {
        delete mPhysicsBodies[i];
    }

    mMeshes.clear();
    mPhysicsBodies.clear();

    delete mDynamicsWorld;


}
