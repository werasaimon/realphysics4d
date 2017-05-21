/*
 * UnitSceneDynamics.cpp
 *
 *  Created on: 10 февр. 2017 г.
 *      Author: wera
 */

#include "UnitSceneDynamics.h"

#define SHADOW_MAP_SIZE 1024

namespace
{
    static opengl_utility::Vector3 LightPosition( 5.0f, 55.f, 25.0f);

    opengl_utility::Mesh *meshTest;


    real_physics::rpRigidPhysicsBody *body11 = NULL;
    real_physics::rpRigidPhysicsBody *body22 = NULL;
}


UnitSceneDynamics::UnitSceneDynamics( Viewer* viewer )
:UnitScene(viewer)
{

}

UnitSceneDynamics::~UnitSceneDynamics()
{
	Destroy();
}

bool UnitSceneDynamics::Init()
{

	mViewer->getCamera().setPosition(opengl_utility::Vector3::Z * -20.0);



	/**************************************
	 *
	 * Initialization render OpenGL utility
	 *
	 **************************************/
	opengl_utility::TextureReaderWriter::loadTextureFromFile( "Files/floor.bmp" ,  mTexture[0] );
	opengl_utility::TextureReaderWriter::loadTextureFromFile( "Files/cube.bmp"  ,  mTexture[1] );
	opengl_utility::TextureReaderWriter::loadTextureFromFile( "Files/box.bmp"   ,  mTexture[2] );


	mLight0.translateWorld( Vector3( LightPosition.x , LightPosition.y , LightPosition.z ) );
	mLight0.createShadowMap( SHADOW_MAP_SIZE, SHADOW_MAP_SIZE );

	bool init = mShaderProgram.create("shaders/LightShadow.vert",
				                      "shaders/LightShadow.frag");

	mShaderProgram.bind();
	//mShaderProgram.setIntUniform( "Texture"   , 0);
	//mShaderProgram.setUniformValue( "ShadowMap" , 1);
	mShaderProgram.unbind();



	float Data[] =
	{	// s, t, nx, ny, nz, x, y, z
			0.0f, 0.0f, 0.0f, 1.0f, 0.0f, -100.0f, -0.5f, 100.0f,
			20.0f, 0.0f, 0.0f, 1.0f, 0.0f, 100.0f, -0.5f, 100.0f,
			20.0f, 20.0f, 0.0f, 1.0f, 0.0f, 100.0f, -0.5f, -100.0f,
			0.0f, 20.0f, 0.0f, 1.0f, 0.0f, -100.0f, -0.5f, -100.0f,
	};

	glGenBuffers(1, &VBO);
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Data) * sizeof(float), Data, GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);



    mLightProjectionMatrix = Matrix4::Perspectivee( 90.f, 1.0f, 4.0f, 500.0f );
	//mLightProjectionMatrix = Matrix4::Perspective2( 90.f, 1.0f, 4.0f, 500.0f );


    mFBO.create( SHADOW_MAP_SIZE, SHADOW_MAP_SIZE , true);
    mShadowMapTexure.createDepth( SHADOW_MAP_SIZE , SHADOW_MAP_SIZE );
    mFBO.attachTexture( 0 , mShadowMapTexure.getID() );

//    glLightf(GL_LIGHT0, GL_CONSTANT_ATTENUATION,  1.5f);
//    glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION, 1.0f / 128.0f);
//    glLightf(GL_LIGHT0, GL_QUADRATIC_ATTENUATION, 1.0f / 256.0f);



    /**************************************
     *
     * Initialization real-physics engine
     *
     **************************************/
	 mGravity       =     real_physics::Vector3::Y * -30;
	 mDynamicsWorld = new real_physics::rpDynamicsWorld(mGravity);



	 real_physics::Transform transform( real_physics::Vector3::Y * 0.0 ,  real_physics::Quaternion::identity());

     real_physics::rpRigidPhysicsBody *physicsObject = mDynamicsWorld->createRigidBody(transform);
	 physicsObject->setType(real_physics::BodyType::STATIC);

	 real_physics::rpCollisionShape *shape = new real_physics::rpBoxShape( real_physics::Vector3(50,5,50));
     physicsObject->addCollisionShape(shape, 10, real_physics::Transform(real_physics::Vector3::Y * -5.5 , real_physics::Quaternion::identity()));

     bool b = false;
	 for( unsigned int j = 0; j < 2; ++j )
	 {
		 for( unsigned int i = 0; i < 52; ++i )
		 {
			 //------------------------------------------------------//
			 opengl_utility::Mesh *mesh = new opengl_utility::MeshReadFile3DS( "Files/cub.3ds");

			 opengl_utility::Vector3 pos(0, 0 ,0);

			 Matrix4 matrix;
			 matrix.setToIdentity();
			 matrix = opengl_utility::Matrix4::translationMatrix(pos);

			 mesh->setTransformMatrix( matrix );
			 mesh->setTexture(  mTexture[2] );

			 mMeshes.push_back( meshTest = mesh );



			 //-----------------------------------------------------/
			 real_physics::Vector3 position( (b=!b)? 1.0 : -1.0 ,10 + i * 10.0, 10 * j );
			 real_physics::Transform transform( position ,  real_physics::Quaternion::identity());

			 real_physics::rpRigidPhysicsBody *physicsObject = mDynamicsWorld->createRigidBody(transform);
			 physicsObject->setType(real_physics::BodyType::DYNAMIC);


			 if( i == 5 ) body11 = physicsObject;
			 if( i == 6 ) body22 = physicsObject;

			 real_physics::rpModelConvexHull *initHull = new real_physics::rpModelConvexHull(MeshConvertToVertexes(mesh));
			 real_physics::rpConvexHullShape* convexHull = new real_physics::rpConvexHullShape(initHull);

			 UltimatePhysics physicsDriverMesh(physicsObject);

			 physicsDriverMesh.addConnectMeshToShape( mesh, convexHull, 10 );

			 mDireverObjects.push_back(physicsDriverMesh);
		 }
	 }


	return true;
}

void UnitSceneDynamics::Render(float FrameTime)
{

	if(mPause)
	{
		mDynamicsWorld->update(FrameTime);
		for (unsigned int i = 0; i < mDireverObjects.size(); ++i)
		{
			mDireverObjects[i].update();
		}
	}


	glViewport(0, 0, mViewer->getWidth() , mViewer->getHeight());


//	glMatrixMode(GL_PROJECTION);
//	glLoadMatrixf(&mViewer->getProjectionMatrix().getTranspose());
//	//glLoadIdentity();
//
//	glMatrixMode(GL_MODELVIEW);
//	glLoadMatrixf(&mViewer->getViewMatrix().getTranspose());

	//---------------------- render To unit Scene ----------------------------------//

	glEnable(GL_DEPTH_TEST);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glDisable(GL_CULL_FACE);
	glFrontFace(GL_CW);
    glCullFace(GL_FRONT);
    //glEnable(GL_CULL_FACE);
    //glEnable(GL_TEXTURE_2D);

	mShaderProgram.bind();


	/// Camera Viewer
	Matrix4 projection = mViewer->getProjectionMatrix();
	Matrix4 viewMatrix = mViewer->getViewMatrix();
	mShaderProgram.setUniformValue("projectionMatrix",projection);
	mShaderProgram.setUniformValue("worldToViewMatrix",viewMatrix);





	/// Shadow viewer
	//Matrix4 ShadowMatrixSum = (biasMatrix * mLightProjectionMatrix * mLightViewMatrix).getTranspose();// * mViewer->ViewMatrixInverse);
	//mShaderProgram.setUniformValue( "worldMatrixShadow" , ShadowMatrixSum  , GL_TRUE );


	mShaderProgram.setUniformValue( "Texturing" , 1);

	/// Position camera and position light object
	mShaderProgram.setUniformValue( "cameraWorldPosition" , mViewer->getCamera().getPosition());
	mShaderProgram.setUniformValue(  "lightWorldPosition" , mLight0.getOrigin());


	/// Light
	const Color& diffCol = mLight0.getDiffuseColor();
	const Color& specCol = mLight0.getSpecularColor();
	mShaderProgram.setUniformValue(   "lightAmbientColor"   , Vector3(0.0f, 0.0f, 0.0f));
	mShaderProgram.setUniformValue(   "lightDiffuseColor"   , Vector3(diffCol.r, diffCol.g, diffCol.b) * 0.99f);
	mShaderProgram.setUniformValue(   "lightSpecularColor"  , Vector3(specCol.r, specCol.g, specCol.b) * 0.99f);

	mShaderProgram.setUniformValue( "shininess", 60.0f );





	mShadowMapTexure.bind();


	Matrix4 mmat;
	mmat.setToIdentity();
	mShaderProgram.setUniformValue( "modelToWorldMatrix" , mmat );
	glPushMatrix();
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glEnableClientState(GL_TEXTURE_COORD_ARRAY);
	glTexCoordPointer(2, GL_FLOAT, 32, (void*)0);
	glEnableClientState(GL_NORMAL_ARRAY);
	glNormalPointer(GL_FLOAT, 32, (void*)8);
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, 32, (void*)20);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, mTexture[2]);
	glColor3f(1.0f, 1.0f, 1.0f);
	glDrawArrays(GL_QUADS, 0, 4);
	glDisable(GL_CULL_FACE);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glPopMatrix();



	glDisable(GL_CULL_FACE);
    glFrontFace(GL_CW);
	glCullFace(GL_FRONT);

	/// Render Objects
	for( unsigned int i = 0;  i < mMeshes.size(); ++i )
	{

		mShaderProgram.setUniformValue( "modelToWorldMatrix" ,  mMeshes[i]->getTransformMatrix() );
		opengl_utility::UtilityOpenGLMesh::RenderMesh( mMeshes[i] );
	}


	mShadowMapTexure.unbind();

	//glActiveTexture(GL_TEXTURE1); glBindTexture(GL_TEXTURE_2D, 0);
	//glActiveTexture(GL_TEXTURE0); glBindTexture(GL_TEXTURE_2D, 0);


	glEnable(GL_CULL_FACE);
	glDisable(GL_DEPTH_TEST);


	//--------------------------------------------------------------------------//
	// RenderShadow(FrameTime);
	//--------------------------------------------------------------------------//


	mShaderProgram.unbind();


	body11->changeToFrameOfReference(body22);

}





void UnitSceneDynamics::RenderShadow(float FrameTime)
{


	glViewport(0, 0,  SHADOW_MAP_SIZE , SHADOW_MAP_SIZE );
	//glViewport(0, 0, Width, Height);


	//LightViewMatrix  =  look( LightPosition, vec3(0,0,0), vec3(0.0f, 1.0f, 0.0f));
	mLightViewMatrix =  Matrix4::Look( Vector3(LightPosition.x , LightPosition.y , LightPosition.z) ,
			                           Vector3(0,0,0) ,
			                           Vector3(0,1,0) );


	//mLightViewMatrix =  Matrix4::Ortho(50, 50, 50, 50, 50, 50);


	/****************************************************/
	//			glMatrixMode(GL_PROJECTION);
	//			glLoadMatrixf(&LightProjectionMatrix0);
	//			//glLoadIdentity();
	//
	//			glMatrixMode(GL_MODELVIEW);
	//			glLoadMatrixf(&LightViewMatrix0);



	Matrix4 projectionLight = mLightProjectionMatrix.getTranspose();
	Matrix4 viewMatrixLight = mLightViewMatrix.getTranspose();
	mShaderProgram.setUniformValue("projectionMatrix",projectionLight);
	mShaderProgram.setUniformValue("worldToViewMatrix",viewMatrixLight);



	mFBO.bind(GL_NONE);
	glClear(GL_DEPTH_BUFFER_BIT);


	glColorMask     ( GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE );
	glDisable       ( GL_TEXTURE_2D );

	glEnable        ( GL_POLYGON_OFFSET_FILL );
	glPolygonOffset ( 4, 4 );


	glEnable(GL_DEPTH_TEST);
	glCullFace(GL_FRONT);

	/// Render Objects
	for( unsigned int i = 0;  i < mMeshes.size(); ++i )
	{
		mShaderProgram.setUniformValue( "modelToWorldMatrix" ,  mMeshes[i]->getTransformMatrix() );
		opengl_utility::UtilityOpenGLMesh::RenderMesh( mMeshes[i] );
	}


	glCullFace(GL_BACK);
	glEnable(GL_CULL_FACE);
	glDisable(GL_DEPTH_TEST);


	// restore state
	glDisable        ( GL_POLYGON_OFFSET_FILL );
	glColorMask      ( GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE );
	glEnable         ( GL_TEXTURE_2D );
	mFBO.unbind();


}


void UnitSceneDynamics::keyboard(unsigned char key)
{
	switch (key)
	{
		case ' ':
		{

			//------------------------------------------------------//
			opengl_utility::Mesh *mesh = new opengl_utility::MeshReadFile3DS( "Files/cub.3ds");

			Matrix4 matrix;
			matrix.setToIdentity();
			mesh->setTransformMatrix( matrix );
			mesh->setTexture( mTexture[2] );

			mMeshes.push_back( mesh );


			//-----------------------------------------------------/
			real_physics::Vector3 position;
			position.x = mViewer->getCamera().getPosition().x;
			position.y = mViewer->getCamera().getPosition().y;
			position.z = mViewer->getCamera().getPosition().z;

			opengl_utility::Vector3 dir = mViewer->getCamera().getRotatetMatrix().getTranspose() * opengl_utility::Vector3::Z;

			real_physics::Quaternion quat;
			quat.orientateBetweenAngleAxis(-real_physics::Vector3::Z, -real_physics::Vector3(dir.x,dir.y,dir.z));

			real_physics::Transform transform( position ,  quat );

			real_physics::rpRigidPhysicsBody *physicsObject = mDynamicsWorld->createRigidBody(transform);
			physicsObject->setType(real_physics::BodyType::DYNAMIC);


			real_physics::rpModelConvexHull *initHull = new real_physics::rpModelConvexHull(MeshConvertToVertexes(mesh));
			real_physics::rpConvexHullShape* convexHull = new real_physics::rpConvexHullShape(initHull);

			UltimatePhysics physicsDriverMesh(physicsObject);

			physicsDriverMesh.addConnectMeshToShape( mesh, convexHull, 10);

			mDireverObjects.push_back(physicsDriverMesh);


			physicsObject->applyForce( real_physics::Vector3(dir.x,dir.y,dir.z) * -60000  , position );

			break;
		}

		case 'p':
		{
			mPause = !mPause;
			break;
		}


		case 'x':
		{
			cout<< "X" <<endl;
			meshTest->rotateWorld( opengl_utility::Vector3::Z  , 0.01);
			break;
		}


		default:
			break;
	}

}

void UnitSceneDynamics::Destroy()
{

//	mFBO.destroy();
//
//	mTexture[0].destroy();
//	mTexture[1].destroy();
//	mTexture[2].destroy();
//
//	mLight0.destroyShadowMap();
//
//	mShaderProgram.destroy();
//	mShadowMapTexure.destroy();
//
//	glDeleteBuffers(1, &VBO);
//
//	for (unsigned int i = 0; i < mMeshes.size(); ++i)
//	{
//	  	if(mMeshes[i] != NULL) delete mMeshes[i];
//	}
//
//
//	if(mDynamicsWorld != NULL) delete mDynamicsWorld;

}


