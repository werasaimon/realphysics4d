/*
 * UnitSceneDrawMesh.cpp
 *
 *  Created on: 10 февр. 2017 г.
 *      Author: wera
 */

#include "UnitSceneDrawMesh.h"

#define SHADOW_MAP_SIZE 1024

namespace
{
    static Vector3 LightPosition( 5.0f, 55.f, 25.0f);

    Mesh *meshTest;
}


UnitSceneDrawMesh::UnitSceneDrawMesh( Viewer* viewer )
:UnitScene(viewer)
{

}

UnitSceneDrawMesh::~UnitSceneDrawMesh()
{
	Destroy();
}

bool UnitSceneDrawMesh::Init()
{

	//mViewer->getCamera().setPosition(Vector3::Z * -20.0);


	cameraValue.mEyePosition = Vector3::Z * -20.0;

	/**************************************
	 *
	 * Initialization render OpenGL utility
	 *
	 **************************************/
	TextureReaderWriter::loadTextureFromFile( "Files/floor.bmp" ,  mTexture[0] );
	TextureReaderWriter::loadTextureFromFile( "Files/cube.bmp"  ,  mTexture[1] );
	TextureReaderWriter::loadTextureFromFile( "Files/box.bmp"   ,  mTexture[2] );


	mLight0.translateWorld( Vector3( LightPosition.x , LightPosition.y , LightPosition.z ) );
	mLight0.createShadowMap( SHADOW_MAP_SIZE, SHADOW_MAP_SIZE );

	bool init = mShaderProgram.create("shaders/phong.vert",
				                      "shaders/phong.frag");

	mShaderProgram.bind();
	mShaderProgram.setUniformValue( "Texturing" , 0);
	mShaderProgram.setUniformValue( "ShadowMap" , 1);
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


    bool b = false;
    for( unsigned int i = 0; i < 10; ++i )
    {
    	Mesh *mesh = new MeshReadFile3DS( "Files/cub.3ds");

    	Vector3 pos(0, i * 7.0 ,0);

    	Matrix4 matrix;
    	matrix.setToIdentity();
    	matrix = Matrix4::translationMatrix(pos);

    	mesh->setTransformMatrix( matrix );
    	mesh->setTexture( mTexture[0] );

    	mMeshes.push_back( meshTest = mesh );
    }



    /**************************************
     *
     * Initialization real-physics engine
     *
     **************************************/


	return init;
}

void UnitSceneDrawMesh::Render(float FrameTime)
{

	mViewer->beginLookCameara();


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
	Matrix4 viewMatrix = mViewer->getViewMatrix().getTranspose();
	mShaderProgram.setUniformValue("projectionMatrix",projection);
	mShaderProgram.setUniformValue("worldToViewMatrix",viewMatrix);


	/// Shadow viewer
	Matrix4 ShadowMatrixSum = (biasMatrix * mLightProjectionMatrix * mLightViewMatrix).getTranspose();// * mViewer->ViewMatrixInverse);
	mShaderProgram.setUniformValue( "ShadowMatrix" , ShadowMatrixSum  , GL_TRUE );


	mShaderProgram.setUniformValue( "Texturing" , 1);

	/// Position camera and position light object
	mShaderProgram.setUniformValue( "cameraWorldPosition" , cameraValue.mEyePosition);
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
		UtilityOpenGLMesh::RenderMesh( mMeshes[i] );
	}


	mShadowMapTexure.unbind();

	glActiveTexture(GL_TEXTURE1); glBindTexture(GL_TEXTURE_2D, 0);
	glActiveTexture(GL_TEXTURE0); glBindTexture(GL_TEXTURE_2D, 0);


	glEnable(GL_CULL_FACE);
	glDisable(GL_DEPTH_TEST);


	//--------------------------------------------------------------------------//
	 RenderShadow(FrameTime);
	//--------------------------------------------------------------------------//


	mShaderProgram.unbind();

}



void UnitSceneDrawMesh::RenderShadow(float FrameTime)
{


	glViewport(0, 0,  SHADOW_MAP_SIZE , SHADOW_MAP_SIZE );
	//glViewport(0, 0, Width, Height);


	//LightViewMatrix  =  look( LightPosition, vec3(0,0,0), vec3(0.0f, 1.0f, 0.0f));
	mLightViewMatrix =  Matrix4::Look( Vector3(LightPosition.x , LightPosition.y , LightPosition.z) ,
			                           Vector3(0,0,0) ,
			                           Vector3(0,1,0) );


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
		UtilityOpenGLMesh::RenderMesh( mMeshes[i] );
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


void UnitSceneDrawMesh::keyboard(unsigned char key)
{
	switch (key)
	{
		case ' ':
		{


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
			meshTest->rotateWorld( Vector3::Z  , 0.01);
			break;
		}


		default:
			break;
	}

}

void UnitSceneDrawMesh::Destroy()
{

	mFBO.destroy();

	mTexture[0].destroy();
	mTexture[1].destroy();
	mTexture[2].destroy();

	mLight0.destroyShadowMap();

	mShaderProgram.destroy();
	mShadowMapTexure.destroy();

	glDeleteBuffers(1, &VBO);

	for (unsigned int i = 0; i < mMeshes.size(); ++i)
	{
	  	if(mMeshes[i] != NULL) delete mMeshes[i];
	}

}


