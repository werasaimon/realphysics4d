/*
 * UnitSceneDemo.cpp
 *
 *  Created on: 1 июн. 2017 г.
 *      Author: werqa
 */

#include "UnitSceneDemo.h"


std::vector<Mesh*> mMeshes;

Texture2D mTexture[4];
Shader    mShaderProgram;
Light     mLight0;

bool UnitSceneDemo::Init()
{



	TextureReaderWriter::loadTextureFromFile( "Files/floor.bmp" ,  mTexture[0] );
	TextureReaderWriter::loadTextureFromFile( "Files/cube.bmp"  ,  mTexture[1] );
	TextureReaderWriter::loadTextureFromFile( "Files/box.bmp"   ,  mTexture[2] );


	bool init = mShaderProgram.create("shaders/LightShadow.vert",
				                      "shaders/LightShadow.frag");



	mLight0.translateWorld( Vector3( 5 , 55 , -25 ) );
    mLight0.createShadowMap( 1024, 1024 );

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


	for( int i = 0; i < 12; ++i )
	{
		//------------------------------------------------------//
		Mesh *mesh = new MeshReadFile3DS( "Files/cub.3ds");
		Vector3 pos(0, 7 * i , -20);
		mesh->setToIdentity();
		mesh->translateWorld(pos);
		mesh->setTexture(mTexture[1]);

		mMeshes.push_back(mesh);

	}



	return init;


}

void UnitSceneDemo::Render(float FrameTime)
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


    //glTranslatef(0,0,0);
    glutSolidCube(2.0);

//        glPushMatrix();
//    	glBindBuffer(GL_ARRAY_BUFFER, VBO);
//    	glEnableClientState(GL_TEXTURE_COORD_ARRAY);
//    	glTexCoordPointer(2, GL_FLOAT, 32, (void*)0);
//    	glEnableClientState(GL_NORMAL_ARRAY);
//    	glNormalPointer(GL_FLOAT, 32, (void*)8);
//    	glEnableClientState(GL_VERTEX_ARRAY);
//    	glVertexPointer(3, GL_FLOAT, 32, (void*)20);
//    	glActiveTexture(GL_TEXTURE0);
//    	glBindTexture(GL_TEXTURE_2D, mTexture[0]);
//    	glColor3f(1.0f, 1.0f, 1.0f);
//    	glDrawArrays(GL_QUADS, 0, 4);
//    	glDisable(GL_CULL_FACE);
//    	glBindBuffer(GL_ARRAY_BUFFER, 0);
//    	glPopMatrix();



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
    mShaderProgram.setUniformValue( "cameraWorldPosition" , mViewer->getCamera().getOrigin());
    mShaderProgram.setUniformValue(  "lightWorldPosition" , mLight0.getOrigin());


    /// Light
    const Color& diffCol = mLight0.getDiffuseColor();
    const Color& specCol = mLight0.getSpecularColor();
    mShaderProgram.setUniformValue(   "lightAmbientColor"   , Vector3(0.0f, 0.0f, 0.0f));
    mShaderProgram.setUniformValue(   "lightDiffuseColor"   , Vector3(diffCol.r, diffCol.g, diffCol.b) * 0.99f);
    mShaderProgram.setUniformValue(   "lightSpecularColor"  , Vector3(specCol.r, specCol.g, specCol.b) * 0.99f);

    mShaderProgram.setUniformValue( "shininess", 60.0f );


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


   // mShaderProgram.unbind();

}

void UnitSceneDemo::keyboard(unsigned char key)
{
}

void UnitSceneDemo::Destroy()
{
}
