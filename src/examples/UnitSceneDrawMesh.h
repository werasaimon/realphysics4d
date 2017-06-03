/*
 * UnitSceneDynamics.h
 *
 *  Created on: 10 февр. 2017 г.
 *      Author: wera
 */

#ifndef SRC_EXAMPLES_UNITSCENEDRAWMESH_H_
#define SRC_EXAMPLES_UNITSCENEDRAWMESH_H_

#include "UnitScene.h"

#include "../engine/engine.h"

using namespace utility_engine;

class UnitSceneDrawMesh: public UnitScene
{

  private:

	bool mPause = false;


	GLuint mWidth, mHeight;
	GLuint VBO ,vbo;


	std::vector<Mesh*> mMeshes;

	FrameBufferObject   mFBO;

	Shader    mShaderProgram;
	Light     mLight0;

	Texture2D mTexture[4];
	Texture2D mShadowMapTexure;


	Matrix4  mViewMatrix;
	Matrix4  mViewMatrixInverse;
	Matrix4  mProjectionMatrix;

	Matrix4  mLightProjectionMatrix;
	Matrix4  mLightViewMatrix;



  public:
	         UnitSceneDrawMesh( Viewer* viewer );
	virtual ~UnitSceneDrawMesh();

	bool Init();
	void Render(float FrameTime);
	void RenderShadow(float FrameTime);
	void keyboard( unsigned char key );
	void Destroy();


};

#endif /* SRC_EXAMPLES_UNITSCENEDRAWMESH_H_ */
