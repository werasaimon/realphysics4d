/*
 * UnitSceneDynamics.h
 *
 *  Created on: 10 февр. 2017 г.
 *      Author: wera
 */

#ifndef SRC_EXAMPLES_UNITSCENEDYNAMICS_H_
#define SRC_EXAMPLES_UNITSCENEDYNAMICS_H_

#include "../element-engine/interface.h"
#include "UnitScene.h"



class UnitSceneDynamics: public UnitScene
{

  private:

	bool mPause = false;

	real_physics::Vector3          mGravity;
	real_physics::rpDynamicsWorld *mDynamicsWorld;


	GLuint mWidth, mHeight;
	GLuint VBO ,vbo;

	std::vector<UltimatePhysics> mDireverObjects;
	std::vector<opengl_utility::Mesh*> mMeshes;

	opengl_utility::FrameBufferObject   mFBO;

	opengl_utility::Shader    mShaderProgram;
	opengl_utility::Light     mLight0;

	opengl_utility::Texture2D mTexture[4];
	opengl_utility::Texture2D mShadowMapTexure;


	opengl_utility::Matrix4  mViewMatrix;
	opengl_utility::Matrix4  mViewMatrixInverse;
	opengl_utility::Matrix4  mProjectionMatrix;

	opengl_utility::Matrix4  mLightProjectionMatrix;
	opengl_utility::Matrix4  mLightViewMatrix;



  public:
	         UnitSceneDynamics( Viewer* viewer );
	virtual ~UnitSceneDynamics();

	bool Init();
	void Render(float FrameTime);
	void RenderShadow(float FrameTime);
	void keyboard( unsigned char key );
	void Destroy();


};

#endif /* SRC_EXAMPLES_UNITSCENEDYNAMICS_H_ */
