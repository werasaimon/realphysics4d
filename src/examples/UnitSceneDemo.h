/*
 * UnitSceneDemo.h
 *
 *  Created on: 1 июн. 2017 г.
 *      Author: werqa
 */

#ifndef SRC_EXAMPLES_UNITSCENEDEMO_H_
#define SRC_EXAMPLES_UNITSCENEDEMO_H_

#include "UnitScene.h"
#include "../engine/UI-engine/ui_engine.h"
#include "../engine/UI-engine/OpenGL/Shader.h"


class UnitSceneDemo : public UnitScene
{

private:

	GLuint mWidth, mHeight;
	GLuint VBO ,vbo;



    std::vector<Mesh*> mMesheees;


public:

	UnitSceneDemo(Viewer* viewer)
    : UnitScene(viewer)
    {
    }


	bool Init();
	void Render(float FrameTime);
	void keyboard( unsigned char key );
	void Destroy();

};

#endif /* SRC_EXAMPLES_UNITSCENEDEMO_H_ */
