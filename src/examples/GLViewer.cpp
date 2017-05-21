/*
 * GLViewer.cpp
 *
 *  Created on: 3 окт. 2016 г.
 *      Author: wera
 */

#include "GLViewer.h"

#include <iostream>

using namespace std;
using namespace opengl_utility;

//---------------------------------------------------------------------------//

Viewer::Viewer():
mLastMouseAngleX(0),
mLastMouseAngleY(0)
{
}

Viewer::~Viewer()
{
    // TODO Auto-generated destructor stub
}

bool Viewer::init(int argc, char** argv, const std::string& windowsTitle,
		const Vector2& windowsSize, const Vector2& windowsPosition,
		bool isMultisamplingActive)
{

	 // Initialize the GLUT library
	    bool outputValue = initGLUT(argc, argv, windowsTitle, windowsSize,
	                                windowsPosition, isMultisamplingActive);

	    // Active the multi-sampling by default
	    if (isMultisamplingActive)
	    {
	        activateMultiSampling(true);
	    }

	    return outputValue;
}



bool Viewer::initGLUT(int argc, char** argv, const std::string& windowsTitle,
		              const Vector2& windowsSize, const Vector2& windowsPosition,
		              bool isMultisamplingActive)
{

	  // Initialize GLUT
	    glutInit(&argc, argv);
	    uint modeWithoutMultiSampling = GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH;
	    uint modeWithMultiSampling = GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GL_MULTISAMPLE;
	    uint displayMode = isMultisamplingActive ? modeWithMultiSampling : modeWithoutMultiSampling;
	    glutInitDisplayMode(displayMode);

	    // Initialize the size of the GLUT windows
	    glutInitWindowSize(static_cast<int>(windowsSize.x),
	                       static_cast<int>(windowsSize.y));

	    // Initialize the position of the GLUT windows
	    glutInitWindowPosition(static_cast<int>(windowsPosition.x),
	                           static_cast<int>(windowsPosition.y));

	    // Create the GLUT windows
	    glutCreateWindow(windowsTitle.c_str());

	    // Initialize the GLEW library
	    GLenum error = glewInit();

	    if (error != GLEW_OK)
	    {
	        // Problem: glewInit failed, something is wrong
	        //cerr << "GLEW Error : " << glewGetErrorString(error) << std::endl;
	        assert(false);
	        return false;
	    }

	    return true;
}


void Viewer::initilisation()
{
    int w = 600;//width;
    int h = 400;//height;

    uint aspect = uint(w) / uint(h ? h : 1);
    // Set near plane to 3.0, far plane to 7.0, field of view 45 degrees
    const uint zNear = 3.0, zFar = 512, fov = 45.0;

    camera.initProjectionMatrix( fov , aspect , zNear , zFar );


    camera.OnMouseMove(0,0);
    camera.setPosition( Vector3(1.75f, 25.75f, 40.0f) );
}

void Viewer::update()
{
    camera.update();
}


void Viewer::reshape(int width, int height)
{
    this->mWidth  = width;
    this->mHeight = height;

    camera.setDimensions(width, height);
    glViewport(0, 0, width, height);

    glutPostRedisplay();
}

void Viewer::zoom(int xMouse, int yMouse)
{
}

void Viewer::motion(int _x, int _y )
{
    //float m_world_size = 1;
    float mouse_x = _x;//       (( _x / (float) mWidth )) * m_world_size;
    float mouse_y = _y;//(1.0f - ( _y / (float) mHeight)) * m_world_size;

    float MouseSpeedX = (mouse_x - mOldMousePosX) * 0.01;
    float MouseSpeedY = (mouse_y - mOldMousePosY) * 0.01;

    camera.OnMouseMove( mLastMouseAngleX += MouseSpeedX,
                        mLastMouseAngleY += MouseSpeedY);


    mOldMousePosX = mouse_x;
    mOldMousePosY = mouse_y;

    glutPostRedisplay();

}



void Viewer::mouseButtonEvent(int button, int state, int x, int y)
{
        //float m_world_size = 1;
        float mouse_x = x;//       (( x / (float) mWidth )) * m_world_size;
        float mouse_y = y;//(1.0f - ( y / (float) mHeight)) * m_world_size;
        mOldMousePosX = mouse_x;
        mOldMousePosY = mouse_y;

}



void Viewer::keyboard(unsigned char key, int x, int y)
{

	Matrix4 rot =camera.getRotatetMatrix().getTranspose();
	float speed = 0.5;
    switch (key)
    {

		case 'w': case'W': camera.setPosition( camera.getPosition() - rot * Vector3::Z * speed );  break;
		case 's': case'S': camera.setPosition( camera.getPosition() + rot * Vector3::Z * speed );  break;
		case 'a': case'A': camera.setPosition( camera.getPosition() - rot * Vector3::X * speed );  break;
		case 'd': case'D': camera.setPosition( camera.getPosition() + rot * Vector3::X * speed );  break;


        default:   break;
    }
}

void Viewer::special(unsigned int key, int x, int y)
{

}


int Viewer::getHeight() const
{
    return mHeight;
}


int Viewer::getWidth() const
{
    return mWidth;
}

Matrix4 Viewer::getViewMatrix() const
{
    return camera.getViewMatrix();
}

Matrix4 Viewer::getProjectionMatrix() const
{
    return camera.getProjectionMatrix();
}



CCameraEya Viewer::getCamera() const
{
    return camera;
}



inline void Viewer::activateMultiSampling(bool isActive) const
{
	 (isActive) ? glEnable(GL_MULTISAMPLE) : glDisable(GL_MULTISAMPLE);
}





