/*
 * GLViewer.cpp
 *
 *  Created on: 3 окт. 2016 г.
 *      Author: wera
 */

#include "GLViewer.h"
#include <assert.h>
#include <iostream>

using namespace std;
using namespace utility_engine;

//---------------------------------------------------------------------------//


Matrix4 mRotatetMatrix;

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

    mCamera.setToIdentity();
    mCamera.ProjectionMatrix( fov , aspect , zNear , zFar );
   // camera.translateWorld( Vector3(1.75f, 25.75f, 40.0f) );


    cameraValue = _camera_value();
}



void Viewer::beginLookCameara()
{
	Vector3 eye = cameraValue.mEyePosition;
	Vector3 cen = cameraValue.mLookCenter;
	Vector3 up  = cameraValue.mUp;

	mCamera.LookAt(eye , cen , up);
}


void Viewer::reshape(int width, int height)
{
    this->mWidth  = width;
    this->mHeight = height;

    float w = width;
    float h = height;

    uint aspect = uint(w) / uint(h ? h : 1);
    // Set near plane to 3.0, far plane to 7.0, field of view 45 degrees
    const uint zNear = 3.0, zFar = 512, fov = 45.0;

    mCamera.setToIdentity();
    mCamera.ProjectionMatrix( fov , aspect , zNear , zFar );
   // camera.translateWorld( Vector3(1.75f, 25.75f, 40.0f) );

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

//    camera.OnMouseMove( mLastMouseAngleX += MouseSpeedX,
//                        mLastMouseAngleY += MouseSpeedY);


    mRotatetMatrix.setToIdentity();
    mRotatetMatrix = mRotatetMatrix * Matrix4::rotationMatrix(Vector3::Y, mLastMouseAngleX += MouseSpeedX );
    mRotatetMatrix = mRotatetMatrix * Matrix4::rotationMatrix(Vector3::X, mLastMouseAngleY += MouseSpeedY );


	cameraValue.mLookCenter = cameraValue.mEyePosition + mRotatetMatrix.getTranspose() * -Vector3::Z * 40.0;

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
	float speed = 0.5;
    switch (key)
    {

		case 'w': case'W': cameraValue.mEyePosition += (mRotatetMatrix.getTranspose() *  (-Vector3::Z * speed));  break;
		case 's': case'S': cameraValue.mEyePosition += (mRotatetMatrix.getTranspose() *  ( Vector3::Z * speed));  break;
		case 'a': case'A':
		{
			cameraValue.mLookCenter  += (mRotatetMatrix.getTranspose() *  (-Vector3::X * speed));
			cameraValue.mEyePosition += (mRotatetMatrix.getTranspose() *  (-Vector3::X * speed));
			break;
		}
		case 'd': case'D':
		{
			cameraValue.mLookCenter  += (mRotatetMatrix.getTranspose() *  ( Vector3::X * speed));
			cameraValue.mEyePosition += (mRotatetMatrix.getTranspose() *  ( Vector3::X * speed));
			break;
		}


        default:   break;
    }

    mCamera.setTransformMatrix( Matrix4::translationMatrix(cameraValue.mEyePosition) );

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
    return mCamera.getViewMatrix();
}

Matrix4 Viewer::getProjectionMatrix() const
{
    return mCamera.getProjectionMatrix();
}



Camera &Viewer::getCamera()
{
    return mCamera;
}



inline void Viewer::activateMultiSampling(bool isActive) const
{
	// (isActive) ? glEnable(GL_MULTISAMPLE) : glDisable(GL_MULTISAMPLE);
}





