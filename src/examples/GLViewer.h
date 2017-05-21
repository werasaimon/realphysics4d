/*
 * GLViewer.h
 *
 *  Created on: 3 окт. 2016 г.
 *      Author: wera
 */

#ifndef OPENGL_GLVIEWER_H_
#define OPENGL_GLVIEWER_H_


// Libraries
#include "../opengl-utility/opengl-utility.h"
#include <GL/glut.h>

using namespace opengl_utility;

class Viewer
  {
    public:
               Viewer();
      virtual ~Viewer();



      // Initialize the viewer
        bool init(int argc, char** argv, const std::string& windowsTitle,
                  const Vector2& windowsSize, const Vector2& windowsPosition,
                  bool isMultisamplingActive = false);

      //----------------- function ----------------------------//

              void initilisation();
              void update();

              // Called when the windows is reshaped
              void reshape(int width, int height);
              void zoom(int xMouse, int yMouse);
              void motion( int _x, int _y );

              // Called when a GLUT mouse button event occurs
              void mouseButtonEvent(int button, int state, int x, int y);

              void keyboard(unsigned char key, int x, int y);
              void special(unsigned int key, int x, int y);


      /********************* get method ****************/

      Matrix4 getProjectionMatrix() const;
      Matrix4 getViewMatrix() const;

      int getWidth() const;
      int getHeight() const;

      CCameraEya getCamera() const;


      // Enable/Disable the multi-sampling for anti-aliasing
     inline void activateMultiSampling(bool isActive) const;

  private:


      bool initGLUT(int argc, char** argv, const std::string& windowsTitle,
   	        	    const Vector2& windowsSize, const Vector2& windowsPosition,
   				    bool isMultisamplingActive);

       CCameraEya camera;

       int mWidth;
       int mHeight;

       // Last mouse coordinates on the windows
       float mLastMouseAngleX;
       float mLastMouseAngleY;

       float mOldMousePosX;
       float mOldMousePosY;


  };


#endif /* OPENGL_GLVIEWER_H_ */
