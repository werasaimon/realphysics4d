#ifndef UTILOPENGL_H
#define UTILOPENGL_H

#include <GL/glew.h>

#include "engine/UI-engine/engine.h"

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glext.h>

class UtilOpenGL
{
    public:

        UtilOpenGL();


        static void glViewport_( int x , int y , int w , int h )
        {
            glViewport( x , y , w , h );
        }


        static void glProject_( const engine::Matrix4 &M )
        {
            glMatrixMode(GL_PROJECTION);
            glLoadMatrixf(M.getTranspose().dataBlock());
        }


        static void glModelView_( const engine::Matrix4 &M )
        {
            glMatrixMode(GL_MODELVIEW);
            glLoadMatrixf(M.getTranspose().dataBlock());
        }


        static void glLoadIdentity_( engine::Matrix4 &M )
        {
            M.setToIdentity();
        }

        static void glClear_( int mask )
        {
            glClear( mask );
        }


};

#endif // UTILOPENGL_H
