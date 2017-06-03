#ifndef UTILITYOPENGL_H
#define UTILITYOPENGL_H

#include "GLUtilityGeometry.h"

namespace utility_engine
{


class UtilityOpenGL
{
public:


    static void DrawMesh( Mesh* mesh , QOpenGLShaderProgram *program);



#ifdef __ANDROID__
#elif defined(WIN32) || defined(__linux__)
    static void DrawMesh( Mesh* mesh );
#endif


};


}


#endif // UTILITYOPENGL_H
