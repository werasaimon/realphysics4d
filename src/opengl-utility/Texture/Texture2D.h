/********************************************************************************
* OpenGL-Framework                                                              *
* Copyright (c) 2013 Daniel Chappuis                                            *
*********************************************************************************
*                                                                               *
* This software is provided 'as-is', without any express or implied warranty.   *
* In no event will the authors be held liable for any damages arising from the  *
* use of this software.                                                         *
*                                                                               *
* Permission is granted to anyone to use this software for any purpose,         *
* including commercial applications, and to alter it and redistribute it        *
* freely, subject to the following restrictions:                                *
*                                                                               *
* 1. The origin of this software must not be misrepresented; you must not claim *
*    that you wrote the original software. If you use this software in a        *
*    product, an acknowledgment in the product documentation would be           *
*    appreciated but is not required.                                           *
*                                                                               *
* 2. Altered source versions must be plainly marked as such, and must not be    *
*    misrepresented as being the original software.                             *
*                                                                               *
* 3. This notice may not be removed or altered from any source distribution.    *
*                                                                               *
********************************************************************************/

#ifndef TEXTURE2D_H
#define TEXTURE2D_H

// Libraries
#include <string>
#include <cassert>
#include <iostream>

#include <GL/gl.h>


namespace opengl_utility
{
// Class Texture2D
// This class represents a 2D texture
class Texture2D
{


	friend class  TextureReader;

    private:

        // -------------------- Attributes -------------------- //

        // OpenGL texture ID
        GLuint mID;

        // Layer of the texture
        GLuint mLayer;

        // Width
        int mWidth;

        // Height
        int mHeight;




    public:

        // -------------------- Methods -------------------- //

        // Constructor
        Texture2D();

        // Constructor
        Texture2D(int width, int height, int internalFormat, int format, int type);

        // Destructor
        ~Texture2D();

        // Create the texture
        void create(int width, int height, int internalFormat, int format, int type, void* data = NULL)
        throw(std::invalid_argument);

        // Create Depth the texture
        void createDepth( int width, int height )
        throw(std::invalid_argument);



        // Destroy the texture
        void destroy();

        // Bind the texture
        void bind() const;

        // Unbind the texture
        void unbind() const;

        // Get the OpenGL texture ID
        int getID() const;

        // Get the layer of the texture
        int getLayer() const;

        // Set the layer of the texture
        void setLayer(int layer);

        // Get the width
        int getWidth() const;

        // Get the height
        int getHeight() const;


        operator GLuint();

};



// Get the OpenGL texture ID
inline int Texture2D::getID() const {
    return mID;
}

// Get the layer of the texture
inline int Texture2D::getLayer() const {
    return mLayer;
}

// Set the layer of the texture
inline void Texture2D::setLayer(int layer) {
    mLayer = layer;
}

// Get the width
inline int Texture2D::getWidth() const {
    return mWidth;
}



inline Texture2D::operator GLuint()
{
	return mID;
}

// Get the height
inline int Texture2D::getHeight() const {
    return mHeight;
}

}


#endif
