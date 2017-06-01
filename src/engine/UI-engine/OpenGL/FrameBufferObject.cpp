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

// Libraries
#include "FrameBufferObject.h"
#include <iostream>

using namespace std;
using namespace utility_engine;

// Constructor
FrameBufferObject::FrameBufferObject()
: mFrameBufferID(0), mRenderBufferID (0)
{

}

// Destructor
FrameBufferObject::~FrameBufferObject()
{

}

// Create the frame buffer object
bool FrameBufferObject::create(uint width, uint height, bool needRenderBuffer)
{

    // Destroy the current FBO
    destroy();

    // Check that the needed OpenGL extensions are available
    bool isExtensionOK = checkOpenGLExtensions();
    if (!isExtensionOK) {
        std::cerr << "Error : Impossible to use Framebuffer Object on this platform" << std::endl;
        assert(false);
        return false;
    }

    // Generate a new FBO
    glGenFramebuffersEXT(1, &mFrameBufferID);
    assert(mFrameBufferID != 0);

    // If we also need to create a render buffer
    if (needRenderBuffer) {

        // Generate the render buffer
        glGenRenderbuffersEXT(1, &mRenderBufferID);
        assert(mRenderBufferID != 0);

        glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, mRenderBufferID);
        glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT32, width, height);
        glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, mFrameBufferID);
        glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER,  mRenderBufferID);
        glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, 0);
        glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
    }

    // Check the FBO status
    GLenum statusFBO = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if(statusFBO != GL_FRAMEBUFFER_COMPLETE) {
        cerr << "Error : An error occured while creating the Framebuffer Object !" << endl;
        return false;
    }

    return true;
}

// Destroy the FBO
void FrameBufferObject::destroy() {

    // Delete the frame buffer object
    if (mFrameBufferID)
    {
        glDeleteFramebuffersEXT(1, &mFrameBufferID);
        mFrameBufferID = 0;
    }

    // Delete the render buffer
    if (mRenderBufferID)
    {
        glDeleteRenderbuffersEXT(1, &mRenderBufferID);
    }
}

// Attach a texture to the frame buffer object
void FrameBufferObject::attachTexture(uint position, uint textureID)
{
    assert(mFrameBufferID);

    // Bind the current FBO
      glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, mFrameBufferID);

    // Bind the texture
    //glFramebufferTexture2D(GL_FRAMEBUFFER, position, GL_TEXTURE_2D, textureID, 0);
      glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_TEXTURE_2D, textureID, 0);

    // Unbind the current FBO
      glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
}


// Bind the FBO
void FrameBufferObject::bind(uint position) const
{
    assert(mFrameBufferID != 0);

    //glBindFramebuffer(GL_FRAMEBUFFER, mFrameBufferID);
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, mFrameBufferID);

    glDrawBuffer(position);
    glReadBuffer(position);

    glDrawBuffers(position, NULL);

}

// Unbind the FBO
void FrameBufferObject::unbind() const
{
    assert(mFrameBufferID != 0);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    //glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT , 0);
}

// Return true if the needed OpenGL extensions are available for FBO
bool FrameBufferObject::checkOpenGLExtensions()
{

    // Check that OpenGL version is at least 3.0 or there the framebuffer object extension exists
    return (GLEW_VERSION_3_0 || GLEW_ARB_framebuffer_object);
}

