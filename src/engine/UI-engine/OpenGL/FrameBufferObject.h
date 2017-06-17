#ifndef FRAME_BUFFER_OBJECT_H
#define FRAME_BUFFER_OBJECT_H

// Libraries

#include <GL/glew.h>
#include <cassert>


namespace utility_engine
{


typedef unsigned int uint;

// Class FrameBufferObject
class FrameBufferObject
{

    private:

        // -------------------- Attributes -------------------- //

        // Frame buffer ID
        uint mFrameBufferID;

        // Render buffer ID
        uint mRenderBufferID;

    public:

        // -------------------- Methods -------------------- //

        // Constructor
        FrameBufferObject();

        // Destructor
        ~FrameBufferObject();

        // Create the frame buffer object
        bool create(uint width, uint height, bool needRenderBuffer = true);

        // Attach a texture to the frame buffer object
        void attachTexture(uint position, uint textureID);

        // Bind the FBO
        void bind(uint position) const;

        // Unbind the FBO
        void unbind() const;

        // Return true if the needed OpenGL extensions are available for FBO
        static bool checkOpenGLExtensions();

        // Destroy the FBO
        void destroy();


    ///  Info value buffer frame id
	uint getFrameBufferId() const
	{
		return mFrameBufferID;
	}

	///  Info value buffer render id
	uint getRenderBufferId() const
	{
		return mRenderBufferID;
	}
};


}

#endif
