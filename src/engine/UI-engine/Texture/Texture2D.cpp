#include <GL/glew.h>
#include <GL/glu.h>
// Libraries
#include "Texture2D.h"
#include <fstream>
#include <iostream>
#include <string>



using namespace utility_engine;



// Constructor
Texture2D::Texture2D() : mID(0), mLayer(0), mWidth(0), mHeight(0)
{

}

// Constructor
Texture2D::Texture2D(int width, int height, int internalFormat, int format, int type)
          : mID(0), mLayer(0), mWidth(0), mHeight(0)
{
    // Create the texture
    create(width, height, internalFormat, format, type);
}

// Destructor
Texture2D::~Texture2D()
{

}



// Create the texture
void Texture2D::create(int width, int height, int internalFormat, int format, int type, void* data)
throw(std::invalid_argument)
{
    // Destroy the current texture
    destroy();

    mWidth = width;
    mHeight = height;


    glGenTextures(1, &mID);
    glBindTexture(GL_TEXTURE_2D , mID); // Bind the ID texture specified by the 2nd parameter

    // The next commands sets the texture parameters
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT); // If the u,v coordinates overflow the range 0,1 the image is repeated
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR); // The magnification function ("linear" produces better results)
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST); //The minifying function


//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);


    // We don't combine the color with the original surface color, use only the texture map.
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);


    // Finally we define the 2d texture
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT16, width, height, 0, format, type , data );


    // And create 2d mipmaps for the minifying function
    if( data != NULL)
    {
    	gluBuild2DMipmaps(GL_TEXTURE_2D, 4, width, height, format , type , data );
    }

    glBindTexture(GL_TEXTURE_2D, 0);


}


void Texture2D::createDepth( int width, int height)
throw (std::invalid_argument)
{

	    // Destroy the current texture
	    destroy();

	    // запросим у OpenGL свободный индекс текстуры
		glGenTextures(1, &mID);

		// сделаем текстуру активной
		glBindTexture(GL_TEXTURE_2D, mID);


//		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
//		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

		// установим параметры фильтрации текстуры - линейная фильтрация
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

		// установим параметры "оборачиваниея" текстуры - отсутствие оборачивания
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);



		//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_R_TO_TEXTURE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
		glTexParameteri(GL_TEXTURE_2D, GL_DEPTH_TEXTURE_MODE, GL_INTENSITY);
		// необходимо для использования depth-текстуры как shadow map
		//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);

		// соаздем "пустую" текстуру под depth-данные
		glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT , width, height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);

		glBindTexture(GL_TEXTURE_2D, 0);


		setLayer(1);
		// проверим на наличие ошибок
		//OPENGL_CHECK_FOR_ERRORS();

}


// Bind the texture
void Texture2D::bind() const
{
    assert(mID != 0);
    glEnable(GL_TEXTURE_2D);
    glActiveTexture(GL_TEXTURE0 + mLayer);
    glBindTexture(GL_TEXTURE_2D, mID);
}

// Unbind the texture
void Texture2D::unbind() const
{
    assert(mID != 0);
    glActiveTexture(GL_TEXTURE0 + mLayer);
    glBindTexture(GL_TEXTURE_2D, 0);
    glDisable(GL_TEXTURE_2D);
}

// Destroy the texture
void Texture2D::destroy()
{
    if (mID != 0)
    {
        glDeleteTextures(1, &mID);
        mID = 0;
        mLayer = 0;
        mWidth = 0;
        mHeight = 0;
    }
}
