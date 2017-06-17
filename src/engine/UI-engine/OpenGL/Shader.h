#ifndef SHADER_H_
#define SHADER_H_

#include <assert.h>
#include <GL/glew.h>
#include <iostream>
#include <string>


#include "../maths/Vector2.h"
#include "../maths/Vector3.h"
#include "../maths/glmath.h"


namespace utility_engine
{

class Vector4;
class Matrix4;


class Shader
{

    private :

        //-------------------- Attributes -----------------//

        // Shader object program ID
        GLuint mProgramObjectID;

        // Filenames of the vertex and fragment shaders
        std::string mFilenameVertexShader, mFilenameFragmentShader;

    public :

        //-------------------- Methods --------------------//

        // Constructor
        Shader();

        // Constructor with arguments
        Shader(const std::string vertexShaderFilename, const std::string fragmentShaderFilename);

        // Destructor
        ~Shader();

        // Create the shader
        bool create(const std::string vertexShaderFilename,
                    const std::string fragmentShaderFilename);

        // Clear the shader
        void destroy();

        // Bind the shader
        void bind() const;

        // Unbind the shader
        void unbind() const;

        // Return the location of a uniform variable inside a shader program
        int getUniformLocation(const std::string& variableName) const;

        // Set a float uniform value to this shader (be careful if the uniform is not
        // used in the shader, the compiler will remove it, then when you will try
        // to set it, an assert will occur)
        void setUniformValue(const std::string& variableName, float value) const;

        // Set an int uniform value to this shader (be careful if the uniform is not
        // used in the shader, the compiler will remove it, then when you will try
        // to set it, an assert will occur)
        void setUniformValue(const std::string& variableName, int value) const;

        // Set a vector 2 uniform value to this shader (be careful if the uniform is not
        // used in the shader, the compiler will remove it, then when you will try
        // to set it, an assert will occur)
        void setUniformValue(const std::string& variableName, const Vector2& v) const;

        // Set a vector 3 uniform value to this shader (be careful if the uniform is not
        // used in the shader, the compiler will remove it, then when you will try
        // to set it, an assert will occur)
        void setUniformValue(const std::string& variableName, const Vector3& v) const;

        // Set a vector 4 uniform value to this shader (be careful if the uniform is not
        // used in the shader, the compiler will remove it, then when you will try
        // to set it, an assert will occur)
        void setUniformValue(const std::string& variableName, const Vector4 &v) const;


        // Set a 4x4 matrix uniform value to this shader (be careful if the uniform is not
        // used in the shader, the compiler will remove it, then when you will try
        // to set it, an assert will occur)
        void setUniformValue(const std::string& variableName, const Matrix4& matrix , bool transpose = true) const;


        // Set a 3x3 matrix uniform value to this shader (be careful if the uniform is not
        // used in the shader, the compiler will remove it, then when you will try
        // to set it, an assert will occur)
        void setMatrix3x3Uniform(const std::string& variableName, const float* matrix, bool transpose = false) const;


        // Set a 4x4 matrix uniform value to this shader (be careful if the uniform is not
        // used in the shader, the compiler will remove it, then when you will try
        // to set it, an assert will occur)
        void setMatrix4x4Uniform(const std::string& variableName, const float* matrix , bool transpose = false) const;


        // Return true if the needed OpenGL extensions are available
        static bool checkOpenGLExtensions();
};

// Bind the shader
inline void Shader::bind() const
{
    assert(mProgramObjectID != 0);
    glUseProgram(mProgramObjectID);
}

// Unbind the shader
inline void Shader::unbind() const
{
    assert(mProgramObjectID != 0);
    glUseProgram(0);
}

// Return the location of a uniform variable inside a shader program
inline int Shader::getUniformLocation(const std::string& variableName) const
{
	assert(mProgramObjectID != 0);
	int location = glGetUniformLocation(mProgramObjectID, variableName.c_str());
	if (location == -1) {
		std::cerr << "Error in vertex shader " << mFilenameVertexShader << " or in fragment shader"
				<< mFilenameFragmentShader << " : No Uniform variable : " << variableName
				<< std::endl;
	}
	assert(location != -1);
	return location;
}

// Clear the shader
inline void Shader::destroy()
{
	if (mProgramObjectID != 0)
	{
		glDeleteShader(mProgramObjectID);
		mProgramObjectID = 0;
	}
}

// Set a float uniform value to this shader (be careful if the uniform is not
// used in the shader, the compiler will remove it, then when you will try
// to set it, an assert will occur)
inline void Shader::setUniformValue(const std::string& variableName, float value) const
{
    assert(mProgramObjectID != 0);
    glUniform1f(getUniformLocation(variableName), value);
}

// Set an int uniform value to this shader (be careful if the uniform is not
// used in the shader, the compiler will remove it, then when you will try
// to set it, an assert will occur)
inline void Shader::setUniformValue(const std::string& variableName, int value) const
{
    assert(mProgramObjectID != 0);
    glUniform1i(getUniformLocation(variableName), value);
}

// Set a vector 2 uniform value to this shader (be careful if the uniform is not
// used in the shader, the compiler will remove it, then when you will try
// to set it, an assert will occur)
inline void Shader::setUniformValue(const std::string& variableName, const Vector2& v) const
{
    assert(mProgramObjectID != 0);
    glUniform2f(getUniformLocation(variableName), v.x, v.y);
}

// Set a vector 3 uniform value to this shader (be careful if the uniform is not
// used in the shader, the compiler will remove it, then when you will try
// to set it, an assert will occur)
inline void Shader::setUniformValue(const std::string& variableName, const Vector3 &v) const
{
    assert(mProgramObjectID != 0);
    glUniform3f(getUniformLocation(variableName), v.x, v.y, v.z);
}

//// Set a vector 4 uniform value to this shader (be careful if the uniform is not
//// used in the shader, the compiler will remove it, then when you will try
//// to set it, an assert will occur)
//inline void Shader::setVector4Uniform(const std::string& variableName, const Vector4& v) const
//{
//    assert(mProgramObjectID != 0);
//    glUniform4f(getUniformLocation(variableName), v.x, v.y, v.z, v.w);
//}

// Set a 4x4 matrix uniform value to this shader (be careful if the uniform is not
// used in the shader, the compiler will remove it, then when you will try
// to set it, an assert will occur)
inline void Shader::setMatrix3x3Uniform(const std::string& variableName, const float* matrix , bool transpose) const
{
    assert(mProgramObjectID != 0);
    glUniformMatrix3fv(getUniformLocation(variableName), 1, transpose, matrix);
}

// Set a 4x4 matrix uniform value to this shader (be careful if the uniform is not
// used in the shader, the compiler will remove it, then when you will try
// to set it, an assert will occur)
inline void Shader::setMatrix4x4Uniform(const std::string& variableName, const float* matrix, bool transpose) const
{
    assert(mProgramObjectID != 0);
    glUniformMatrix4fv(getUniformLocation(variableName), 1, transpose, matrix);
}



// Return true if the needed OpenGL extensions are available for shaders
inline bool Shader::checkOpenGLExtensions()
{
    // Check that GLSL vertex and fragment shaders are available on the platform
    return (GLEW_VERSION_2_0 || (GLEW_ARB_vertex_shader && GLEW_ARB_fragment_shader));
}


}

#endif /* SHADER_H_ */
