/*
 * Shader.cpp
 *
 *  Created on: 27.08.2016
 *      Author: wera
 */

#include "Shader.h"

#include <stdio.h>
#include <cstdio>
#include <iostream>
#include <string.h>

#include <cassert>
#include <fstream>


#include "../maths/Matrix4.h"
#include "../maths/Vector4.h"



using namespace utility_engine;



void GLShaderProgram::UniformValue(const char *name , int value)
{
     setUniformValue( name ,  value );
}

void GLShaderProgram::UniformValue(const char *name, const float& value)
{
    setUniformValue( name ,  value );
}

void GLShaderProgram::UniformValue(const char *name, const Vector2 &value)
{
    setUniformValue( name , QVector2D(value.x , value.y) );
}

void GLShaderProgram::UniformValue(const char *name, const Vector3& value)
{
    setUniformValue( name , QVector3D(value.x , value.y , value.z) );
}

void GLShaderProgram::UniformValue(const char *name, const Vector4 &value)
{
    setUniformValue( name , QVector4D(value.x , value.y , value.z , value.w ) );
}

void GLShaderProgram::UniformValue(const char *name, const Matrix4& value)
{
    QMatrix4x4 mat( value.m[0][0] , value.m[0][1] , value.m[0][2] , value.m[0][3] ,
                    value.m[1][0] , value.m[1][1] , value.m[1][2] , value.m[1][3] ,
                    value.m[2][0] , value.m[2][1] , value.m[2][2] , value.m[2][3] ,
                    value.m[3][0] , value.m[3][1] , value.m[3][2] , value.m[3][3] );

     setUniformValue( name , mat );
}
