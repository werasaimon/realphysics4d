/*
 * MeshTriangle.cpp
 *
 *  Created on: 17 нояб. 2016 г.
 *      Author: wera
 */

#include "MeshTriangle.h"

namespace utility_engine
{


MeshTriangle::MeshTriangle(const Vector3& a,
		                   const Vector3& b,
		                   const Vector3& c)
:Mesh(PRIMITIVE_MESH_TRIANGLE)
{
	initialization(a,b,c);
}



} /* namespace utility_engine */


