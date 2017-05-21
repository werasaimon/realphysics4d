/*
 * MeshTriangle.cpp
 *
 *  Created on: 17 нояб. 2016 г.
 *      Author: wera
 */

#include "MeshTriangle.h"

namespace opengl_utility
{


MeshTriangle::MeshTriangle(const Vector3& a,
		                   const Vector3& b,
		                   const Vector3& c)
:Mesh(PRIMITIVE_MESH_TRIANGLE)
{
	initialization(a,b,c);
}


MeshTriangle::~MeshTriangle()
{
}



bool MeshTriangle::initialization(const Vector3& a,
	                              const Vector3& b,
				 			      const Vector3& c)
{

	mVertices.push_back(a);
	mVertices.push_back(b);
	mVertices.push_back(c);


	Vector3 normal_face = ((b-a).cross(c-a)).normalize();

	mNormals.push_back(normal_face);
	mNormals.push_back(normal_face);
	mNormals.push_back(normal_face);

	mUVs.push_back(Vector2(1,0));
	mUVs.push_back(Vector2(1,1));
	mUVs.push_back(Vector2(0,1));

	mIndicess.push_back(0);
	mIndicess.push_back(1);
	mIndicess.push_back(2);

	return true;
}


} /* namespace opengl_utility */


