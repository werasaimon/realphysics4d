/*
 * MeshPlane.cpp
 *
 *  Created on: 25 сент. 2016 г.
 *      Author: wera
 */

#include "MeshPlane.h"

using namespace opengl_utility;

MeshPlane::MeshPlane(float width, float height)
:Mesh(PRIMITIVE_MESH_PLANE)
{
	initialization( width , height );
}

MeshPlane::~MeshPlane()
{
	// TODO Auto-generated destructor stub
}


/* namespace opengl_utility */
