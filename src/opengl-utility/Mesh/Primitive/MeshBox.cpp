/*
 * MeshBox.cpp
 *
 *  Created on: 6 февр. 2017 г.
 *      Author: wera
 */

#include "MeshBox.h"

namespace opengl_utility
{

MeshBox::MeshBox( Vector3 HalfSize )
:Mesh(PRIMITIVE_MESH_BOX) , mHalfSize(HalfSize)
{

}

MeshBox::~MeshBox()
{
	// TODO Auto-generated destructor stub
}

} /* namespace opengl_utility */
