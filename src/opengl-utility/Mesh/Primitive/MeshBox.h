/*
 * MeshBox.h
 *
 *  Created on: 6 февр. 2017 г.
 *      Author: wera
 */

#ifndef OPENGL_UTILITY_MESH_PRIMITIVE_MESHBOX_H_
#define OPENGL_UTILITY_MESH_PRIMITIVE_MESHBOX_H_

#include "../Mesh.h"

namespace opengl_utility
{

class MeshBox: public Mesh
{
  public:
	         MeshBox( Vector3 HalfSize );
	virtual ~MeshBox();


	void initialization( const Vector3& halfSize )
	{

	}

  private:

	Vector3 mHalfSize;
};

} /* namespace opengl_utility */

#endif /* OPENGL_UTILITY_MESH_PRIMITIVE_MESHBOX_H_ */
