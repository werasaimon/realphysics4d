/*
 * CollectionMesh.h
 *
 *  Created on: 2 февр. 2017 г.
 *      Author: wera
 */

#ifndef OPENGL_UTILITY_MESH_COLLECTIONMESH_H_
#define OPENGL_UTILITY_MESH_COLLECTIONMESH_H_

#include <vector>
#include <set>

#include "../OpenGL/Shader.h"
#include "../OpenGL/UtilityOpenGLMesh.h"

#include "Object3D.h"
#include "Mesh.h"

namespace opengl_utility
{

class CollectionMesh : public Object3D
{


 private:

	// -------------------- Attributes -------------------- //
	std::vector<Mesh*> mMeshes;
	std::vector<opengl_utility::Matrix4> mInitMatrix;


 public:

	         CollectionMesh();
	virtual ~CollectionMesh();


	/// Add new mesh object
	void addMesh( Mesh* mesh )
	{
		mMeshes.push_back(mesh);
		mInitMatrix.push_back( mesh->getTransformMatrix() );
	}

	const std::vector<Mesh*>& getMeshes() const
	{
		return mMeshes;
	}

	const std::vector<opengl_utility::Matrix4>& getInitMatrix() const
	{
		return mInitMatrix;
	}
};


} /* namespace opengl_utility */

#endif /* OPENGL_UTILITY_MESH_COLLECTIONMESH_H_ */
