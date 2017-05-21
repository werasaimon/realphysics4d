/*
 * GroupObjectsMesh.h
 *
 *  Created on: 6 мар. 2017 г.
 *      Author: wera
 */

#ifndef SRC_INTERFACE_GROUPOBJECTSMESH_H_
#define SRC_INTERFACE_GROUPOBJECTSMESH_H_

#include <vector>

#include "../opengl-utility/opengl-utility.h"
#include "../physics-engine/realphysics.h"


struct ElementMesh
{
	// -------------------- Attributes -------------------- //
	opengl_utility::Mesh     *mMesh;
    opengl_utility::Matrix4   mInitTransform;

    ElementMesh( opengl_utility::Mesh *mesh , opengl_utility::Matrix4 initTransform )
	: mMesh(mesh) , mInitTransform(initTransform)
	{

	}

	void updateTransformMatrix( opengl_utility::Matrix4 matrix )
	{
		mMesh->setToIdentity();
		mMesh->setTransformMatrix(mInitTransform * matrix);
	}

};

class GroupMesh
{

	private:

    // -------------------- Attributes -------------------- //
	std::vector<ElementMesh> mElements;

   public:
	         GroupMesh();
	virtual ~GroupMesh();


	void addInGroup( opengl_utility::Mesh *mesh , opengl_utility::Matrix4 initTransform )
	{
		mElements.push_back( ElementMesh(mesh , initTransform)  );
	}

	std::vector<ElementMesh>& getElements()
    {
		return mElements;
	}
};








#endif /* SRC_INTERFACE_GROUPOBJECTSMESH_H_ */
