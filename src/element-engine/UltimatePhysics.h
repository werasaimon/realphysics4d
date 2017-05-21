/*
 * UltimatePhysics.h
 *
 *  Created on: 6 мар. 2017 г.
 *      Author: wera
 */

#ifndef SRC_ELEMENT_ENGINE_ULTIMATEPHYSICS_H_
#define SRC_ELEMENT_ENGINE_ULTIMATEPHYSICS_H_


#include "../element-engine/GroupMesh.h"
#include "../opengl-utility/opengl-utility.h"
#include "../physics-engine/realphysics.h"

namespace
{

static real_physics::Transform Matrix4ConvertToTransform( opengl_utility::Matrix4 matrix4 )
{
	real_physics::Transform transform;
	float m[16];
    matrix4.getTranspose().getDataValue(m);
	transform.setFromOpenGL(m);
	return transform;
}


static opengl_utility::Matrix4 TransformConvertToMatrix4( real_physics::Transform transform )
{
	opengl_utility::Matrix4 matrix;
	float m[16];
	transform.getOpenGLMatrix(m);
	matrix.setDataValue(m);
	return matrix.getTranspose();
}


static std::vector<real_physics::Vector3> MeshConvertToVertexes( const opengl_utility::Mesh *mesh )
{
	  std::vector<real_physics::Vector3> vecrtices;
	  for (unsigned int i = 0; i < mesh->getNbVertices(); ++i)
	  {
		  vecrtices.push_back(real_physics::Vector3( mesh->getVertices()[i].x ,
													 mesh->getVertices()[i].y ,
													 mesh->getVertices()[i].z ));
	  }
	  return vecrtices;
}


}



class UltimatePhysics
{

  private:

	 // -------------------- Attributes -------------------- //
	real_physics::rpCollisionBody*    mPhysicsDriver;
	GroupMesh                         mGroupMesh;

  public:
	         UltimatePhysics(real_physics::rpCollisionBody* _physBody);
	virtual ~UltimatePhysics();


	void addConnectMeshToShape( opengl_utility::Mesh *mesh , real_physics::rpCollisionShape* shape , real_physics::scalar mass = 1.0 )
	{
		assert( mass >= 1.0 );
		mPhysicsDriver->addCollisionShape( shape, mass, Matrix4ConvertToTransform(mesh->getTransformMatrix()) );
		mGroupMesh.addInGroup(mesh , mesh->getTransformMatrix());
	}


	void update()
	{
		for (unsigned int i = 0; i < mGroupMesh.getElements().size(); ++i)
		{
			opengl_utility::Matrix4 matrix = TransformConvertToMatrix4(mPhysicsDriver->getTransform());
			mGroupMesh.getElements()[i].updateTransformMatrix(matrix);
		}
	}



};

#endif /* SRC_ELEMENT_ENGINE_ULTIMATEPHYSICS_H_ */
