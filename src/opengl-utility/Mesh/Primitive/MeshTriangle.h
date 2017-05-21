/*
 * MeshTriangle.h
 *
 *  Created on: 17 нояб. 2016 г.
 *      Author: wera
 */

#ifndef OPENGL_FRAMEWORK_MESH_PRIMITIVE_MESHTRIANGLE_H_
#define OPENGL_FRAMEWORK_MESH_PRIMITIVE_MESHTRIANGLE_H_

#include "../Mesh.h"


namespace opengl_utility
{

struct Triangle
{
	Vector3 mTriVertex[3];
};


class MeshTriangle: public Mesh
{
  public:
	         MeshTriangle(const Vector3& a ,
			              const Vector3& b ,
						  const Vector3& c);
	virtual ~MeshTriangle();


	void Draw()
	{
		glBegin(GL_LINE_LOOP);
		for (int i = 0; i < mIndicess.size(); ++i)
		{
		   Vector3 v = mVertices[mIndicess[i]];
		   glVertex3f(v.x, v.y, v.z);
		}
		glEnd();
	}

  private:

	bool initialization(const Vector3& a ,
			            const Vector3& b ,
						const Vector3& c);

};

} /* namespace opengl_utility */

#endif /* OPENGL_FRAMEWORK_MESH_PRIMITIVE_MESHTRIANGLE_H_ */
