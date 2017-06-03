/*
 * MeshTriangle.h
 *
 *  Created on: 17 нояб. 2016 г.
 *      Author: wera
 */

#ifndef OPENGL_FRAMEWORK_MESH_PRIMITIVE_MESHTRIANGLE_H_
#define OPENGL_FRAMEWORK_MESH_PRIMITIVE_MESHTRIANGLE_H_

#include "../Mesh.h"


namespace utility_engine
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


  private:

	bool initialization(const Vector3& a,
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

        mColors.push_back( Color(1,1,1,1) );
        mColors.push_back( Color(1,1,1,1) );
        mColors.push_back( Color(1,1,1,1) );


        std::vector<uint> indx;
        indx.push_back(0);
        indx.push_back(1);
        indx.push_back(2);
        mIndices.push_back(indx);

		return true;
	}

};

} /* namespace utility_engine */

#endif /* OPENGL_FRAMEWORK_MESH_PRIMITIVE_MESHTRIANGLE_H_ */
