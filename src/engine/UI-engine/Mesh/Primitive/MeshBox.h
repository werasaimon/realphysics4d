/*
 * MeshBox.h
 *
 *  Created on: 6 февр. 2017 г.
 *      Author: wera
 */

#ifndef OPENGL_UTILITY_MESH_PRIMITIVE_MESHBOX_H_
#define OPENGL_UTILITY_MESH_PRIMITIVE_MESHBOX_H_

#include "../Mesh.h"

namespace utility_engine
{

class MeshBox: public Mesh
{
public:
	MeshBox( Vector3 HalfSize );
	virtual ~MeshBox();



private:

	void initialization( const Vector3& halfSize )
	{

		const float size = 1.0;

		// Vertices
		const Vector3 vertices[] =
		{
				// front
				Vector3(-size, size, size),
				Vector3( size, size, size),
				Vector3( size,-size, size),
				Vector3(-size,-size, size),
				// back
				Vector3( size, size,-size),
				Vector3(-size, size,-size),
				Vector3(-size,-size,-size),
				Vector3( size,-size,-size),
				// top
				Vector3(-size, size,-size),
				Vector3( size, size,-size),
				Vector3( size, size, size),
				Vector3(-size, size, size),
				// bottom
				Vector3( size,-size,-size),
				Vector3(-size,-size,-size),
				Vector3(-size,-size, size),
				Vector3( size,-size, size),
				// left
				Vector3(-size, size,-size),
				Vector3(-size, size, size),
				Vector3(-size,-size, size),
				Vector3(-size,-size,-size),
				// right
				Vector3( size, size, size),
				Vector3( size, size,-size),
				Vector3( size,-size,-size),
				Vector3( size,-size, size)
		};


		// textures UV
		const Vector2 vUVs[] =
		{
				// front
				Vector2(0.0f,1.0f),
				Vector2(1.0f,1.0f),
				Vector2(1.0f,0.0f),
				Vector2(0.0f,0.0f),
				// back
				Vector2(0.0f,1.0f),
				Vector2(1.0f,1.0f),
				Vector2(1.0f,0.0f),
				Vector2(0.0f,0.0f),
				// top
				Vector2(0.0f,1.0f),
				Vector2(1.0f,1.0f),
				Vector2(1.0f,0.0f),
				Vector2(0.0f,0.0f),
				// bottom
				Vector2(0.0f,1.0f),
				Vector2(1.0f,1.0f),
				Vector2(1.0f,0.0f),
				Vector2(0.0f,0.0f),
				// left
				Vector2(0.0f,1.0f),
				Vector2(1.0f,1.0f),
				Vector2(1.0f,0.0f),
				Vector2(0.0f,0.0f),
				// right
				Vector2(0.0f,1.0f),
				Vector2(1.0f,1.0f),
				Vector2(1.0f,0.0f),
				Vector2(0.0f,0.0f),
		};


		// Normals
		const Vector3 normals[] =
		{
				Vector3(0.0f, 0.0f, 1.0f),
				Vector3(0.0f, 0.0f, 1.0f),
				Vector3(0.0f, 0.0f, 1.0f),
				Vector3(0.0f, 0.0f, 1.0f),
				// back
				Vector3(0.0f, 0.0f,-1.0f),
				Vector3(0.0f, 0.0f,-1.0f),
				Vector3(0.0f, 0.0f,-1.0f),
				Vector3(0.0f, 0.0f,-1.0f),
				// top
				Vector3(0.0f, 1.0f, 0.0f),
				Vector3(0.0f, 1.0f, 0.0f),
				Vector3(0.0f, 1.0f, 0.0f),
				Vector3(0.0f, 1.0f, 0.0f),
				// bottom
				Vector3(0.0f,-1.0f, 0.0f),
				Vector3(0.0f,-1.0f, 0.0f),
				Vector3(0.0f,-1.0f, 0.0f),
				Vector3(0.0f,-1.0f, 0.0f),
				// left
				Vector3(-1.0f, 0.0f, 0.0f),
				Vector3(-1.0f, 0.0f, 0.0f),
				Vector3(-1.0f, 0.0f, 0.0f),
				Vector3(-1.0f, 0.0f, 0.0f),
				// right
				Vector3(1.0f, 0.0f, 0.0f),
				Vector3(1.0f, 0.0f, 0.0f),
				Vector3(1.0f, 0.0f, 0.0f),
				Vector3(1.0f, 0.0f, 0.0f)
		};

		const Color colors[] =
		{
				Color( 1 , 1 , 1 , 1 ),
				Color( 1 , 1 , 1 , 1 ),
				Color( 1 , 1 , 1 , 1 ),
				Color( 1 , 1 , 1 , 1 ),

				Color( 1 , 1 , 1 , 1 ),
				Color( 1 , 1 , 1 , 1 ),
				Color( 1 , 1 , 1 , 1 ),
				Color( 1 , 1 , 1 , 1 ),

				Color( 1 , 1 , 1 , 1 ),
				Color( 1 , 1 , 1 , 1 ),
				Color( 1 , 1 , 1 , 1 ),
				Color( 1 , 1 , 1 , 1 ),

				Color( 1 , 1 , 1 , 1 ),
				Color( 1 , 1 , 1 , 1 ),
				Color( 1 , 1 , 1 , 1 ),
				Color( 1 , 1 , 1 , 1 ),

				Color( 1 , 1 , 1 , 1 ),
				Color( 1 , 1 , 1 , 1 ),
				Color( 1 , 1 , 1 , 1 ),
				Color( 1 , 1 , 1 , 1 ),

				Color( 1 , 1 , 1 , 1 ),
				Color( 1 , 1 , 1 , 1 ),
				Color( 1 , 1 , 1 , 1 ),
				Color( 1 , 1 , 1 , 1 )

		};


		// indices
		const uint indices[] =
		{
				0, 3, 1,  1, 3, 2,  // front
				4, 7, 5,  5, 7, 6,  // back
				8,11, 9,  9,11,10,  // top
				12,15,13, 13,15,14, // bottom
				16,19,17, 17,19,18, // left
				20,23,21, 21,23,22  // right
		};



		//-------------------- init geometry -----------------------//

		for (uint i = 0; i < sizeof(vertices) / sizeof(Vector3); ++i)
		{
			mVertices.push_back( vertices[i] * halfSize * 0.5f );
			mUVs.push_back( vUVs[i] );
		}

		for (uint i = 0; i < sizeof(normals) / sizeof(Vector3); ++i)
		{
			mNormals.push_back( normals[i] );
		}

		for (uint i = 0; i < (sizeof(colors)/sizeof(Color)); ++i)
		{
			mColors.push_back(colors[i]);
		}


		for (uint i = 0; i < (sizeof(indices)/sizeof(uint)); ++i)
		{
			mIndicess.push_back(indices[i]);
		}


	}


	Vector3 mHalfSize;
};

} /* namespace opengl_utility */

#endif /* OPENGL_UTILITY_MESH_PRIMITIVE_MESHBOX_H_ */
