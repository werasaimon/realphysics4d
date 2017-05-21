/*
 * MeshPlane.h
 *
 *  Created on: 25 сент. 2016 г.
 *      Author: wera
 */

#ifndef MESH_MESHPLANE_H_
#define MESH_MESHPLANE_H_

#include <vector>

#include "../../maths/Vector2.h"
#include "../../maths/Vector3.h"
#include "../Mesh.h"





namespace opengl_utility
{

	class MeshPlane : public Mesh
	{

	   private:

	   // -------------------- Attributes -------------------- //

		float mWidth;
		float mHeight;


	   public:

				 MeshPlane( float width =2 ,
						    float height=2 );

		virtual ~MeshPlane();

		// -------------------- Methods -------------------- //

		void initialization( float w ,  float h )
		{
			mWidth  = w;
			mHeight = h;

			mIndicess.push_back(0);
			mIndicess.push_back(1);
			mIndicess.push_back(2);
			mIndicess.push_back(3);



			Vector3 a(-0.5f * w,  0.0f, -0.5f * h);
			Vector3 b( 0.5f * w,  0.0f, -0.5f * h);
			Vector3 c( 0.5f * w,  0.0f,  0.5f * h);
			Vector3 d(-0.5f * w,  0.0f,  0.5f * h);
			mVertices.push_back(a);
			mVertices.push_back(b);
			mVertices.push_back(c);
			mVertices.push_back(d);



			Vector3 normal = Vector3::Y;
			mNormals.push_back(normal);
			mNormals.push_back(normal);
			mNormals.push_back(normal);
			mNormals.push_back(normal);



			Vector2 uv_a(0.0f ,  0.0f);
			Vector2 uv_b(20.0f,  0.0f);
			Vector2 uv_c(20.0f, 20.0f);
			Vector2 uv_d(0.0f , 20.0f);
			mUVs.push_back(uv_a);
			mUVs.push_back(uv_b);
			mUVs.push_back(uv_c);
			mUVs.push_back(uv_d);

		}



	float getHeight() const
	{
		return mHeight;
	}

	float getWidth() const
	{
		return mWidth;
	}
};

}
 /* namespace opengl_utility */

#endif /* MESH_MESHPLANE_H_ */
