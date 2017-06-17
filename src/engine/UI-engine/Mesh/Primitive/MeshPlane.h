#ifndef MESH_MESHPLANE_H_
#define MESH_MESHPLANE_H_

#include <vector>

#include "../../maths/Vector2.h"
#include "../../maths/Vector3.h"
#include "../Mesh.h"





namespace utility_engine
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


    float getHeight() const
    {
      return mHeight;
    }

    float getWidth() const
    {
      return mWidth;
    }


  private:

    // -------------------- Methods -------------------- //
    void initialization( float w ,  float h )
    {

			mWidth  = w;
			mHeight = h;


			const float size = 1.0;

			const Vector3 vertices[] =
			{
					Vector3(-size, 0, -size),
					Vector3( size, 0, -size),
					Vector3( size, 0,  size),
					Vector3(-size, 0,  size)
			};


			const Vector2 vUVs[] =
			{
					Vector2(0.0f,1.0f),
					Vector2(1.0f,1.0f),
					Vector2(1.0f,0.0f),
					Vector2(0.0f,0.0f)
			};


			const Vector3 normals[] =
			{
					Vector3( 0.0f, 1.0f, 0.0f),
					Vector3( 0.0f, 1.0f, 0.0f),
					Vector3( 0.0f, 1.0f, 0.0f),
					Vector3( 0.0f, 1.0f, 0.0f)
			};



			const Color colors[]
							   {
									   Color( 1 , 1 , 1 , 1 ) ,
									   Color( 1 , 1 , 1 , 1 ) ,
									   Color( 1 , 1 , 1 , 1 ) ,
									   Color( 1 , 1 , 1 , 1 )
							   };

			const uint indices[] =
			{
					0, 3, 1,  1, 3, 2
			};




			//-------------------- init geometry -----------------------//

			for (uint i = 0; i < sizeof(vertices) / sizeof(Vector3); ++i)
			{
				mVertices.push_back( vertices[i] * Vector3( mWidth , 0.0 , mHeight ) );
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




  };

}
 /* namespace opengl_utility */

#endif /* MESH_MESHPLANE_H_ */
