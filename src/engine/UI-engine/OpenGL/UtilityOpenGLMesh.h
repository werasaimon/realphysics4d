/*
 * UtilityOpenGLMesh.h
 *
 *  Created on: 3 февр. 2017 г.
 *      Author: wera
 */

#ifndef OPENGL_UTILITY_OPENGL_UTILITYOPENGLMESH_H_
#define OPENGL_UTILITY_OPENGL_UTILITYOPENGLMESH_H_

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <vector>

#include "../Mesh/Mesh.h"
//#include "../Mesh/CollectionMesh.h"
#include "../Texture/Texture2D.h"


namespace utility_engine
{

class UtilityOpenGLMesh
{


 public:

	static void RenderMesh( Mesh *meshe /*, Shader mShaderProgram*/)
	{

		glPushMatrix();

		//meshe->getTexture2D().bind();

		int level = 0;
		for( auto it = meshe->getTextures().begin(); it != meshe->getTextures().end(); ++it)
		{
			it->second.bind();
			it->second.setLayer(level);
		}


		/***************************************/
		//mPhongShader.setMatrix4x4Uniform("modelToWorldMatrix", meshe.getTransformMatrix());
		glEnableClientState(GL_VERTEX_ARRAY);
		glEnableClientState(GL_NORMAL_ARRAY);
		//if (meshe.hasTexture())
		{
			glEnableClientState(GL_TEXTURE_COORD_ARRAY);
		}
		//mPhongShader.setIntUniform( "isTexture" , 1);

		glVertexPointer(3, GL_FLOAT, 0, meshe->getVerticesPointer());
		glNormalPointer(GL_FLOAT, 0, meshe->getNormalsPointer());
		//if(mMesh3.hasTexture())
		{
			glTexCoordPointer(2, GL_FLOAT, 0, meshe->getUVTextureCoordinatesPointer());
		}

		// for (uint i=0; i < mMesh3.getNbParts(); i++ )
		{
			glDrawElements(GL_TRIANGLES, meshe->getIndicess().size() , GL_UNSIGNED_INT, (void*)&meshe->getIndicess()[0]);
		}


		glDisableClientState(GL_NORMAL_ARRAY);
		glDisableClientState(GL_VERTEX_ARRAY);
		//if (meshe.hasTexture())
		{
			glDisableClientState(GL_TEXTURE_COORD_ARRAY);
		}
		/***************************************/

		for( auto it = meshe->getTextures().begin(); it != meshe->getTextures().end(); ++it)
		{
			it->second.unbind();
		}


		glPopMatrix();

	}



//	static void RenderCollectionMesh( CollectionMesh &meshe , Shader mShaderProgram )
//	{
//
//	}

};

} /* namespace opengl_utility */

#endif /* OPENGL_UTILITY_OPENGL_UTILITYOPENGLMESH_H_ */
