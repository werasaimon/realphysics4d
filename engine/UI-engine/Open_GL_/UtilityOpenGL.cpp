#include "UtilityOpenGL.h"
#include "../Mesh/Mesh.h"

namespace utility_engine
{


void UtilityOpenGL::DrawMesh( Mesh* mesh , QOpenGLShaderProgram *program)
{

    GLUtilityGeometry* OpenGLUtilGeometry = NULL;
    OpenGLUtilGeometry = new GLUtilityGeometry(mesh);

    int level = 0;
    for(auto it = mesh->mTextures.begin(); it != mesh->mTextures.end(); ++it)
    {
        it->second.bind();
        it->second.setLayer(++level);
    }

    OpenGLUtilGeometry->drawGeometry(program);


    for(auto it = mesh->mTextures.begin(); it != mesh->mTextures.end(); ++it)
    {
        it->second.unbind();
    }

    delete OpenGLUtilGeometry;
}




#ifdef __ANDROID__
#elif defined(WIN32) || defined(__linux__)

void UtilityOpenGL::DrawMesh( Mesh* mesh )
{
    assert(mesh);

    //------------------ sempler textures --------------------//
    int level = 0;
    for(auto it = mesh->mTextures.begin(); it != mesh->mTextures.end(); ++it)
    {
        it->second.bind();
        it->second.setLayer(++level);
    }

    //------------------ render geometry ---------------------//
    glPushMatrix();
    glColor3f(1,1,1);
    glMultMatrixf(mesh->mTransformMatrix.getTranspose().dataBlock());
    for (int i = 0; i < mesh->mIndicess.size(); i+=3)
    {
        uint index_a = mesh->mIndicess[i+0];
        uint index_b = mesh->mIndicess[i+1];
        uint index_c = mesh->mIndicess[i+2];

        Vector3 a = mesh->mVertices[index_a];
        Vector3 b = mesh->mVertices[index_b];
        Vector3 c = mesh->mVertices[index_c];

        Vector2 UVa = mesh->mUVs[index_a];
        Vector2 UVb = mesh->mUVs[index_b];
        Vector2 UVc = mesh->mUVs[index_c];

        Color   color_a = mesh->mColors[index_a];
        Color   color_b = mesh->mColors[index_b];
        Color   color_c = mesh->mColors[index_c];

        Vector3 face_a = mesh->mNormals[index_a];
        Vector3 face_b = mesh->mNormals[index_b];
        Vector3 face_c = mesh->mNormals[index_c];


        //glBegin(GL_TRIANGLES);
        glBegin(GL_LINE_LOOP);
        glVertex3f(a.x , a.y , a.z); glTexCoord2f(UVa.x, UVa.y); glNormal3f( face_a.x , face_a.y , face_a.z ); glColor4f(color_a.r , color_a.g , color_a.b , color_a.a);
        glVertex3f(b.x , b.y , b.z); glTexCoord2f(UVb.x, UVb.y); glNormal3f( face_b.x , face_b.y , face_b.z ); glColor4f(color_b.r , color_b.g , color_b.b , color_b.a);
        glVertex3f(c.x , c.y , c.z); glTexCoord2f(UVc.x, UVc.y); glNormal3f( face_c.x , face_c.y , face_c.z ); glColor4f(color_c.r , color_c.g , color_c.b , color_c.a);
        glEnd();

    }
    glPopMatrix();


    //------------- realase sempler textures ------------------//
    for(auto it = mesh->mTextures.begin(); it != mesh->mTextures.end(); ++it)
    {
        it->second.unbind();
    }

}

#endif




}
