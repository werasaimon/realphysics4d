#ifndef MESH_H
#define MESH_H

// Libraries
#include <string>
#include <vector>
#include <map>
#include "../../maths/definitions.h"
#include "../../maths/Vector2.h"
#include "../../maths/Vector3.h"
#include "../../maths/Color.h"
#include "../../Texture/Texture2D.h"
#include "../../Geometry/Object/Object3D.h"

#include "GLUtilityGeometry.h"


namespace utility_engine
{

typedef unsigned int uint;


enum Meshype
{
  DEFAULT_MESH,
  MESH_LOAD_FILE,
  PRIMITIVE_MESH_BOX,
  PRIMITIVE_MESH_PLANE,
  PRIMITIVE_MESH_TRIANGLE
};

// Class Mesh
// This class represents a 3D triangular mesh
// object that can be loaded from an OBJ file for instance.
class Mesh : public Object3D
{

	friend class MeshReaderFile3DS;

    protected:

	// -------------------- Attributes -------------------- //

	    Meshype mType;

        // A triplet of vertex indices for each triangle
	    std::vector<std::vector<uint> > mIndices;
	    std::vector<uint>               mIndicess;

        // Vertices coordinates (local space)
        std::vector<Vector3> mVertices;

        // Normals coordinates
        std::vector<Vector3> mNormals;

        // Tangents coordinates
        std::vector<Vector3> mTangents;

        // Color for each vertex
        std::vector<Color>   mColors;

        // UV texture coordinates
        std::vector<Vector2> mUVs;

        // Textures of the mesh (one for each part of the mesh)
        std::map<uint, Texture2D> mTextures;


        // Geimetry of the render OpenGL on shader
        GLUtilityGeometry    *mOpenGLUtilGeometry;


    public:


       /**/
        //initilisation buffer VBO OpenGL !!!
        void initilisationGL();

        //draw render buffer openGL VBO OpenGL
        void DrawOpenGL( QOpenGLShaderProgram *program );
       /**/



        /**/

#ifdef __ANDROID__
#elif defined(WIN32) || defined(__linux__)

        void Draw()
        {

            //------------------ sempler textures --------------------//
            int level = 0;
            for(auto it = mTextures.begin(); it != mTextures.end(); ++it)
            {
                it->second.bind();
                it->second.setLayer(++level);
            }

            //------------------ render geometry ---------------------//
            glPushMatrix();
            glColor3f(1,1,1);
            glMultMatrixf(mTransformMatrix.getTranspose().dataBlock());
            for (int i = 0; i < mIndicess.size(); i+=3)
            {
                uint index_a = mIndicess[i+0];
                uint index_b = mIndicess[i+1];
                uint index_c = mIndicess[i+2];

                Vector3 a = mVertices[index_a];
                Vector3 b = mVertices[index_b];
                Vector3 c = mVertices[index_c];

                Vector2 UVa = mUVs[index_a];
                Vector2 UVb = mUVs[index_b];
                Vector2 UVc = mUVs[index_c];

                Color   color_a = mColors[index_a];
                Color   color_b = mColors[index_b];
                Color   color_c = mColors[index_c];

                Vector3 face_a = mNormals[index_a];
                Vector3 face_b = mNormals[index_b];
                Vector3 face_c = mNormals[index_c];



                glBegin(GL_TRIANGLES);
                glVertex3f(a.x , a.y , a.z); glTexCoord2f(UVa.x, UVa.y); glNormal3f( face_a.x , face_a.y , face_a.z ); glColor4f(color_a.r , color_a.g , color_a.b , color_a.a);
                glVertex3f(b.x , b.y , b.z); glTexCoord2f(UVb.x, UVb.y); glNormal3f( face_b.x , face_b.y , face_b.z ); glColor4f(color_b.r , color_b.g , color_b.b , color_b.a);
                glVertex3f(c.x , c.y , c.z); glTexCoord2f(UVc.x, UVc.y); glNormal3f( face_c.x , face_c.y , face_c.z ); glColor4f(color_c.r , color_c.g , color_c.b , color_c.a);
                glEnd();

            }
            glPopMatrix();


            //------------- realase sempler textures ------------------//
            for(auto it = mTextures.begin(); it != mTextures.end(); ++it)
            {
                it->second.unbind();
            }

        }

#endif

        /**/


        // -------------------- Methods -------------------- //

        // Constructor
        Mesh(Meshype type);

        // Destructor
        virtual ~Mesh();



        // Destroy the mesh
        void destroy();

        // Compute the normals of the mesh
        void calculateNormals();

        // Compute the tangents of the mesh
        void calculateTangents();

        // Calculate the bounding box of the mesh
        void calculateBoundingBox(Vector3& min, Vector3& max) const;

        // Scale of vertices of the mesh using a given factor
        void scaleVertices(float factor);


        // Return the type mesh
        Meshype getType() const { return mType; }

        // Return the number of triangles
        uint getNbFaces(uint part = 0) const;

        // Return the number of vertices
        uint getNbVertices() const;

        // Return the number of parts in the mesh
        uint getNbParts() const;

        // Return a reference to the vertices
        const std::vector<Vector3>& getVertices() const;

        // Set the vertices of the mesh
        void setVertices(std::vector<Vector3>& vertices);

        // Return a reference to the normals
        const std::vector<Vector3>& getNormals() const;

        // set the normals of the mesh
        void setNormals(std::vector<Vector3>& normals);

        // Return a reference to the UVs
        const std::vector<Vector2>& getUVs() const;

        // Set the UV texture coordinates of the mesh
        void setUVs(std::vector<Vector2>& uvs);

        // Return a reference to the vertex indices
        const std::vector<uint>& getIndices(uint part = 0) const;

        // Set the vertices indices of the mesh
        void setIndices(std::vector<std::vector<uint> >& indices);

        // Return the coordinates of a given vertex
        const Vector3& getVertex(uint i) const;

        // Set the coordinates of a given vertex
        void setVertex(uint i, const Vector3& vertex);

        // Return the coordinates of a given normal
        const Vector3& getNormal(uint i) const;

        // Set the coordinates of a given normal
        void setNormal(uint i, const Vector3& normal);

        // Return the color of a given vertex
        const Color& getColor(uint i) const;

        // Set the color of a given vertex
        void setColor(uint i, const Color& color);

        // Set a color to all the vertices
        void setColorToAllVertices(const Color& color);

        // Return the UV of a given vertex
        const Vector2& getUV(uint i) const;

        // Set the UV of a given vertex
        void setUV(uint i, const Vector2& uv);

        // Return the vertex index of the ith (i=0,1,2) vertex of a given face
        uint getVertexIndexInFace(uint faceIndex, uint i, uint part = 0) const;

        // Return true if the mesh has normals
        bool hasNormals() const;

        // Return true if the mesh has tangents
        bool hasTangents() const;

        // Return true if the mesh has vertex colors
        bool hasColors() const;

        // Return true if the mesh has UV texture coordinates
        bool hasUVTextureCoordinates() const;

        // Return true if the mesh has a texture for a given part of the mesh and if it
        // also have texture coordinates
        bool hasTextureForPart(uint part = 0) const;

        // Return true if the mesh has a texture (and texture coordinates) for at least one
        // part of the mesh
        bool hasTexture() const;

        // Return a pointer to the vertices data
        void* getVerticesPointer();

        // Return a pointer to the normals data
        void* getNormalsPointer();

        // Return a pointer to the colors data
        void* getColorsPointer();

        // Return a pointer to the tangents data
        void* getTangentsPointer();

        // Return a pointer to the UV texture coordinates data
        void* getUVTextureCoordinatesPointer();

        // Return a pointer to the vertex indicies data
        void* getIndicesPointer(uint part = 0);

        // Return a reference to a texture of the mesh
        Texture2D &getTexture(uint part = 0);

        // Set a texture to a part of the mesh
        void setTexture(Texture2D &texture, uint part = 0);

        // Return a index to the vertex indicies data
        const std::vector<std::vector<uint> > getIndices() const { return mIndices; }

        // Return a index to the vertex indiciess data
	    const std::vector<uint>& getIndicess() const { return mIndicess; }




};

// Return the number of triangles
inline uint Mesh::getNbFaces(uint part) const
{
    return mIndices[part].size() / 3;
}

// Return the number of vertices
inline uint Mesh::getNbVertices() const
{
    return mVertices.size();
}

// Return the number of parts in the mesh
inline uint Mesh::getNbParts() const
{
    return mIndices.size();
}

// Return a reference to the vertices
inline const std::vector<Vector3>& Mesh::getVertices() const
{
    return mVertices;
}

// Set the vertices of the mesh
inline void Mesh::setVertices(std::vector<Vector3>& vertices)
{
    mVertices = vertices;
}

// Return a reference to the normals
inline const std::vector<Vector3>& Mesh::getNormals() const
{
    return mNormals;
}

// set the normals of the mesh
inline void Mesh::setNormals(std::vector<Vector3>& normals)
{
    mNormals = normals;
}

// Return a reference to the UVs
inline const std::vector<Vector2>& Mesh::getUVs() const
{
    return mUVs;
}

// Set the UV texture coordinates of the mesh
inline void Mesh::setUVs(std::vector<Vector2>& uvs)
{
    mUVs = uvs;
}

// Return a reference to the vertex indices
inline const std::vector<uint>& Mesh::getIndices(uint part) const
{
    return mIndices[part];
}

// Set the vertices indices of the mesh
inline void Mesh::setIndices(std::vector<std::vector<uint> >& indices)
{
    mIndices = indices;
}

// Return the coordinates of a given vertex
inline const Vector3& Mesh::getVertex(uint i) const
{
    assert(i < getNbVertices());
    return mVertices[i];
}

// Set the coordinates of a given vertex
inline void Mesh::setVertex(uint i, const Vector3& vertex)
{
    assert(i < getNbVertices());
    mVertices[i] = vertex;
}

// Return the coordinates of a given normal
inline const Vector3& Mesh::getNormal(uint i) const
{
    assert(i < getNbVertices());
    return mNormals[i];
}

// Set the coordinates of a given normal
inline void Mesh::setNormal(uint i, const Vector3& normal)
{
    assert(i < getNbVertices());
    mNormals[i] = normal;
}

// Return the color of a given vertex
inline const Color& Mesh::getColor(uint i) const
{
    assert(i < getNbVertices());
    return mColors[i];
}

// Set the color of a given vertex
inline void Mesh::setColor(uint i, const Color& color)
{

    // If the color array does not have the same size as
    // the vertices array
    if (mColors.size() != mVertices.size())
    {

        // Create the color array with the same size
        mColors = std::vector<Color>(mVertices.size());
    }

    mColors[i] = color;
}

// Set a color to all the vertices
inline void Mesh::setColorToAllVertices(const Color& color)
{

    // If the color array does not have the same size as
    // the vertices array
    if (mColors.size() != mVertices.size())
    {

        // Create the color array with the same size
        mColors = std::vector<Color>(mVertices.size());
    }

    for (size_t v=0; v<mVertices.size(); v++)
    {
        mColors[v] = color;
    }
}

// Return the UV of a given vertex
inline const Vector2& Mesh::getUV(uint i) const
{
    assert(i < getNbVertices());
    return mUVs[i];
}

// Set the UV of a given vertex
inline void Mesh::setUV(uint i, const Vector2& uv)
{
    assert(i < getNbVertices());
    mUVs[i] = uv;
}

// Return the vertex index of the ith (i=0,1,2) vertex of a given face
inline uint Mesh::getVertexIndexInFace(uint faceIndex, uint i, uint part) const
{
    return (mIndices[part])[faceIndex*3 + i];
}

// Return true if the mesh has normals
inline bool Mesh::hasNormals() const
{
    return mNormals.size() == mVertices.size();
}

// Return true if the mesh has tangents
inline bool Mesh::hasTangents() const
{
    return mTangents.size() == mVertices.size();
}

// Return true if the mesh has vertex colors
inline bool Mesh::hasColors() const
{
    return mColors.size() == mVertices.size();
}

// Return true if the mesh has UV texture coordinates
inline bool Mesh::hasUVTextureCoordinates() const
{
    return mUVs.size() == mVertices.size();
}

// Return true if the mesh has a texture for a given part of the mesh and if it
// also have texture coordinates
inline bool Mesh::hasTextureForPart(uint part) const
{
    return hasUVTextureCoordinates() && mTextures.count(part);
}

// Return true if the mesh has a texture (and texture coordinates) for at least one
// part of the mesh
inline bool Mesh::hasTexture() const
{
    return hasUVTextureCoordinates() && (mTextures.size() > 0);
}

// Return a pointer to the vertices data
inline void* Mesh::getVerticesPointer()
{
    return &(mVertices[0]);
}

// Return a pointer to the normals data
inline void* Mesh::getNormalsPointer()
{
    return &(mNormals[0]);
}

// Return a pointer to the colors data
inline void* Mesh::getColorsPointer()
{
    return &(mColors[0]);
}

// Return a pointer to the tangents data
inline void* Mesh::getTangentsPointer()
{
    return &(mTangents[0]);
}

// Return a pointer to the UV texture coordinates data
inline void* Mesh::getUVTextureCoordinatesPointer()
{
    return &(mUVs[0]);
}

// Return a pointer to the vertex indicies data
inline void* Mesh::getIndicesPointer(uint part)
{
    return &(mIndices[part])[0];
}

// Return a reference to a texture of the mesh
inline Texture2D& Mesh::getTexture(uint part)
{
    return mTextures[part];
}

// Set a texture to a part of the mesh
inline void Mesh::setTexture(Texture2D& texture, uint part)
{
    mTextures[part] = texture;
}

}

#endif
