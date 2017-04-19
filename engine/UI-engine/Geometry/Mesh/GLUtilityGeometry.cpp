#include "GLUtilityGeometry.h"


using namespace utility_engine;


GLUtilityGeometry::GLUtilityGeometry(const Vector3 *verticess, unsigned int sizeVertex  ,
                                     const Vector2 *coordTex , unsigned int sizeTexCoors,
                                     const Vector3 *normals  , unsigned int sizeNormals ,
                                     const uint    *indicess , unsigned int sizeIndexes):
   indexBuf(QOpenGLBuffer::IndexBuffer)
{
    // initializeOpenGLFunctions();

    // Generate 2 VBOs
    arrayBufTexUV.create();
    arrayBufVertex.create();
    arrayBufNormal.create();
    indexBuf.create();


    // Initializes geometry and transfers it to VBOs
    initGeometry( verticess , sizeVertex   ,
                  coordTex  , sizeTexCoors ,
                  normals   , sizeNormals  ,
                  indicess  , sizeIndexes );
}

GLUtilityGeometry::~GLUtilityGeometry()
{
    arrayBufTexUV.destroy();
    arrayBufVertex.destroy();
    arrayBufNormal.destroy();
    indexBuf.destroy();
}

void GLUtilityGeometry::drawGeometry(QOpenGLShaderProgram *program)
{

    arrayBufVertex.bind();
    // Tell OpenGL programmable pipeline how to locate vertex position data
    int vertexLocation = program->attributeLocation("a_vertex");
    program->enableAttributeArray(vertexLocation);
    program->setAttributeBuffer(vertexLocation, GL_FLOAT, 0 , 3 , sizeof(Vector3));


    arrayBufTexUV.bind();
    // Tell OpenGL programmable pipeline how to locate vertex texture coordinate data
    int texcoordLocation = program->attributeLocation("a_texcoord");
    program->enableAttributeArray(texcoordLocation);
    program->setAttributeBuffer(texcoordLocation, GL_FLOAT,  0 , 2, sizeof(Vector2));


    arrayBufNormal.bind();
    // Tell OpenGL programmable normal coordinate data
    int normalLocation = program->attributeLocation("a_normal");
    program->enableAttributeArray(normalLocation);
    program->setAttributeBuffer(normalLocation, GL_FLOAT, 0 , 3, sizeof(Vector3));


    indexBuf.bind();
    // Draw cube geometry using indices from VBO 1
    glDrawElements(GL_TRIANGLES, mSizeVertex , GL_UNSIGNED_INT , 0);

}

void GLUtilityGeometry::initGeometry(const Vector3 *vertices, unsigned int sizeVertex  ,
                                     const Vector2 *coordTex, unsigned int sizeTexCoors,
                                     const Vector3 *normals , unsigned int sizeNormals ,
                                     const uint    *indicess, unsigned int sizeIndexes)
{

      arrayBufVertex.bind();
      arrayBufVertex.allocate(vertices , sizeVertex * sizeof(Vector3));


      arrayBufTexUV.bind();
      arrayBufTexUV.allocate(coordTex , sizeTexCoors * sizeof(Vector2));

      arrayBufNormal.bind();
      arrayBufNormal.allocate( normals , sizeNormals * sizeof(Vector3));

      indexBuf.bind();
      indexBuf.allocate(indicess, mSizeVertex = sizeIndexes * sizeof(uint));

}
