#ifndef GLUTILITYGEOMETRY_H
#define GLUTILITYGEOMETRY_H


#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>

#include <QOpenGLTexture>

#include "../Object/Object3D.h"
#include "../maths/Vector2.h"
#include "../maths/Vector3.h"


namespace utility_engine
{

    class Mesh;

    class GLUtilityGeometry
    {
      public:

         GLUtilityGeometry( const Vector3  *verticess , unsigned int sizeVertex,
                            const Vector2  *coordTex  , unsigned int sizeTexCoors,
                            const Vector3  *normals   , unsigned int sizeNormals,
                            const uint     *indicess  , unsigned int sizeIndexes );

         GLUtilityGeometry(const Mesh *mesh );

        ~GLUtilityGeometry();

         void drawGeometry(QOpenGLShaderProgram *program);

      private:

         void initGeometry( const Vector3  *vertices  , unsigned int sizeVertex,
                            const Vector2  *coordTex  , unsigned int sizeTexCoors,
                            const Vector3  *normals   , unsigned int sizeNormals,
                            const uint     *indicess  , unsigned int sizeIndexes );

            QOpenGLBuffer arrayBufTexUV;
            QOpenGLBuffer arrayBufVertex;
            QOpenGLBuffer arrayBufNormal;
            QOpenGLBuffer indexBuf;

            unsigned int mSizeVertex;


            // Textures of the mesh (one for each part of the mesh)
            //std::map<uint, QOpenGLTexture> mTextures;


    };

}

#endif // GLUTILITYGEOMETRY_H
