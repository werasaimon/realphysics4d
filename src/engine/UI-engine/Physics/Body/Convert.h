#ifndef CONVERT_H
#define CONVERT_H

#include "../../Mesh/Mesh.h"
#include "../../../physics-engine/physics.h"

namespace utility_engine
{



    static real_physics::Transform Matrix4ConvertToTransform( Matrix4 matrix4 )
    {
        real_physics::Transform transform;
        float m[16];
        matrix4.getTranspose().getDataValue(m);
        transform.setFromOpenGL(m);
        return transform;
    }




    static Matrix4 TransformConvertToMatrix4( real_physics::Transform transform )
    {
        Matrix4 matrix;
        float m[16];
        transform.getOpenGLMatrix(m);
        matrix.setDataValue(m);
        return matrix.getTranspose();
    }



    static std::vector<real_physics::Vector3> MeshConvertToVertexes( const Mesh *mesh )
    {
          std::vector<real_physics::Vector3> vecrtices;
          for (unsigned int i = 0; i < mesh->getNbVertices(); ++i)
          {
              vecrtices.push_back(real_physics::Vector3( mesh->getVertices()[i].x ,
                                                         mesh->getVertices()[i].y ,
                                                         mesh->getVertices()[i].z ));
          }
          return vecrtices;
    }


}

#endif // CONVERT_H
