#ifndef CAMERA_H
#define CAMERA_H


#include "../Object/Object3D.h"

namespace utility_engine
{

class Camera : public Object3D
{
    public:

        Camera();

        void ProjectionMatrix(float FieldOfView, float aspect, float NearPlane, float FarPlane);
        void LookAt(const Vector3& eye, const Vector3& center, const Vector3& up);


        Matrix4 getProjectionMatrix() const;
        Matrix4 getViewMatrix() const;

protected:

        Matrix4 mProjectionMatrix;
};

}

#endif // CAMERA_H
