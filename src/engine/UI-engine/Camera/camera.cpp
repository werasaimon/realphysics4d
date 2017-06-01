#include "camera.h"



namespace utility_engine
{
    Camera::Camera()
    :Object3D()
    {
         mTransformMatrix.setToIdentity();
         mProjectionMatrix.setToIdentity();
    }

    void Camera::ProjectionMatrix(float FieldOfView, float aspect, float NearPlane, float FarPlane)
    {
        mProjectionMatrix.setToIdentity();
        mProjectionMatrix = Matrix4::Perspective2( FieldOfView , aspect , NearPlane , FarPlane);
    }

    void Camera::LookAt(const Vector3 &eye, const Vector3 &center, const Vector3 &up)
    {
        mTransformMatrix.setToIdentity();
        mTransformMatrix = Matrix4::Look( eye , center , up );
    }

    Matrix4 Camera::getProjectionMatrix() const
    {
        return mProjectionMatrix;
    }

    Matrix4 Camera::getViewMatrix() const
    {
        return mTransformMatrix;
    }
}
