/*
 * CCameraEya.cpp
 *
 *  Created on: 7 нояб. 2016 г.
 *      Author: wera
 */

#include "CCameraEya.h"

using namespace opengl_utility;


CCameraEya::CCameraEya() : Object3D()
{
    // Set default values
    mFieldOfView = 45.0f;
    mNearPlane = 0.1f;
    mFarPlane = 10.0f;
    mWidth = 1;
    mHeight = 1;


    mRotatetMatrix.setToIdentity();
    //    this->mTransformMatrix.setToIdentity();
    //    this->mProjectionMatrix.setToIdentity();

    updateProjectionMatrix(float(mWidth) / float(mHeight));

}


CCameraEya::~CCameraEya()
{

}

void CCameraEya::initProjectionMatrix(float FieldOfView, float aspect, float NearPlane, float FarPlane)
{
    mFieldOfView = FieldOfView;
    mNearPlane   = NearPlane;
    mFarPlane    = FarPlane;

    updateProjectionMatrix(aspect);
}




void CCameraEya::updateProjectionMatrix( float aspect )
{
    mProjectionMatrix.setToIdentity();
    mProjectionMatrix = Matrix4::Perspective2( mFieldOfView , aspect , mNearPlane , mFarPlane);


}


void CCameraEya::setDimensions(float width, float height)
{
    mWidth = width;
    mHeight = height;

    updateProjectionMatrix(float(mWidth) / float(mHeight));
}

void CCameraEya::update()
{

    mTransformMatrix = mRotatetMatrix;
    this->translateWorld( -mPosition );
}


void CCameraEya::OnMouseMove(float dx, float dy)
{
    mMouseX = dx;
    mMouseY = dy;

    mRotatetMatrix.setToIdentity();
    mRotatetMatrix = mRotatetMatrix * Matrix4::rotationMatrix(Vector3::Y, mMouseX);
    mRotatetMatrix = mRotatetMatrix * Matrix4::rotationMatrix(Vector3::X, mMouseY);


}

void CCameraEya::OnMouseWheel(float zDelta)
{
    mCentricRadius = zDelta;
}

void CCameraEya::setPosition(const Vector3 &position)
{
    mPosition = position;
}


Vector3 CCameraEya::getPosition() const
{
    return mPosition;
}

Matrix4 CCameraEya::getProjectionMatrix() const
{
    return mProjectionMatrix;
}

Matrix4 CCameraEya::getViewMatrix() const
{
    return this->mTransformMatrix;
}


