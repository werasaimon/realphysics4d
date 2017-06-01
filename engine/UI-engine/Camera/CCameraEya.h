/*
 * CCameraEya.h
 *
 *  Created on: 7 нояб. 2016 г.
 *      Author: wera
 */

#ifndef OPENGL_FRAMEWORK_CAMERA_CCAMERAEYA_H_
#define OPENGL_FRAMEWORK_CAMERA_CCAMERAEYA_H_

#include "../Object/Object3D.h"

namespace utility_engine
{

    typedef unsigned int uint;

    class CCameraEya : public Object3D
    {
       public:
          CCameraEya();
         ~CCameraEya();


          void initProjectionMatrix(float mFieldOfView, float aspect, float mNearPlane, float mFarPlane);


          void updateProjectionMatrix(float aspect);
          void setDimensions(float width, float height);
          void update();


          void    OnMouseMove(float dx, float dy);
          void    OnMouseWheel(float zDelta);




           /****************** set method **********/
          void setPosition(const Vector3 &position);


          /******************* get method **********/
          Vector3 getPosition() const;
          Matrix4 getProjectionMatrix() const;
          Matrix4 getViewMatrix() const;


          const Matrix4& getRotatetMatrix() const
          {
		    return mRotatetMatrix;
	      }

    private:


         /*************** atribute ***********************************/
          Vector3 mPosition;
          Matrix4 mRotatetMatrix;



          float   mMouseX;
          float   mMouseY;


          float   mCentricRadius;

          float   mFieldOfView;
          float   mNearPlane;
          float   mFarPlane;

          uint    mWidth;
          uint    mHeight;

          Matrix4 mProjectionMatrix;

    };

} /* namespace utility_engine */

#endif /* OPENGL_FRAMEWORK_CAMERA_CCAMERAEYA_H_ */
