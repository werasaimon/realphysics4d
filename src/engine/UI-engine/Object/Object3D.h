#ifndef MESH_OBJECT3D_H_
#define MESH_OBJECT3D_H_


// Libraries
#include "../maths/Vector3.h"
#include "../maths/Matrix4.h"


namespace utility_engine
{


// Class Object3D
// This class represent a generic 3D object on the scene.
class Object3D
{

    protected:


	// -------------------- Attributes -------------------- //

        // Transformation matrix that convert local-space
        // coordinates to world-space coordinates
        Matrix4 mTransformMatrix;

    public:

        // -------------------- Methods -------------------- //

        // Constructor
        Object3D();

        // Destructor
        virtual ~Object3D();

        // Return the transform matrix
        const Matrix4& getTransformMatrix() const;


        void  setTransformMatrix( const Matrix4& matrix );



        // Set to the identity transform
        void setToIdentity();

        // Return the origin of object in world-space
        Vector3 getOrigin() const;

        // Translate the object in world-space
        void translateWorld(const Vector3& v);

        // Translate the object in local-space
        void translateLocal(const Vector3& v);

        // Rotate the object in world-space
        void rotateWorld(const Vector3& axis, float angle);

        // Rotate the object in local-space
        void rotateLocal(const Vector3& axis, float angle);

        // Rotate around a world-space point
        void rotateAroundWorldPoint(const Vector3& axis, float angle, const Vector3& point);

        // Rotate around a local-space point
        void rotateAroundLocalPoint(const Vector3& axis, float angle, const Vector3& worldPoint);
};

// Return the transform matrix
inline const Matrix4& Object3D::getTransformMatrix() const {
    return mTransformMatrix;
}


inline void  Object3D::setTransformMatrix( const Matrix4& matrix )
{
   	mTransformMatrix = matrix;
}


// Set to the identity transform
inline void Object3D::setToIdentity()
{
    mTransformMatrix.setToIdentity();
}

 // Return the origin of object in world-space
inline Vector3 Object3D::getOrigin() const
{
    return mTransformMatrix * Vector3(0.0, 0.0, 0.0);
}


// Translate the object in world-space
inline void Object3D::translateWorld(const Vector3& v) {
    mTransformMatrix = Matrix4::translationMatrix(v) * mTransformMatrix;
}

// Translate the object in local-space
inline void Object3D::translateLocal(const Vector3& v)
{
    mTransformMatrix = mTransformMatrix * Matrix4::translationMatrix2(v);
   // mTransformMatrix =  Matrix4::translationMatrix2(v) * mTransformMatrix;
}

// Rotate the object in world-space
inline void Object3D::rotateWorld(const Vector3& axis, float angle)
{
    mTransformMatrix = Matrix4::rotationMatrix2(axis, angle) * mTransformMatrix;
}

// Rotate the object in local-space
inline void Object3D::rotateLocal(const Vector3& axis, float angle)
{
    mTransformMatrix = mTransformMatrix * Matrix4::rotationMatrix2(axis, angle);
}


//// Translate the object in world-space
//inline void Object3D::translateWorld(const Vector3& v)
//{
//    mTransformMatrix = mTransformMatrix * Matrix4::translationMatrix(v);
//}
//
//// Translate the object in local-space
//inline void Object3D::translateLocal(const Vector3& v)
//{
//    mTransformMatrix = Matrix4::translationMatrix2(v) * mTransformMatrix;
//}
//
//// Rotate the object in world-space
//inline void Object3D::rotateWorld(const Vector3& axis, float angle)
//{
//	mTransformMatrix = mTransformMatrix * Matrix4::rotationMatrix2(axis, angle);
//}
//
//// Rotate the object in local-space
//inline void Object3D::rotateLocal(const Vector3& axis, float angle)
//{
//	mTransformMatrix = Matrix4::rotationMatrix2(axis, angle) * mTransformMatrix;
//}

// Rotate the object around a world-space point
inline void Object3D::rotateAroundWorldPoint(const Vector3& axis, float angle,
                                             const Vector3& worldPoint) {
    mTransformMatrix =   Matrix4::translationMatrix(worldPoint) * Matrix4::rotationMatrix(axis, angle)
                       * Matrix4::translationMatrix(-worldPoint) * mTransformMatrix;
}

// Rotate the object around a local-space point
inline void Object3D::rotateAroundLocalPoint(const Vector3& axis, float angle,
                                             const Vector3& worldPoint) {

    // Convert the world point into the local coordinate system
    Vector3 localPoint = mTransformMatrix.getInverse() * worldPoint;

    mTransformMatrix = mTransformMatrix * Matrix4::translationMatrix(localPoint)
                       * Matrix4::rotationMatrix(axis, angle)
                       * Matrix4::translationMatrix(-localPoint);
}


}


#endif /* MESH_OBJECT3D_H_ */
