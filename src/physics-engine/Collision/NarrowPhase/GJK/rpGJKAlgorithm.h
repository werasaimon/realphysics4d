#ifndef GJKALGORITHM_H
#define GJKALGORITHM_H

// Libraries

#include "../../../Geometry/QuickHull/Structs/Ray.hpp"
#include "../../../Memory/memory.h"
#include "../../Shapes/rpConvexShape.h"
#include "../rpNarrowPhaseCollisionAlgorithm.h"


namespace real_physics
{

// Constants
const scalar REL_ERROR = scalar(1.0e-4);
const scalar REL_ERROR_SQUARE = REL_ERROR * REL_ERROR;
const int MAX_ITERATIONS_GJK_RAYCAST = 32;




class rpGJKAlgorithm
{

    private :

        //-------------------- Attributes --------------------//
        Vector3 mCachedSeparatingAxis;



        //-------------------- Methods --------------------//
        /// Private copy-constructor
        rpGJKAlgorithm(const rpGJKAlgorithm& algorithm);

        /// Private assignment operator
        rpGJKAlgorithm& operator=(const rpGJKAlgorithm& algorithm);

        /// Compute the penetration depth for enlarged objects.
        bool computePenetrationDepthForEnlargedObjects(const rpCollisionShapeInfo& shape1Info, const Transform& transform1,
                                                       const rpCollisionShapeInfo& shape2Info, const Transform& transform2,
                                                       Vector3& v ,
                                                       OutContactInfo& _outInfo);

    public :


        /// Constructor
        rpGJKAlgorithm();

        /// Destructor
        ~rpGJKAlgorithm();

        /// Compute a contact info if the two bounding volumes collide.
        bool computeGJK(const rpCollisionShapeInfo& shape1Info,
                        const rpCollisionShapeInfo& shape2Info,
                        OutContactInfo& _outInfo);




        /// Use the GJK Algorithm to find if a point is inside a convex collision shape
        bool testPointInside(const Vector3& localPoint, rpProxyShape* proxyShape);

        /// Ray casting algorithm agains a convex collision shape using the GJK Algorithm
        bool raycast(const Ray& ray, RaycastInfo& raycastInfo , rpProxyShape* proxyShape );



    //-----------------------------------------------//
    const Vector3& getCachedSeparatingAxis() const
    {
        return mCachedSeparatingAxis;
    }

    void setCachedSeparatingAxis(const Vector3& cachedSeparatingAxis)
    {
        mCachedSeparatingAxis = cachedSeparatingAxis;
    }

};



} /* namespace real_physics */




#endif // GJKALGORITHM_H
