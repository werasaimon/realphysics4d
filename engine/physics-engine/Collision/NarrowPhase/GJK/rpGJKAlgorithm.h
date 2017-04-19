/*
 * rpGJKAlgorithm.h
 *
 *  Created on: 19 нояб. 2016 г.
 *      Author: wera
 */

#ifndef SOURCE_ENGIE_COLLISION_NARROWPHASE_GJK_RPGJKALGORITHM_H_
#define SOURCE_ENGIE_COLLISION_NARROWPHASE_GJK_RPGJKALGORITHM_H_


// Libraries

#include "../../../Geometry/QuickHull/Structs/Ray.hpp"
#include "../../../Memory/memory.h"
#include "../../Shapes/rpConvexShape.h"
#include "../EPA/rpEPAAlgorithm.h"
#include "../rpNarrowPhaseCollisionAlgorithm.h"




namespace real_physics
{

// Constants
const scalar REL_ERROR = scalar(1.0e-7);
const scalar REL_ERROR_SQUARE = REL_ERROR * REL_ERROR;
const int MAX_ITERATIONS_GJK_RAYCAST = 32;




class GJKAlgorithm: public NarrowPhaseCollisionAlgorithm
{

    private :

        // -------------------- Attributes -------------------- //

	    Vector3 mCachedSeparatingAxis;

        /// EPA Algorithm
        EPAAlgorithm mAlgoEPA;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        GJKAlgorithm(const GJKAlgorithm& algorithm);

        /// Private assignment operator
        GJKAlgorithm& operator=(const GJKAlgorithm& algorithm);

        /// Compute the penetration depth for enlarged objects.
        bool computePenetrationDepthForEnlargedObjects(const CollisionShapeInfo& shape1Info, const Transform& transform1,
                                                       const CollisionShapeInfo& shape2Info, const Transform& transform2,
													   const MinkowskiDifferens<const rpConvexShape> Minkowski ,
                                                       Vector3& v ,
													   OutContactInfo& _outInfo);

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        GJKAlgorithm();

        /// Destructor
        ~GJKAlgorithm();

//        /// Initalize the algorithm
//        virtual void init(CollisionDetection* collisionDetection,
//                          MemoryAllocator* memoryAllocator);

        /// Compute a contact info if the two bounding volumes collide.
        virtual bool testCollision(const CollisionShapeInfo& shape1Info,
                                   const CollisionShapeInfo& shape2Info,
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


static GJKAlgorithm NarrowPhaseGJKAlgorithm;

//// Initalize the algorithm
//inline void GJKAlgorithm::init(CollisionDetection* collisionDetection,
//                               MemoryAllocator* memoryAllocator)
//{
//    /*NarrowPhaseAlgorithm::init(collisionDetection, memoryAllocator);*/
//    mAlgoEPA.init(memoryAllocator);
//}

} /* namespace real_physics */




#endif /* SOURCE_ENGIE_COLLISION_NARROWPHASE_GJK_RPGJKALGORITHM_H_ */
