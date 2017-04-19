/*
 * rpBoxShape.cpp
 *
 *  Created on: 17 нояб. 2016 г.
 *      Author: wera
 */


// Libraries
#include "rpBoxShape.h"

#include <algorithm>
#include <cassert>
#include <cmath>

#include "../../LinearMaths/mathematics.h"
#include "../../LinearMaths/rpVector3D.h"
#include "../../Memory/memory.h"
#include "../rpProxyShape.h"
#include "../rpRaycastInfo.h"
#include "rpCollisionShape.h"



namespace real_physics
{




	//------------------------------------------------------------------------------------------------//

	 bool ClipSegment(scalar min, scalar max, scalar a, scalar b, scalar d, scalar& t0, scalar& t1)
	 {
	 	const scalar threshold = 1.0e-6f;

	 	if (Abs(d) < threshold)
	 	{
	 		if (d > 0.0f)
	 		{
	 			return !(b < min || a > max);
	 		}
	 		else
	 		{
	 			return !(a < min || b > max);
	 		}
	 	}

	 	scalar u0, u1;

	 	u0 = (min - a) / (d);
	 	u1 = (max - a) / (d);

	 	if (u0 > u1)
	 	{
	 		Swap(u0, u1);
	 	}

	 	if (u1 < t0 || u0 > t1)
	 	{
	 		return false;
	 	}

	 	t0 = Max(u0, t0);
	 	t1 = Min(u1, t1);

	 	if (t1 < t0)
	 	{
	 		return false;
	 	}

	 	return true;
	 }




	 bool ClipSegment(const Vector3& A,   const Vector3& B,
			          const Vector3& Min, const Vector3& Max ,
					  scalar &min , scalar &max)
	 {
	 	Vector3 S = A;
	 	Vector3 D = (B - A);

	 	scalar t0 = 0.0f, t1 = 1.0f;

	 	if (!ClipSegment(Min.x, Max.x, A.x, B.x, D.x, t0, t1))
	 	{
	 		return false;
	 	}

	 	if (!ClipSegment(Min.y, Max.y, A.y, B.y, D.y, t0, t1))
	 	{
	 		return false;
	 	}

	 	if (!ClipSegment(Min.z, Max.z, A.z, B.z, D.z, t0, t1))
	 	{
	 		return false;
	 	}

	 	min = t0;
	 	max = t1;

	 	//A = S + D * t0;
	 	//B = S + D * t1;

	 	return true;
	 }

	 //-----------------------------------------------------------------------------------------------//


 // Constructor
 /**
 * @param extent The vector with the three extents of the box (in meters)
 * @param margin The collision margin (in meters) around the collision shape
 */

     rpBoxShape::rpBoxShape(const Vector3 &extent)
         : rpConvexShape(BOX, OBJECT_MARGIN) ,
           mExtent(extent - Vector3(OBJECT_MARGIN,OBJECT_MARGIN,OBJECT_MARGIN))
     {

         assert(extent.x > scalar(0.0) && extent.x > OBJECT_MARGIN);
         assert(extent.y > scalar(0.0) && extent.y > OBJECT_MARGIN);
         assert(extent.z > scalar(0.0) && extent.z > OBJECT_MARGIN);

         mNbMaxPeturberationIteration = (4); // maximum iteration  for Axis Peturberation
         mEpsilonPeturberation = (0.08f);// epsilon for Peturberation
     }


// Constructor
/**
 * @param extent The vector with the three extents of the box (in meters)
 * @param margin The collision margin (in meters) around the collision shape
 */
rpBoxShape::rpBoxShape(const Vector3& extent, scalar margin)
: rpConvexShape(BOX, margin) ,
  mExtent(extent - Vector3(margin, margin, margin))
{

    assert(extent.x > scalar(0.0) && extent.x > margin);
    assert(extent.y > scalar(0.0) && extent.y > margin);
    assert(extent.z > scalar(0.0) && extent.z > margin);

    mNbMaxPeturberationIteration = (4); // maximum iteration  for Axis Peturberation
    mEpsilonPeturberation = (0.08f);// epsilon for Peturberation
}

// Destructor
rpBoxShape::~rpBoxShape()
{

}

// Return the local inertia tensor of the collision shape
/**
 * @param[out] tensor The 3x3 inertia tensor matrix of the shape in local-space
 *                    coordinates
 * @param mass Mass to use to compute the inertia tensor of the collision shape
 */
void rpBoxShape::computeLocalInertiaTensor(Matrix3x3& tensor, scalar mass) const
{
    scalar factor = (scalar(1.0) / scalar(3.0)) * mass;
    Vector3 realExtent = mExtent + Vector3(mMargin, mMargin, mMargin);
    scalar xSquare = realExtent.x * realExtent.x;
    scalar ySquare = realExtent.y * realExtent.y;
    scalar zSquare = realExtent.z * realExtent.z;
    tensor.setAllValues(factor * (ySquare + zSquare), 0.0, 0.0,
                        0.0, factor * (xSquare + zSquare), 0.0,
                        0.0, 0.0, factor * (xSquare + ySquare));
}

// Raycast method with feedback information
bool rpBoxShape::raycast(const Ray& ray, RaycastInfo& raycastInfo, rpProxyShape* proxyShape) const
{

    Vector3 rayDirection = ray.point2 - ray.point1;
    scalar tMin = DECIMAL_SMALLEST;
    scalar tMax = DECIMAL_LARGEST;
    Vector3 normalDirection(scalar(0), scalar(0), scalar(0));
    Vector3 currentNormal;




//    if(!ClipSegment(ray.point1 , ray.point2 , -mExtent * 0.5 , mExtent * 0.5 , tMin , tMax))
//    {
//         return false;
//    }

//		    scalar enter = 0.0f;
//		    scalar exit = 1.0f;
//		    // For each of the three slabs
//		    for (int i=0; i<3; i++)
//		    {
//		    	// If ray is parallel to the slab
//		    	if (std::abs(rayDirection[i]) < MACHINE_EPSILON)
//		    	{
//		    		// If the ray's origin is not inside the slab, there is no hit
//		    		if (ray.point1[i] > mExtent[i] || ray.point1[i] < -mExtent[i]) return false;
//		    	}
//
//		    	if (!line_vs_aabb_1d( ray.point1[i] , -rayDirection[i] , -mExtent[i] * 0.5 , mExtent[i] * 0.5 , enter , exit))
//		    	{
//		    		return false;
//		    	}
//		    }
//		    tMin = exit;
//		    tMax = enter;


		    // For each of the three slabs
		    for (int i=0; i<3; i++)
		    {


		        // If ray is parallel to the slab
		        if (std::abs(rayDirection[i]) < MACHINE_EPSILON)
		        {

		            // If the ray's origin is not inside the slab, there is no hit
		            if (ray.point1[i] > mExtent[i] || ray.point1[i] < -mExtent[i]) return false;
		        }
		        else
		        {

		            // Compute the intersection of the ray with the near and far plane of the slab
		            scalar oneOverD = scalar(1.0) / rayDirection[i];
		            scalar t1 = (-mExtent[i] - ray.point1[i]) * oneOverD;
		            scalar t2 = (mExtent[i] - ray.point1[i]) * oneOverD;
		            currentNormal[0] = (i == 0) ? -mExtent[i] : scalar(0.0);
		            currentNormal[1] = (i == 1) ? -mExtent[i] : scalar(0.0);
		            currentNormal[2] = (i == 2) ? -mExtent[i] : scalar(0.0);

		            // Swap t1 and t2 if need so that t1 is intersection with near plane and
		            // t2 with far plane
		            if (t1 > t2)
		            {
		                std::swap(t1, t2);
		                currentNormal = -currentNormal;
		            }

		            // Compute the intersection of the of slab intersection interval with previous slabs
		            if (t1 > tMin)
		            {
		                tMin = t1;
		                normalDirection = currentNormal;
		            }
		            tMax = std::min(tMax, t2);

		            // If tMin is larger than the maximum raycasting fraction, we return no hit
		            if (tMin > ray.maxFraction) return false;

		            // If the slabs intersection is empty, there is no hit
		            if (tMin > tMax) return false;
		        }
		    }

    // If tMin is negative, we return no hit
    if (tMin < scalar(0.0) || tMin > ray.maxFraction) return false;

    // The ray intersects the three slabs, we compute the hit point
    Vector3 localHitPoint = ray.point1 + tMin * rayDirection;


    raycastInfo.body = proxyShape->getBody();
    raycastInfo.proxyShape = proxyShape;
    raycastInfo.hitFraction = tMin;
    raycastInfo.worldPoint = localHitPoint;
    raycastInfo.worldNormal = normalDirection;

    return true;
}




} /* namespace real_physics */
