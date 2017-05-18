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

    mNbMaxPeturberationIteration = (4); // maximum iteration  for Peturberation
    mEpsilonPeturberation = (0.088f);// epsilon for Peturberation
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
