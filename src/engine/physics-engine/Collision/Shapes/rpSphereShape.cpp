/*
 * rpSphereShape.cpp
 *
 *  Created on: 18 нояб. 2016 г.
 *      Author: wera
 */



// Libraries
#include "../rpProxyShape.h"
#include <cassert>
#include "rpSphereShape.h"

namespace real_physics
{



bool RaySphereIntersect(const Vector3 &p, const Vector3 &d, scalar r, scalar  &t0, scalar &t1)
{


	Vector3 m_center(0,0,0);
	Vector3 origin = p;
	Vector3 direction = (d - p);
	scalar m_radiusSquared = r*r;


	    //Squared distance between ray origin and sphere center
	    scalar squaredDist = (origin-m_center).dot(origin-m_center);

	    //If the distance is less than the squared radius of the sphere...
	    if(squaredDist <= m_radiusSquared)
	    {
	        //Point is in sphere, consider as no intersection existing
	        //std::cout << "Point inside sphere..." << std::endl;
	        return false;
	    }


	    //Calculating the coefficients of the quadratic equation
	    scalar a = direction.dot(direction); // a = d*d
	    scalar b = 2.0f* (direction.dot(origin-m_center)); // b = 2d(o-C)
	    scalar c = (origin-m_center).dot(origin-m_center) - m_radiusSquared; // c = (o-C)^2-R^2

	    //Calculate discriminant
	    scalar disc = (b*b)-(4.0f*a*c);

	    if(disc < 0) //If discriminant is negative no intersection happens
	    {
	        //std::cout << "No intersection with sphere..." << std::endl;
	        return false;
	    }
	    else //If discriminant is positive one or two intersections (two solutions) exists
	    {
	        scalar sqrt_disc =  SquareRoot(disc);
	        t0 = (-b - sqrt_disc) / (2.0f * a);
	        t1 = (-b + sqrt_disc) / (2.0f * a);
	    }

	    //If the second intersection has a negative value then the intersections
	    //happen behind the ray origin which is not considered. Otherwise t0 is
	    //the intersection to be considered
	    if(t1<0)
	    {
	        //std::cout << "No intersection with sphere..." << std::endl;
	        return false;
	    }
	    else
	    {
	        //std::cout << "Intersection with sphere..." << std::endl;
	        return true;
	    }
};




// Constructor
/**
 * @param radius Radius of the sphere (in meters)
 */
rpSphereShape::rpSphereShape( scalar radius )
: rpConvexShape(SPHERE, radius)
{
    assert(radius > scalar(0.0));
    mNbMaxPeturberationIteration = (1); // maximum iteration  for Axis Peturberation
    mEpsilonPeturberation = (0.000001f);// epsilon for Peturberation
}

// Destructor
rpSphereShape::~rpSphereShape()
{

}

// Raycast method with feedback information
bool rpSphereShape::raycast(const Ray& ray, RaycastInfo& raycastInfo, rpProxyShape* proxyShape) const
{

	float radius = mMargin;

//	const Vector3 m = ray.point2;
//	scalar c = m.dot(m) - radius * radius;
//
//	// If the origin of the ray is inside the sphere, we return no intersection
//	if (c < scalar(0.0)) return false;
//
//	const Vector3 rayDirection = ray.point1 - ray.point2;
//	scalar b = m.dot(rayDirection);
//
//	// If the origin of the ray is outside the sphere and the ray
//	// is pointing away from the sphere, there is no intersection
//	if (b > scalar(0.0)) return false;
//
//	scalar raySquareLength = rayDirection.lengthSquare();
//
//	// Compute the discriminant of the quadratic equation
//	scalar discriminant = b * b - raySquareLength * c;
//
//	// If the discriminant is negative or the ray length is very small, there is no intersection
//	if (discriminant < scalar(0.0) || raySquareLength < MACHINE_EPSILON) return false;
//
//	// Compute the solution "t" closest to the origin
//	scalar t = -b - Sqrt(discriminant);
//
//	assert(t >= scalar(0.0));
//
//	// If the hit point is withing the segment ray fraction
//	if (t < ray.maxFraction * raySquareLength)
//	{
//
//		// Compute the intersection information
//		t /= raySquareLength;
//		// raycastInfo.body = proxyShape->getBody();
//		raycastInfo.proxyShape = proxyShape;
//		raycastInfo.hitFraction = t;
//		raycastInfo.worldPoint = ray.point1 + t * rayDirection;
//		raycastInfo.worldNormal = raycastInfo.worldPoint;
//
//		return true;
//	}



	    const Vector3 m = ray.point1;
	    scalar c = m.dot(m) - radius * radius;

	    // If the origin of the ray is inside the sphere, we return no intersection
	    if (c < scalar(0.0)) return false;

	    const Vector3 rayDirection = ray.point2 - ray.point1;
	    scalar b = m.dot(rayDirection);

	    // If the origin of the ray is outside the sphere and the ray
	    // is pointing away from the sphere, there is no intersection
	    if (b > scalar(0.0)) return false;

	    scalar raySquareLength = rayDirection.lengthSquare();

	    // Compute the discriminant of the quadratic equation
	    scalar discriminant = b * b - raySquareLength * c;

	    // If the discriminant is negative or the ray length is very small, there is no intersection
	    if (discriminant < scalar(0.0) || raySquareLength < MACHINE_EPSILON) return false;

	    // Compute the solution "t" closest to the origin
	    scalar t = -b - SquareRoot(discriminant);

	    assert(t >= scalar(0.0));

	    // If the hit point is withing the segment ray fraction
	    if (t < ray.maxFraction * raySquareLength) {

	        // Compute the intersection information
	        t /= raySquareLength;

	        raycastInfo.body = proxyShape->getBody();
	        raycastInfo.proxyShape = proxyShape;
	        raycastInfo.hitFraction = t;
	        raycastInfo.worldPoint = ray.point1 + t * rayDirection;
	        raycastInfo.worldNormal = raycastInfo.worldPoint;

	        return true;
	    }

	    return false;


//	scalar tMin;
//	scalar tMax;
//
//    return RaySphereIntersect(ray.point2 , ray.point1 , mRadius * 0.5 , tMin , tMax);


}



} /* namespace real_physics */
