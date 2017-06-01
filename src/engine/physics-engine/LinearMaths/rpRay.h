/*
 * rpRay.h
 *
 *  Created on: 15 нояб. 2016 г.
 *      Author: wera
 */

#ifndef SOURCE_REAL_PHYSICS_LINEARMATHS_RPRAY_H_
#define SOURCE_REAL_PHYSICS_LINEARMATHS_RPRAY_H_


#include "rpVector3D.h"

namespace real_physics
{


template<class T> class rpRay
{
  public:

	 // -------------------- Attributes -------------------- //

	        /// First point of the ray (origin)
	        rpVector3D<T> point1;

	        /// Second point of the ray
	        rpVector3D<T> point2;

	        /// Maximum fraction value
	        T maxFraction;


	        // -------------------- Methods -------------------- //


	        rpRay(){}
	        /// Constructor with arguments
	        rpRay(const rpVector3D<T>& p1, const rpVector3D<T>& p2, T maxFrac = T(1.0))
	        : point1(p1), point2(p2), maxFraction(maxFrac)
	        {
	        }

	        /// Copy-constructor
	        rpRay(const rpRay<T>& ray)
	        : point1(ray.point1), point2(ray.point2), maxFraction(ray.maxFraction)
	        {

	        }

	        /// Destructor
	        ~rpRay()
	        {

	        }

	        /// Overloaded assignment operator
	        rpRay& operator=(const rpRay& ray)
	        {
	        	if (&ray != this)
	        	{
	        		point1 = ray.point1;
	        		point2 = ray.point2;
	        		maxFraction = ray.maxFraction;
	        	}
	        	return *this;
	        }

};

} /* namespace real_physics */

#endif /* SOURCE_REAL_PHYSICS_LINEARMATHS_RPRAY_H_ */
