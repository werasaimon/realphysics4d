/*
 * algorithms3D.h
 *
 *  Created on: 22 нояб. 2016 г.
 *      Author: wera
 */

#ifndef SOURCE_ENGIE_GEOMETRY_GEOMETRY_H_
#define SOURCE_ENGIE_GEOMETRY_GEOMETRY_H_



#include "../Geometry/QuickHull/ConvexHull.hpp"
#include "../Geometry/QuickHull/QuickHull.hpp"
#include "../Geometry/rpGrahamScan2dConvexHull.h"
#include "../Geometry/rpPolygonClipping.h"

namespace  real_physics
{
	//----------------------------------------------------------------------------//
    template<class T> using rpQuickHull  = quickhull::QuickHull<T>;
	template<class T> using rpConvexHull = quickhull::ConvexHull<T>;
	//----------------------------------------------------------------------------//
}

#endif /* SOURCE_ENGIE_GEOMETRY_GEOMETRY_H_ */
