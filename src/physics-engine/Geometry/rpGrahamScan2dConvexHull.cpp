/*
 * rpGrahamScan2dConvexHull.cpp
 *
 *  Created on: 21 нояб. 2016 г.
 *      Author: wera
 */

#include "../Geometry/rpGrahamScan2dConvexHull.h"

namespace real_physics
{

SIMD_INLINE  void GrahamScanConvexHull2D( b3AlignedObjectArray<GrahamVector3>& originalPoints,
		                                  b3AlignedObjectArray<GrahamVector3>& hull,
		                                  const Vector3& normalAxis)
{
	Vector3 axis0,axis1;
	Vector3::btPlaneSpace1(normalAxis,axis0,axis1);

	if (originalPoints.size()<=1)
	{
		for (int i=0;i<originalPoints.size();i++)
			hull.push_back(originalPoints[0]);

		return;
	}
	//step1 : find anchor point with smallest projection on axis0 and move it to first location
	for (int i=0;i<originalPoints.size();i++)
	{
		//		const wsVector3& left = originalPoints[i];
		//		const wsVector3& right = originalPoints[0];
		scalar projL = originalPoints[i].dot(axis0);
		scalar projR = originalPoints[0].dot(axis0);

		if (projL < projR)
		{
			originalPoints.swap(0,i);
		}
	}

	//also precompute angles
	originalPoints[0].m_angle = -1e30f;
	for (int i=1;i<originalPoints.size();i++)
	{
		Vector3 ar = originalPoints[i]-originalPoints[0];
		scalar ar1 = axis1.dot(ar);
		scalar ar0 = axis0.dot(ar);

		if( ar1*ar1+ar0*ar0 < FLT_EPSILON )
		{
			originalPoints[i].m_angle = 0.0f;
		}
		else
		{
			originalPoints[i].m_angle = btAtan2Fast(ar1, ar0);
		}
	}

	//step 2: sort all points, based on 'angle' with this anchor
	btAngleCompareFunc comp(originalPoints[0]);
	originalPoints.quickSortInternal(comp,1,originalPoints.size()-1);


	int i;
	for (i = 0; i<2; i++)
		hull.push_back(originalPoints[i]);

	//step 3: keep all 'convex' points and discard concave points (using back tracking)
	for (; i != originalPoints.size(); i++)
	{
		bool isConvex = false;
		while (!isConvex&& hull.size()>1)
		{
			Vector3& a = hull[hull.size()-2];
			Vector3& b = hull[hull.size()-1];
			isConvex = (a-b).cross(a-originalPoints[i]).dot(normalAxis)> 0;

			if (!isConvex)
				hull.pop_back();
			else
				hull.push_back(originalPoints[i]);
		}

		if( hull.size() == 1 )
		{
			hull.push_back( originalPoints[i] );
		}
	}
}

SIMD_INLINE void GrahamScanConvexHull2D( b3AlignedObjectArray<Vector3> inputVertices,
		                                 b3AlignedObjectArray<Vector3>& outputVertices,
		                                 const Vector3& normalAxis)
{
	b3AlignedObjectArray<GrahamVector3> InputOriginalPoints;
	b3AlignedObjectArray<GrahamVector3> OutputOriginalPoints;
	for (int i = 0; i < inputVertices.size(); ++i)
	{
		InputOriginalPoints.push_back(inputVertices[i]);
	}

	GrahamScanConvexHull2D( InputOriginalPoints , OutputOriginalPoints , normalAxis );

	for (int i = 0; i < OutputOriginalPoints.size(); ++i)
	{
		scalar x = OutputOriginalPoints[i].getX();
		scalar y = OutputOriginalPoints[i].getY();
		scalar z = OutputOriginalPoints[i].getZ();

		outputVertices.push_back(Vector3(x,y,z));
	}
}


} /* namespace real_physics */


