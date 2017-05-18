/*
 * rpConvexHullShape.cpp
 *
 *  Created on: 22 нояб. 2016 г.
 *      Author: wera
 */

#include "rpConvexHullShape.h"

namespace real_physics
{


rpConvexHullShape::rpConvexHullShape( rpModelConvexHull* initHull ,
		                              scalar margin)
: rpConvexShape( CONVEX_HULL_MESH , margin )
{
	mQuickHull  = &initHull->mQuickHull;
	mConvexHull = &initHull->mConvexHull;

	mNbMaxPeturberationIteration = 10;
    mEpsilonPeturberation = 0.055;
}



rpConvexHullShape::~rpConvexHullShape()
{

//   if(mQuickHull != NULL)
//   {
//	   delete mQuickHull;
//	   mQuickHull=NULL;
//   }

}


Vector3 rpConvexHullShape::getLocalSupportPointWithoutMargin(const Vector3& direction , void** cachedCollisionData) const
{
	uint index = 0;
	scalar max = (mConvexHull->getVertexBuffer()[0].dot(direction));

	for (uint i = 1; i < mConvexHull->getVertexBuffer().size(); i++)
	{
		scalar d = (mConvexHull->getVertexBuffer()[i].dot(direction));
		if (d > max)
		{
			max = d;
			index = i;
		}
	}

	return mConvexHull->getVertexBuffer()[index];
}




bool rpConvexHullShape::testPointInside(const Vector3& localPoint, rpProxyShape* proxyShape) const
{
    return true;//NarrowPhaseGJKAlgorithm.testPointInside(localPoint, proxyShape);
}


bool rpConvexHullShape::raycast(const Ray& ray, RaycastInfo& raycastInfo, rpProxyShape* proxyShape) const
{
    return true;//NarrowPhaseGJKAlgorithm.raycast(ray, raycastInfo, proxyShape);
}


size_t rpConvexHullShape::getSizeInBytes() const
{
	return sizeof(rpConvexHullShape);
}



void rpConvexHullShape::setLocalScaling(const Vector3& scaling)
{
  mScaling = scaling;
}


void rpConvexHullShape::getIntervalLocal(const Vector3& xAxis, scalar& min, scalar& max) const
{

	Vector3 s_p0 = getLocalSupportPointWithoutMargin( xAxis ,  NULL);
	Vector3 s_p1 = getLocalSupportPointWithoutMargin(-xAxis ,  NULL);
	min = s_p1.dot(xAxis);
	max = s_p0.dot(xAxis);
}


void rpConvexHullShape::getLocalBounds(Vector3& min, Vector3& max) const
{
	getIntervalLocal( Vector3::X , min.x , max.x);
	getIntervalLocal( Vector3::Y , min.y , max.y);
	getIntervalLocal( Vector3::Z , min.z , max.z);
}



void rpConvexHullShape::computeLocalInertiaTensor(Matrix3x3& tensor, scalar mass) const
{
	Vector3 min;
	Vector3 max;
	getLocalBounds( min , max );

	Vector3 halfSize;
	halfSize.x = Abs(min.x - max.x) * 0.5;
	halfSize.y = Abs(min.y - max.y) * 0.5;
	halfSize.z = Abs(min.z - max.z) * 0.5;


	scalar  factor = (scalar(1.0) / scalar(3.0)) * mass;
	Vector3 realExtent = halfSize + Vector3(mMargin, mMargin, mMargin);
	scalar  xSquare = realExtent.x * realExtent.x;
	scalar  ySquare = realExtent.y * realExtent.y;
	scalar  zSquare = realExtent.z * realExtent.z;
	tensor.setAllValues(factor * (ySquare + zSquare), 0.0, 0.0,
			            0.0, factor * (xSquare + zSquare), 0.0,
			            0.0, 0.0, factor * (xSquare + ySquare));

}


} /* namespace real_physics */


