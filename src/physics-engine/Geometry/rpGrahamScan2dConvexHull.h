/*
 * rpGrahamScan2dConvexHull.h
 *
 *  Created on: 21 нояб. 2016 г.
 *      Author: wera
 */

#ifndef SOURCE_ENGIE_GEOMETRY_RPGRAHAMSCAN2DCONVEXHULL_H_
#define SOURCE_ENGIE_GEOMETRY_RPGRAHAMSCAN2DCONVEXHULL_H_


#include "../LinearMaths/mathematics.h"
#include "../Memory/memory.h"

namespace real_physics
{

class rpGrahamScan2dConvexHull;


SIMD_INLINE scalar btAtan2Fast(scalar y, scalar x)
{
	scalar coeff_1 = Pi() / 4.0f;
	scalar coeff_2 = 3.0f * coeff_1;
	scalar abs_y = Abs(y);
	scalar angle;

	if (x >= 0.0f)
	{
		scalar r = (x - abs_y) / (x + abs_y);
		angle = coeff_1 - coeff_1 * r;
	}
	else
	{
		scalar r = (x + abs_y) / (abs_y - x);
		angle = coeff_2 - coeff_1 * r;
	}

	return (y < 0.0f) ? -angle : angle;
}


struct GrahamVector3 : public Vector3
{

    GrahamVector3(const Vector3& org = Vector3(0,0,0),
                  int orgIndex = -1)
        :Vector3(org),
         m_orgIndex(orgIndex)
    {
    }

    void operator = ( const Vector3 &otherV )
    {
        this->x = otherV.x;
        this->y = otherV.y;
        this->z = otherV.z;
    }


    scalar	  m_angle;
    iint      m_orgIndex;
};





struct btAngleCompareFunc
{
    Vector3 m_anchor;

    btAngleCompareFunc(const Vector3& anchor)
    : m_anchor(anchor)
    {
    }

    bool operator()(const GrahamVector3& a, const GrahamVector3& b) const
    {
        if (a.m_angle != b.m_angle)
        {
            return a.m_angle < b.m_angle;
        }
        else
        {
            scalar al = (a-m_anchor).lengthSquare();
            scalar bl = (b-m_anchor).lengthSquare();
            if (al != bl)
            {
                return  al < bl;
            }
            else
            {
                return a.m_orgIndex < b.m_orgIndex;
            }
        }
    }
};



SIMD_INLINE void GrahamScanConvexHull2D(b3AlignedObjectArray<GrahamVector3>& originalPoints,
		                                b3AlignedObjectArray<GrahamVector3>& hull,
									    const Vector3& normalAxis);


SIMD_INLINE void GrahamScanConvexHull2D( b3AlignedObjectArray<Vector3> inputVertices   ,
                                         b3AlignedObjectArray<Vector3> &outputVertices ,
                                         const Vector3& normalAxis );

void wera();

} /* namespace real_physics */

#endif /* SOURCE_ENGIE_GEOMETRY_RPGRAHAMSCAN2DCONVEXHULL_H_ */
