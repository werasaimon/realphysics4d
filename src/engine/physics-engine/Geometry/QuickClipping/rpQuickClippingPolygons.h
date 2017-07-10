/*
 * rpClipingPoly.h
 *
 *  Created on: 2 дек. 2016 г.
 *      Author: wera
 */

#ifndef SOURCE_ENGIE_GEOMETRY_RPQUICKCLIPPNHPOLYGONS_H_
#define SOURCE_ENGIE_GEOMETRY_RPQUICKCLIPPNHPOLYGONS_H_


#include "../../LinearMaths/mathematics.h"
#include "../../Memory/memory.h"


#include <iostream>
using namespace std;

namespace real_physics
{


namespace
{

enum { INDICATOR_EDGE_POLY , INDICATOR_EDGE_CLIP_POLY };
enum { UNKNOWN = 10, P_IS_INSIDE, Q_IS_INSIDE };
enum { LEFT = 30, RIGHT, BEHIND };

enum { COLLINEAR = 40, PARALLEL, SKEW, SKEW_CROSS, SKEW_NO_CROSS };


struct rpEdge
{

    rpEdge( const Vector3& _a = Vector3(0,0,0) ,
            const Vector3& _b = Vector3(0,0,0))
        : mA(_a) , mB(_b)
    {

    }

    Vector3 ClosestPointOnLine(const Vector3& vPoint)
    {
        Vector3 vVector1 = vPoint - mA;
        Vector3 vVector2 = (mB - mA).getUnit();

        scalar d = (mA - mB).length();
        scalar t = vVector2.dot(vVector1);

        if (t < 0) return mA;
        if (t > d) return mB;

        Vector3 vVector3 = vVector2 * t;
        Vector3 vClosestPoint = mA + vVector3;
        return vClosestPoint;
    }



    bool isPointOutsideLine( const Vector3 &point )
    {
        //if( (point - mA).lengthSquare() <= scalar(0.001 * 0.001) ) return true;
        //if( (point - mB).lengthSquare() <= scalar(0.001 * 0.001) ) return true;

        if ((point - mA).dot(point - mB) > 0)  return false;
        return true;
    }

    Vector3 getLineDirection() const
    {
        return  (mB - mA);
    }

    Vector3 mA;
    Vector3 mB;
};





class rpPlane
{
public:


    // -------------------- Attributes --------------------- //

    Vector3 mNFace;
    Vector3 mOrigin;



    // -------------------- Constructor -------------------- //

    //rpPlane(){}

    rpPlane( Vector3 _n , Vector3 _origin)
        : mNFace(_n.getUnit()) , mOrigin(_origin)
    {

    }



    // -------------------- Method ------------------------- //

    int sideClassifyPointToEdge( const Vector3& _inVtx ) const
    {
        scalar length = (mOrigin - _inVtx).dot(mNFace);

        if (length >   0.f) return RIGHT;
        if (length <   0.f)  return LEFT;

        return BEHIND;
    }




    bool isAtLookToPoly(const Vector3& axisEdge,  int aclass ) const
    {
        Vector3 va = axisEdge;
        Vector3 vb = mNFace;

        scalar  v = vb.dot(va);

        if (v >= 0)
        {
            return (aclass != RIGHT);
        }
        else
        {
            return (aclass != LEFT);
        }
    }



    Vector3 vIntersectionLineVsPlane( const rpEdge& _edge ) const
    {
        Vector3 A = _edge.mA;
        Vector3 B = _edge.mB;

        Vector3 N = mNFace;
        Vector3 V = (mOrigin - A);
        Vector3 W = (A - B);

        scalar d = N.dot(V);
        scalar e = N.dot(W);

        if( e == scalar(0) ) return A;

        Vector3 PlaneIntersectLine = A + W * (d/e);

        return  PlaneIntersectLine;

    }


    Vector3 vProjectPointToPlane(const Vector3 &point) const
    {
        Vector3 normal = mNFace;
        scalar lenghtPenetration = (point - mOrigin).dot(normal);
        return  point - normal * lenghtPenetration;
    }


};



//	static bool InsidePolygon(Vector3 vIntersection, const Vector3* Poly, int verticeCount , rpPlane plane)
//	{
//
//		const scalar MATCH_FACTOR = 1.f - MACHINE_EPSILON;
//		scalar Angle = 0.0;
//		Vector3 vA, vB;
//		for (int i = 0; i < verticeCount; i++)
//		{
//			vA = Poly[i] - vIntersection;
//			vB = Poly[(i + 1) % verticeCount] - vIntersection;
//			vA = plane.vProjectPointToPlane(vA);
//			vB = plane.vProjectPointToPlane(vB);
//
//			Angle += vB.AngleBetweenVectors(vA);
//		}
//
//		if ((Angle >= (MATCH_FACTOR * (2.0 * Pi()))) ) return true;
//		return false;
//
//	}


static bool InsidePolygonSAT(const Vector3& vIntersection, const Vector3* Poly, int verticeCount , const rpPlane &plane)
{
    Vector3 centerPoly;
    for (int i = 0; i < verticeCount; i++) centerPoly += (Poly[i] / scalar(verticeCount));

    Vector3 vA, vB;
    Vector3 vNormalAxis;
    for (int i = 0; i < verticeCount; i++)
    {
        vA = Poly[i];
        vB = Poly[(i + 1) % verticeCount];

        vA = plane.vProjectPointToPlane(vA);
        vB = plane.vProjectPointToPlane(vB);

        vNormalAxis = (vA-vB).cross(plane.mNFace);
        vNormalAxis = (vNormalAxis.dot(vB - centerPoly) < 0 )? -vNormalAxis : vNormalAxis;

        if( (vIntersection - vB).dot(vNormalAxis) > 0 ) return false;
    }


    return true;
}


}





class rpQuickClippingPolygons
{

private:

    /// input
    int             mCountPolygonVertices;
    const Vector3  *mPolygonVertices = nullptr;

    int             mCountClipVertices;
    const Vector3  *mClipVertices = nullptr;

    /// output
    int                           mOutCountVertices;
    std::vector<Vector3>          mOutVertices;
    //*************************************************************//


    /// Private copy-constructor
    rpQuickClippingPolygons(const rpQuickClippingPolygons& clipping);

    /// Private assignment operator
    rpQuickClippingPolygons& operator=(const rpQuickClippingPolygons& clipping);


public:

    rpQuickClippingPolygons(const Vector3*  PolygonVertices , int  CountPolygonVertices ,
                            const Vector3*  ClipVertices    , int  CountClipVertices)
        : mPolygonVertices(PolygonVertices) , mCountPolygonVertices(CountPolygonVertices) ,
          mClipVertices(ClipVertices) , mCountClipVertices(CountClipVertices) ,
          mOutCountVertices(0)
    {
        assert(mPolygonVertices != NULL);
        assert(mClipVertices != NULL );
    }


    ~rpQuickClippingPolygons()
    {
        Destroy();
    }


    void Destroy()
    {
        mOutCountVertices = 0;
        mOutVertices.clear();
    }


    void addPoint( const Vector3& p )
    {
        mOutCountVertices++;
        mOutVertices.push_back(p);

    }

    //******************  Compute clipping **************************//

    bool isComputeClippingToPoly();




    //***************************************************************//

    int getSizeClipVertices() const
    {
        return mOutCountVertices;
    }

    const Vector3& getOutClippingPoint( int index ) const
    {
        return mOutVertices[index];
    }

    const std::vector<Vector3>& getOutVertices() const
    {
        return mOutVertices;
    }





private:

    const rpEdge MoveEdgeToIndex( int &end_index , int indicatorEDGE ) const;

    //****************   Help Function ****************************//
    static int   nextMoveIndexToEdges( bool bPlaneAIMSLookToLineA ,
                                       bool aPlaneAIMSLookToLineB ,
                                       bool isLeftClassification  ,
                                       int& moveIndexA ,
                                       int& moveIndexB ,
                                       bool exceptions ,
                                       bool isLookFaceToFace);
    static void   CoretionIndex( int &_out0, int &_out1, int _max, int &_i);
    static void   MoveIndex(int& index , bool isLookFaceToFace );
};









} /* namespace real_physics */

#endif /* SOURCE_ENGIE_GEOMETRY_RPQUICKCLIPPNHPOLYGONS_H_ */
