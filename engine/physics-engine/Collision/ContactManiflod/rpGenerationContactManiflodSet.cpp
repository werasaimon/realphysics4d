/*
 * rpGenerationContactManiflodSet.cpp
 *
 *  Created on: 29 нояб. 2016 г.
 *      Author: wera
 */

#include "rpGenerationContactManiflodSet.h"
#include <iostream>

using namespace std;

namespace real_physics
{



namespace
{

	static Vector3 ClosestPointOnLine(Vector3 vA, Vector3 vB, Vector3 vPoint)
	{
		Vector3 vVector1 = vPoint - vA;
		Vector3 vVector2 = (vB - vA).getUnit();

		scalar d = (vA - vB).length();
		scalar t = (vVector2.dot(vVector1));

		if (t < 0) return vA;
		if (t > d) return vB;

		Vector3 vVector3 = vVector2 * t;
		Vector3 vClosestPoint = vA + vVector3;
		return vClosestPoint;
	}

	static bool intersectionLineToLine(Vector3 start1, Vector3 end1,
									   Vector3 start2, Vector3 end2, Vector3 &ip)
	{

		Vector3 da = end1 - start1;
		Vector3 db = end2 - start2;
		Vector3 dc = start2 - start1;

		if (dc.dot(da.cross(db)) != 0.0) // lines are not coplanar
			return false;

		scalar s = (dc.cross(db)).dot((da.cross(db)))
				/ ((da.cross(db)).lengthSquare());
		if (s >= 0.0 && s <= 1.0)
		{
			ip = start1 + da * (Vector3(s, s, s));
			return true;
		}

		return false;
	}


}

rpGenerationContactManiflodSet::rpGenerationContactManiflodSet( const rpProxyShape* shape1, const rpProxyShape* shape2,
		                                                        const Vector3& Axis) :
mShape1(shape1), mShape2(shape2), mSeparatonAxis(Axis), mNbContacts(0)
{
	mSeparatonAxis.normalize();
}

rpGenerationContactManiflodSet::~rpGenerationContactManiflodSet()
{
	// TODO Auto-generated destructor stub
}

//------------------------------------------------ method --------------------------------------------------------//

SIMD_INLINE void rpGenerationContactManiflodSet::CollidePointPointContacts(const Vector3& A, const Vector3& B)
{

	scalar penetration = (A - B).dot(mSeparatonAxis);
	rpContactPointInfo info(mSeparatonAxis, penetration, A, B);
	//ContactPoint CollidPoint( info );
	addInfoContact(info);
}

SIMD_INLINE void rpGenerationContactManiflodSet::CollidePointFaceContacts(const Vector3& A, const Vector3& xAxis, scalar bd)
{
	scalar dist = (A.dot(xAxis)) - bd;
	Vector3 B = A - dist * xAxis;

	scalar penetration = (A - B).dot(mSeparatonAxis);
	rpContactPointInfo info(mSeparatonAxis, penetration, A, B);
	//ContactPoint CollidPoint( info );
	addInfoContact(info);
}

SIMD_INLINE void rpGenerationContactManiflodSet::CollidePointEdgeContacts(const Vector3& A, const Vector3& B0, const Vector3& B1)
{

	Vector3 B0A = A - B0;
	Vector3 BD = B1 - B0;

	scalar t = (B0A.dot(BD)) / (BD.dot(BD));
	t = Clamp(t, 0.0f, 1.f);

	Vector3 B = B0 + t * BD;
	B = ClosestPointOnLine(B0, B1, A);

	scalar penetration = (A - B).dot(mSeparatonAxis);
	rpContactPointInfo info(mSeparatonAxis, penetration, A, B);
	//ContactPoint CollidPoint( info );
	addInfoContact(info);

}

SIMD_INLINE void rpGenerationContactManiflodSet::CollideEdgeEdgeContacts(const Vector3& A0, const Vector3& A1,
		                                                     const Vector3& B0, const Vector3& B1)
{

	Vector3 AD = A1 - A0;
	Vector3 BD = B1 - B0;
	Vector3 N = AD.cross(BD);

	Vector3 M = N.cross(BD);
	scalar md = M.dot(B0);

	scalar at = (md - A0.dot(M)) / (AD.dot(M));
	at = Clamp(at, 0.0f, 1.f);

	/**/
	Vector3 AA = A0 + at * AD;

	intersectionLineToLine(A0, A1, B0, B1, AA);

	Vector3 BB = ClosestPointOnLine(B0, B1, AA);
	scalar penetration = (AA - BB).dot(mSeparatonAxis);
	/**/

	rpContactPointInfo info(mSeparatonAxis, penetration, AA, BB);
	//ContactPoint CollidPoint( info );
	addInfoContact(info);

}

//======================= Algoritm for Cliping ===================================//

SIMD_INLINE void rpGenerationContactManiflodSet::CollidePolygonContacts(const Vector3* Poly, int iPolySize,
		                                                                const Vector3* Clipper, int iClipperSize)
{

	real_physics::rpPolygonClipping polyClipping(Poly, iPolySize, Clipper,iClipperSize);
	if (polyClipping.isComputeClippingToPoly())
	{
		Vector3 ClipperNormal = Vector3::planeNormal(Poly[0], Poly[1], Poly[2]);
		scalar clipper_d = Poly[0].dot(ClipperNormal);

		for (int i = 0; i < polyClipping.getSizeClipVertices(); i++)
		{
			Vector3 PB = polyClipping[i];
			scalar dist = (PB.dot(ClipperNormal)) - clipper_d;

			if ((dist) <= 0)
			{
				Vector3 A = PB;
				Vector3 B = PB - ClipperNormal * dist;
				scalar penetration = (A - B).dot(mSeparatonAxis);
				//scalar penetration = (A - B).dot(ClipperNormal);

				rpContactPointInfo info(mSeparatonAxis, penetration = dist , A, B);
				addInfoContact(info);

			}
		}
	}
}

//==============================================================================//

SIMD_INLINE bool rpGenerationContactManiflodSet::ConvertSupportPointsToContacts(const Vector3* SupportVertA, int iNumVertsA,
		                                                                        const Vector3* SupportVertB, int iNumVertsB)

{

	if (iNumVertsA == 0 || iNumVertsB == 0)
		return false;

	mNbContacts = 0;

	if (iNumVertsA == 1)
	{
		if (iNumVertsB == 1)
		{
			CollidePointPointContacts(SupportVertA[0], SupportVertB[0]);
		}
		else if (iNumVertsB == 2)
		{
			CollidePointEdgeContacts(SupportVertA[0], SupportVertB[0],SupportVertB[1]);
		}
		else
		{
			Vector3 Bn = Vector3::planeNormal(SupportVertB[0], SupportVertB[1],SupportVertB[2]);
			scalar bd = Bn.dot(SupportVertB[0]);

			CollidePointFaceContacts(SupportVertA[0], Bn, bd);
		}
	}
	else if (iNumVertsA == 2)
	{
		if (iNumVertsB == 1)
		{
			CollidePointEdgeContacts(SupportVertB[0], SupportVertA[0], SupportVertA[1]);

		}
		else if (iNumVertsB == 2)
		{
			CollideEdgeEdgeContacts(SupportVertA[0], SupportVertA[1],
					                SupportVertB[0], SupportVertB[1]);

		}
		else
		{
			CollidePolygonContacts(SupportVertB, iNumVertsB, SupportVertA, iNumVertsA);
		}
	}
	else if (iNumVertsA >= 3)
	{

		if (iNumVertsB == 1)
		{

			Vector3 An = Vector3::planeNormal(SupportVertA[0], SupportVertA[1], SupportVertA[2]);
			scalar bd = An.dot(SupportVertA[0]);

			CollidePointFaceContacts(SupportVertB[0], An, bd);
		}
		else if (iNumVertsB == 2)
		{

			CollidePolygonContacts(SupportVertA, iNumVertsA, SupportVertB,iNumVertsB);

		}
		else if (iNumVertsB >= 3)
		{

			Vector3 normalA = Vector3::planeNormal(SupportVertA[0],SupportVertA[1], SupportVertA[2]);
			Vector3 normalB = Vector3::planeNormal(SupportVertB[0],SupportVertB[1], SupportVertB[2]);

			scalar angleA = normalA.AngleBetweenVectors( mSeparatonAxis);
			scalar angleB = normalB.AngleBetweenVectors(-mSeparatonAxis);

			//Swap(angleA , angleB);

			if (angleA <= angleB)
			{
				CollidePolygonContacts(SupportVertA, iNumVertsA, SupportVertB,iNumVertsB);

			}
			else
			{
				CollidePolygonContacts(SupportVertB, iNumVertsB, SupportVertA,iNumVertsA);
			}

		}
	}

	return (mNbContacts > 0);

}

void rpGenerationContactManiflodSet::computeContacteManiflodSet( rpContactManifoldSet& maniflodSet , bool approximationCorretion )
{

	Vector3 Normal = mSeparatonAxis;

	uint iNumVertsA = 0;
	uint iNumVertsB = 0;

	Vector3* SupportVertA = mShape1->getAxisPeturberationPoints( Normal, iNumVertsA);
	Vector3* SupportVertB = mShape2->getAxisPeturberationPoints(-Normal, iNumVertsB);

	bool isOutside = ConvertSupportPointsToContacts(SupportVertA, iNumVertsA,
			                                        SupportVertB, iNumVertsB);

	free(SupportVertA);
	free(SupportVertB);

//	delete[] SupportVertA;
//	delete[] SupportVertB;


	if (isOutside)
	{


        maniflodSet.update();
		for (uint i = 0; i < mNbContacts; ++i)
		{

			mInfoContacts[i].normal = -mInfoContacts[i].normal;

			if( approximationCorretion )
			{
				Vector3 normal = mInfoContacts[i].normal;
				Vector3 point1 = mInfoContacts[i].localPoint1;
				Vector3 point2 = mInfoContacts[i].localPoint2;
				//mInfoContacts[i].penetrationDepth = (point2 - point1).dot(normal);
				Vector3 approximationPoint =  (point1 + point2) * scalar(0.5);
				mInfoContacts[i].localPoint1 = approximationPoint;
				mInfoContacts[i].localPoint2 = approximationPoint;
			}


			rpContactPoint* contact = new rpContactPoint(mInfoContacts[i]);
			maniflodSet.addContactPoint(contact);
		}

	}


//	for (int i = 0; i < iNumVertsA; ++i)
//	{
//	    Vector3 p = SupportVertA[i];
//
//	    glPushMatrix();
//	    glColor3f(1, 1, 1);
//	     glTranslatef(p.x, p.y, p.z);
//	     glutWireCube(0.2);
//	    glPopMatrix();
//	}
//
//
//	for (int i = 0; i < iNumVertsB; ++i)
//	{
//	    Vector3 p = SupportVertB[i];
//
//	    glPushMatrix();
//	     glColor3f(1, 1, 1);
//	     glTranslatef(p.x, p.y, p.z);
//	     glutWireCube(0.2);
//	    glPopMatrix();
//	}

}

} /* namespace real_physics */

