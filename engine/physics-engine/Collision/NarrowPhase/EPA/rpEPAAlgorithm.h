/*
 * rpEPAAlgorithm.h
 *
 *  Created on: 19 нояб. 2016 г.
 *      Author: wera
 */

#ifndef SOURCE_ENGIE_COLLISION_NARROWPHASE_EPA_RPEPAALGORITHM_H_
#define SOURCE_ENGIE_COLLISION_NARROWPHASE_EPA_RPEPAALGORITHM_H_


// Libraries
#include "../GJK/rpSimplex.h"
#include "../../Shapes/rpCollisionShape.h"
#include "../../rpCollisionShapeInfo.h"
#include "../../../LinearMaths/mathematics.h"
#include "rpTriangleEPA.h"
#include <algorithm>

namespace real_physics
{

// ---------- Constants ---------- //

/// Maximum number of support points of the polytope
const unsigned int MAX_SUPPORT_POINTS = 100;

/// Maximum number of facets of the polytope
const unsigned int MAX_FACETS = 200;


// Class TriangleComparison
/**
 * This class allows the comparison of two triangles in the heap
 * The comparison between two triangles is made using their square distance to the closest
 * point to the origin. The goal is that in the heap, the first triangle is the one with the
 * smallest square distance.
 */
class TriangleComparison
{

    public:

        /// Comparison operator
        bool operator()(const TriangleEPA* face1, const TriangleEPA* face2)
        {
            return (face1->getDistSquare() > face2->getDistSquare());
        }
};



class OutContactInfo
{
     public:


	OutContactInfo()
    {

    }

	OutContactInfo( const Vector3& _normal ,  const scalar&  _depth ,
			        const Vector3& _ALocal ,  const Vector3& _BLocal )
     :m_normal(_normal),
	  m_penetrationDepth(_depth),
	  pALocal(_ALocal),
	  pBLocal(_BLocal)
     {

     }

	//Vector3 v;

	Vector3 m_normal;
	scalar  m_penetrationDepth;
	Vector3 pALocal;
	Vector3 pBLocal;


};




class rpConvexShape;


template <typename btConvexTemplate>
struct	MinkowskiDifferens
{
	const btConvexTemplate* mConvexShape1;
    const btConvexTemplate* mConvexShape2;

	//Matrix3x3				m_toshape1;
	//Transform				m_toshape0;


	Transform				m_toshape1;
	Transform				m_toshape0;


	void** shape1CachedCollisionData;
	void** shape2CachedCollisionData;


	SIMD_INLINE Vector3	Support0(const Vector3& d) const
	{
		return  m_toshape0 *mConvexShape1->getLocalSupportPointWithMargin(m_toshape0.getOrientation().getInverse() * d, shape1CachedCollisionData);
	}


	SIMD_INLINE Vector3	Support1(const Vector3& d) const
	{
		return  m_toshape1 *mConvexShape2->getLocalSupportPointWithMargin(m_toshape1.getOrientation().getInverse() * d, shape2CachedCollisionData);
	}

};




// Class EPAAlgorithm
/**
 * This class is the implementation of the Expanding Polytope Algorithm (EPA).
 * The EPA algorithm computes the penetration depth and contact points between
 * two enlarged objects (with margin) where the original objects (without margin)
 * intersect. The penetration depth of a pair of intersecting objects A and B is
 * the length of a point on the boundary of the Minkowski sum (A-B) closest to the
 * origin. The goal of the EPA algorithm is to start with an initial simplex polytope
 * that contains the origin and expend it in order to find the point on the boundary
 * of (A-B) that is closest to the origin. An initial simplex that contains origin
 * has been computed wit GJK algorithm. The EPA Algorithm will extend this simplex
 * polytope to find the correct penetration depth. The implementation of the EPA
 * algorithm is based on the book "Collision Detection in 3D Environments".
 */
class EPAAlgorithm
{

    private:

        // -------------------- Attributes -------------------- //


        /// Triangle comparison operator
        TriangleComparison mTriangleComparison;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        EPAAlgorithm(const EPAAlgorithm& algorithm);

        /// Private assignment operator
        EPAAlgorithm& operator=(const EPAAlgorithm& algorithm);

        /// Add a triangle face in the candidate triangle heap
        void addFaceCandidate(TriangleEPA* triangle, TriangleEPA** heap, uint& nbTriangles,
                              scalar upperBoundSquarePenDepth);

        /// Decide if the origin is in the tetrahedron.
        int isOriginInTetrahedron(const Vector3& p1, const Vector3& p2,
                                  const Vector3& p3, const Vector3& p4) const;

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        EPAAlgorithm();

        /// Destructor
        ~EPAAlgorithm();



        /// Compute the penetration depth with EPA algorithm.
        bool computePenetrationDepthAndContactPoints(const Simplex& simplex,
                                                     CollisionShapeInfo shape1Info, const Transform& transform1,
                                                     CollisionShapeInfo shape2Info, const Transform& transform2,
													 const MinkowskiDifferens<const rpConvexShape> Minkowski ,
                                                     Vector3& v , OutContactInfo& _outInfo );

};

// Add a triangle face in the candidate triangle heap in the EPA algorithm
SIMD_INLINE void EPAAlgorithm::addFaceCandidate(TriangleEPA* triangle, TriangleEPA** heap,
                                           uint& nbTriangles, scalar upperBoundSquarePenDepth)
{

    // If the closest point of the affine hull of triangle
    // points is internal to the triangle and if the distance
    // of the closest point from the origin is at most the
    // penetration depth upper bound
    if (triangle->isClosestPointInternalToTriangle() &&
        triangle->getDistSquare() <= upperBoundSquarePenDepth)
    {

        // Add the triangle face to the list of candidates
        heap[nbTriangles] = triangle;
        nbTriangles++;
        std::push_heap(&heap[0], &heap[nbTriangles], mTriangleComparison);
    }
}




} /* namespace real_physics */

#endif /* SOURCE_ENGIE_COLLISION_NARROWPHASE_EPA_RPEPAALGORITHM_H_ */
