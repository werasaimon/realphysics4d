/*
 * rpTriangleEPA.h
 *
 *  Created on: 19 нояб. 2016 г.
 *      Author: wera
 */

#ifndef SOURCE_ENGIE_COLLISION_NARROWPHASE_EPA_RPTRIANGLEEPA_H_
#define SOURCE_ENGIE_COLLISION_NARROWPHASE_EPA_RPTRIANGLEEPA_H_


// Libraries
#include "../../../LinearMaths/mathematics.h"
#include "rpEdgeEPA.h"
#include <cassert>

namespace real_physics
{

// Prototypes
bool link(const EdgeEPA& edge0, const EdgeEPA& edge1);
void halfLink(const EdgeEPA& edge0, const EdgeEPA& edge1);


// Class TriangleEPA
/**
 * This class represents a triangle face of the current polytope in the EPA algorithm.
 */
class TriangleEPA
{

    private:

        // -------------------- Attributes -------------------- //

        /// Indices of the vertices y_i of the triangle
        uint mIndicesVertices[3];

        /// Three adjacent edges of the triangle (edges of other triangles)
        EdgeEPA mAdjacentEdges[3];

        /// True if the triangle face is visible from the new support point
        bool mIsObsolete;

        /// Determinant
        scalar mDet;

        /// Point v closest to the origin on the affine hull of the triangle
        Vector3 mClosestPoint;

        /// Lambda1 value such that v = lambda0 * y_0 + lambda1 * y_1 + lambda2 * y_2
        scalar mLambda1;

        /// Lambda1 value such that v = lambda0 * y_0 + lambda1 * y_1 + lambda2 * y_2
        scalar mLambda2;

        /// Square distance of the point closest point v to the origin
        scalar mDistSquare;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        TriangleEPA(const TriangleEPA& triangle);

        /// Private assignment operator
        TriangleEPA& operator=(const TriangleEPA& triangle);

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        TriangleEPA();

        /// Constructor
        TriangleEPA(uint v1, uint v2, uint v3);

        /// Destructor
        ~TriangleEPA();

        /// Return an adjacent edge of the triangle
        EdgeEPA& getAdjacentEdge(int index);

        /// Set an adjacent edge of the triangle
        void setAdjacentEdge(int index, EdgeEPA& edge);

        /// Return the square distance of the closest point to origin
        scalar getDistSquare() const;

        /// Set the isObsolete value
        void setIsObsolete(bool isObsolete);

        /// Return true if the triangle face is obsolete
        bool getIsObsolete() const;

        /// Return the point closest to the origin
        const Vector3& getClosestPoint() const;

        // Return true if the closest point on affine hull is inside the triangle
        bool isClosestPointInternalToTriangle() const;

        /// Return true if the triangle is visible from a given vertex
        bool isVisibleFromVertex(const Vector3* vertices, uint index) const;

        /// Compute the point v closest to the origin of this triangle
        bool computeClosestPoint(const Vector3* vertices);

        /// Compute the point of an object closest to the origin
        Vector3 computeClosestPointOfObject(const Vector3* supportPointsOfObject) const;

        /// Execute the recursive silhouette algorithm from this triangle face.
        bool computeSilhouette(const Vector3* vertices, uint index, TrianglesStore& triangleStore);

        /// Access operator
        uint operator[](int i) const;

        /// Associate two edges
        friend bool link(const EdgeEPA& edge0, const EdgeEPA& edge1);

        /// Make a half-link between two edges
        friend void halfLink(const EdgeEPA& edge0, const EdgeEPA& edge1);
};

// Return an edge of the triangle
SIMD_INLINE EdgeEPA& TriangleEPA::getAdjacentEdge(int index)
{
    assert(index >= 0 && index < 3);
    return mAdjacentEdges[index];
}

// Set an adjacent edge of the triangle
SIMD_INLINE void TriangleEPA::setAdjacentEdge(int index, EdgeEPA& edge)
{
    assert(index >=0 && index < 3);
    mAdjacentEdges[index] = edge;
}

// Return the square distance  of the closest point to origin
SIMD_INLINE scalar TriangleEPA::getDistSquare() const
{
    return mDistSquare;
}

// Set the isObsolete value
SIMD_INLINE void TriangleEPA::setIsObsolete(bool isObsolete)
{
    mIsObsolete = isObsolete;
}

// Return true if the triangle face is obsolete
SIMD_INLINE bool TriangleEPA::getIsObsolete() const
{
    return mIsObsolete;
}

// Return the point closest to the origin
SIMD_INLINE const Vector3& TriangleEPA::getClosestPoint() const
{
    return mClosestPoint;
}

// Return true if the closest point on affine hull is inside the triangle
SIMD_INLINE bool TriangleEPA::isClosestPointInternalToTriangle() const
{
    return (mLambda1 >= 0.0 && mLambda2 >= 0.0 && (mLambda1 + mLambda2) <= mDet);
}

// Return true if the triangle is visible from a given vertex
SIMD_INLINE bool TriangleEPA::isVisibleFromVertex(const Vector3* vertices, uint index) const
{
    Vector3 closestToVert = vertices[index] - mClosestPoint;
    return (mClosestPoint.dot(closestToVert) > 0.0);
}

// Compute the point of an object closest to the origin
SIMD_INLINE Vector3 TriangleEPA::computeClosestPointOfObject(const Vector3* supportPointsOfObject) const
{
    const Vector3& p0 = supportPointsOfObject[mIndicesVertices[0]];
    return p0 + scalar(1.0)/mDet * (mLambda1 * (supportPointsOfObject[mIndicesVertices[1]] - p0) +
                           mLambda2 * (supportPointsOfObject[mIndicesVertices[2]] - p0));
}

// Access operator
SIMD_INLINE uint TriangleEPA::operator[](int i) const
{
    assert(i >= 0 && i <3);
    return mIndicesVertices[i];
}

} /* namespace real_physics */

#endif /* SOURCE_ENGIE_COLLISION_NARROWPHASE_EPA_RPTRIANGLEEPA_H_ */
