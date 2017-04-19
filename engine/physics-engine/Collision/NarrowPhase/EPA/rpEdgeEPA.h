/*
 * rpEdgeEPA.h
 *
 *  Created on: 19 нояб. 2016 г.
 *      Author: wera
 */

#ifndef SOURCE_ENGIE_COLLISION_NARROWPHASE_EPA_RPEDGEEPA_H_
#define SOURCE_ENGIE_COLLISION_NARROWPHASE_EPA_RPEDGEEPA_H_


// Libraries
#include "../../../LinearMaths/mathematics.h"

namespace real_physics
{
// Class declarations
class TriangleEPA;
class TrianglesStore;

// Class EdgeEPA
/**
 * This class represents an edge of the current polytope in the EPA algorithm.
 */
class EdgeEPA
{

    private:

        // -------------------- Attributes -------------------- //

        /// Pointer to the triangle that contains this edge
        TriangleEPA* mOwnerTriangle;

        /// Index of the edge in the triangle (between 0 and 2).
        /// The edge with index i connect triangle vertices i and (i+1 % 3)
        int mIndex;

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        EdgeEPA();

        /// Constructor
        EdgeEPA(TriangleEPA* ownerTriangle, int index);

        /// Copy-constructor
        EdgeEPA(const EdgeEPA& edge);

        /// Destructor
        ~EdgeEPA();

        /// Return the pointer to the owner triangle
        TriangleEPA* getOwnerTriangle() const;

        /// Return the index of the edge in the triangle
        int getIndex() const;

        /// Return index of the source vertex of the edge
        uint getSourceVertexIndex() const;

        /// Return the index of the target vertex of the edge
        uint getTargetVertexIndex() const;

        /// Execute the recursive silhouette algorithm from this edge
        bool computeSilhouette(const Vector3* vertices, uint index, TrianglesStore& triangleStore);

        /// Assignment operator
        EdgeEPA& operator=(const EdgeEPA& edge);
};

// Return the pointer to the owner triangle
SIMD_INLINE TriangleEPA* EdgeEPA::getOwnerTriangle() const
{
    return mOwnerTriangle;
}

// Return the edge index
SIMD_INLINE int EdgeEPA::getIndex() const
{
    return mIndex;
}

// Assignment operator
SIMD_INLINE EdgeEPA& EdgeEPA::operator=(const EdgeEPA& edge)
{
    mOwnerTriangle = edge.mOwnerTriangle;
    mIndex = edge.mIndex;
    return *this;
}

// Return the index of the next counter-clockwise edge of the ownver triangle
SIMD_INLINE int indexOfNextCounterClockwiseEdge(int i)
{
    return (i + 1) % 3;
}

// Return the index of the previous counter-clockwise edge of the ownver triangle
SIMD_INLINE int indexOfPreviousCounterClockwiseEdge(int i)
{
    return (i + 2) % 3;
}


} /* namespace real_physics */

#endif /* SOURCE_ENGIE_COLLISION_NARROWPHASE_EPA_RPEDGEEPA_H_ */
