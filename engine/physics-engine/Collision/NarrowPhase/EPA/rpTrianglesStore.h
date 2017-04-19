/*
 * rpTrianglesStore.h
 *
 *  Created on: 19 нояб. 2016 г.
 *      Author: wera
 */

#ifndef SOURCE_ENGIE_COLLISION_NARROWPHASE_EPA_RPTRIANGLESSTORE_H_
#define SOURCE_ENGIE_COLLISION_NARROWPHASE_EPA_RPTRIANGLESSTORE_H_


// Libraries
#include <cassert>
#include "rpTriangleEPA.h"



namespace real_physics
{

// Constants
const unsigned int MAX_TRIANGLES = 200;     // Maximum number of triangles

// Class TriangleStore
/**
 * This class stores several triangles of the polytope in the EPA algorithm.
 */
class TrianglesStore
{

    private:

        // -------------------- Attributes -------------------- //

        /// Triangles
        TriangleEPA mTriangles[MAX_TRIANGLES];

        /// Number of triangles
        int mNbTriangles;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        TrianglesStore(const TrianglesStore& triangleStore);

        /// Private assignment operator
        TrianglesStore& operator=(const TrianglesStore& triangleStore);

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        TrianglesStore();

        /// Destructor
        ~TrianglesStore();

        /// Clear all the storage
        void clear();

        /// Return the number of triangles
        int getNbTriangles() const;

        /// Set the number of triangles
        void setNbTriangles(int backup);

        /// Return the last triangle
        TriangleEPA& last();

        /// Create a new triangle
        TriangleEPA* newTriangle(const Vector3* vertices, uint v0, uint v1, uint v2);

        /// Access operator
        TriangleEPA& operator[](int i);
};

// Clear all the storage
SIMD_INLINE void TrianglesStore::clear()
{
    mNbTriangles = 0;
}

// Return the number of triangles
SIMD_INLINE int TrianglesStore::getNbTriangles() const
{
    return mNbTriangles;
}


SIMD_INLINE void TrianglesStore::setNbTriangles(int backup)
{
    mNbTriangles = backup;
}

// Return the last triangle
SIMD_INLINE TriangleEPA& TrianglesStore::last()
{
    assert(mNbTriangles > 0);
    return mTriangles[mNbTriangles - 1];
}

// Create a new triangle
SIMD_INLINE TriangleEPA* TrianglesStore::newTriangle(const Vector3* vertices,
                                                uint v0,uint v1, uint v2)
{
    TriangleEPA* newTriangle = NULL;

    // If we have not reached the maximum number of triangles
    if (mNbTriangles != MAX_TRIANGLES)
    {
        newTriangle = &mTriangles[mNbTriangles++];
        new (newTriangle) TriangleEPA(v0, v1, v2);
        if (!newTriangle->computeClosestPoint(vertices))
        {
            mNbTriangles--;
            newTriangle = NULL;
        }
    }

    // Return the new triangle
    return newTriangle;
}

// Access operator
SIMD_INLINE TriangleEPA& TrianglesStore::operator[](int i)
{
    return mTriangles[i];
}

} /* namespace real_physics */

#endif /* SOURCE_ENGIE_COLLISION_NARROWPHASE_EPA_RPTRIANGLESSTORE_H_ */
