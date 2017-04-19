/*
 * rpEdgeEPA.cpp
 *
 *  Created on: 19 нояб. 2016 г.
 *      Author: wera
 */


// Libraries
#include "rpEdgeEPA.h"
#include "rpTriangleEPA.h"
#include "rpTrianglesStore.h"
#include <cassert>


namespace real_physics
{


// Constructor
EdgeEPA::EdgeEPA()
{

}

// Constructor
EdgeEPA::EdgeEPA(TriangleEPA* ownerTriangle, int index)
        : mOwnerTriangle(ownerTriangle), mIndex(index)
{
    assert(index >= 0 && index < 3);
}

// Copy-constructor
EdgeEPA::EdgeEPA(const EdgeEPA& edge)
{
    mOwnerTriangle = edge.mOwnerTriangle;
    mIndex = edge.mIndex;
}

// Destructor
EdgeEPA::~EdgeEPA()
{

}

// Return the index of the source vertex of the edge (vertex starting the edge)
uint EdgeEPA::getSourceVertexIndex() const
{
    return (*mOwnerTriangle)[mIndex];
}

// Return the index of the target vertex of the edge (vertex ending the edge)
uint EdgeEPA::getTargetVertexIndex() const
{
    return (*mOwnerTriangle)[indexOfNextCounterClockwiseEdge(mIndex)];
}

// Execute the recursive silhouette algorithm from this edge
bool EdgeEPA::computeSilhouette(const Vector3* vertices, uint indexNewVertex,
                                TrianglesStore& triangleStore)
{
    // If the edge has not already been visited
    if (!mOwnerTriangle->getIsObsolete())
    {

        // If the triangle of this edge is not visible from the given point
        if (!mOwnerTriangle->isVisibleFromVertex(vertices, indexNewVertex))
        {
            TriangleEPA* triangle = triangleStore.newTriangle(vertices, indexNewVertex,
                                                              getTargetVertexIndex(),
                                                              getSourceVertexIndex());

            // If the triangle has been created
            if (triangle != NULL)

            {
                halfLink(EdgeEPA(triangle, 1), *this);
                return true;
            }

            return false;
        }
        else
        {

            // The current triangle is visible and therefore obsolete
            mOwnerTriangle->setIsObsolete(true);

            int backup = triangleStore.getNbTriangles();

            if(!mOwnerTriangle->getAdjacentEdge(indexOfNextCounterClockwiseEdge(
                                                this->mIndex)).computeSilhouette(vertices,
                                                                                 indexNewVertex,
                                                                                 triangleStore))
            {
                mOwnerTriangle->setIsObsolete(false);

                TriangleEPA* triangle = triangleStore.newTriangle(vertices, indexNewVertex,
                                                                  getTargetVertexIndex(),
                                                                  getSourceVertexIndex());

                // If the triangle has been created
                if (triangle != NULL)
                {
                    halfLink(EdgeEPA(triangle, 1), *this);
                    return true;
                }

                return false;
            }
            else if (!mOwnerTriangle->getAdjacentEdge(indexOfPreviousCounterClockwiseEdge(
                                                  this->mIndex)).computeSilhouette(vertices,
                                                                                   indexNewVertex,
                                                                                   triangleStore))
            {
                mOwnerTriangle->setIsObsolete(false);

                triangleStore.setNbTriangles(backup);

                TriangleEPA* triangle = triangleStore.newTriangle(vertices, indexNewVertex,
                                                                  getTargetVertexIndex(),
                                                                  getSourceVertexIndex());

                if (triangle != NULL)
                {
                    halfLink(EdgeEPA(triangle, 1), *this);
                    return true;
                }

                return false;
            }
        }
    }

    return true;
}


} /* namespace real_physics */
