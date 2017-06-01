/*
 * rpSimplexSolverInterface.h
 *
 *  Created on: 19 нояб. 2016 г.
 *      Author: wera
 */

#ifndef SOURCE_ENGIE_COLLISION_NARROWPHASE_RPSIMPLEXSOLVERINTERFACE_H_
#define SOURCE_ENGIE_COLLISION_NARROWPHASE_RPSIMPLEXSOLVERINTERFACE_H_


#include "../../../../LinearMaths/mathematics.h"

namespace real_physics
{

#define NO_VIRTUAL_INTERFACE 1


/// btSimplexSolverInterface can incrementally calculate distance between origin and up to 4 vertices
/// Used by GJK or Linear Casting. Can be implemented by the Johnson-algorithm or alternative approaches based on
/// voronoi regions or barycentric coordinates
class rpSimplexSolverInterface
{
    public:

    //virtual  btSimplexSolverInterface() {}
    virtual ~rpSimplexSolverInterface() {}

    virtual void reset() = 0;

    virtual void addVertex(const Vector3& w, const Vector3& p, const Vector3& q) = 0;

    virtual bool closest(Vector3& v) = 0;

    virtual scalar maxVertex() = 0;

    virtual bool fullSimplex() const = 0;

    virtual int getSimplex(Vector3 *pBuf, Vector3 *qBuf, Vector3 *yBuf) const = 0;

    virtual bool inSimplex(const Vector3& w) = 0;

    virtual void backup_closest(Vector3& v) = 0;

    virtual bool emptySimplex() const = 0;

    virtual void compute_points(Vector3& p1, Vector3& p2) = 0;

    virtual int numVertices() const =0;


};


} /* namespace real_physics */

#endif /* SOURCE_ENGIE_COLLISION_NARROWPHASE_RPSIMPLEXSOLVERINTERFACE_H_ */
