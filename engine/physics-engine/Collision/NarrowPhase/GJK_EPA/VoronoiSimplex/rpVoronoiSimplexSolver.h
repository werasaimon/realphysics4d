/*
 * rpVoronoiSimplexSolver.h
 *
 *  Created on: 19 нояб. 2016 г.
 *      Author: wera
 */

#ifndef SOURCE_ENGIE_COLLISION_NARROWPHASE_RPVORONOISIMPLEXSOLVER_H_
#define SOURCE_ENGIE_COLLISION_NARROWPHASE_RPVORONOISIMPLEXSOLVER_H_



#include "rpSimplexSolverInterface.h"


namespace real_physics
{


#define VORONOI_SIMPLEX_MAX_VERTS 5

///disable next define, or use defaultCollisionConfiguration->getSimplexSolver()->setEqualVertexThreshold(0.f) to disable/configure
#define BT_USE_EQUAL_VERTEX_THRESHOLD
#define VORONOI_DEFAULT_EQUAL_VERTEX_THRESHOLD 0.0001f


struct rpUsageBitfield
{
    rpUsageBitfield()
    {
        reset();
    }

    void reset()
    {
        usedVertexA = false;
        usedVertexB = false;
        usedVertexC = false;
        usedVertexD = false;
    }
    unsigned short usedVertexA	: 1;
    unsigned short usedVertexB	: 1;
    unsigned short usedVertexC	: 1;
    unsigned short usedVertexD	: 1;
    unsigned short unused1		: 1;
    unsigned short unused2		: 1;
    unsigned short unused3		: 1;
    unsigned short unused4		: 1;
};


struct	rpSubSimplexClosestResult
{
    Vector3	m_closestPointOnSimplex;
    //MASK for m_usedVertices
    //stores the simplex vertex-usage, using the MASK,
    // if m_usedVertices & MASK then the related vertex is used
    rpUsageBitfield	m_usedVertices;
    scalar	        m_barycentricCoords[4];
    bool            m_degenerate;

    void	reset()
    {
        m_degenerate = false;
        setBarycentricCoordinates();
        m_usedVertices.reset();
    }


    bool	isValid()
    {
        bool valid =    (m_barycentricCoords[0] >= scalar(0.)) &&
                        (m_barycentricCoords[1] >= scalar(0.)) &&
                        (m_barycentricCoords[2] >= scalar(0.)) &&
                        (m_barycentricCoords[3] >= scalar(0.));
        return valid;
    }



    void	setBarycentricCoordinates(scalar a=scalar(0.),scalar b=scalar(0.),scalar c=scalar(0.),scalar d=scalar(0.))
    {
        m_barycentricCoords[0] = a;
        m_barycentricCoords[1] = b;
        m_barycentricCoords[2] = c;
        m_barycentricCoords[3] = d;
    }

};

/// rpVoronoiSimplexSolver is an implementation of the closest point distance algorithm from a 1-4 points simplex to the origin.
/// Can be used with GJK, as an alternative to Johnson distance algorithm.
 class rpVoronoiSimplexSolver : public rpSimplexSolverInterface
{
  public:

   // BT_DECLARE_ALIGNED_ALLOCATOR();

    int	m_numVertices;

    Vector3	m_simplexVectorW[VORONOI_SIMPLEX_MAX_VERTS];
    Vector3	m_simplexPointsP[VORONOI_SIMPLEX_MAX_VERTS];
    Vector3	m_simplexPointsQ[VORONOI_SIMPLEX_MAX_VERTS];



    Vector3	m_cachedP1;
    Vector3	m_cachedP2;
    Vector3	m_cachedV;
    Vector3	m_lastW;

    scalar	m_equalVertexThreshold;
    bool	m_cachedValidClosest;


    rpSubSimplexClosestResult m_cachedBC;

    bool	m_needsUpdate;

    void	removeVertex(int index);
    void	reduceVertices (const rpUsageBitfield& usedVerts);
    bool	updateClosestVectorAndPoints();

    bool	closestPtPointTetrahedron(const Vector3& p, const Vector3& a, const Vector3& b, const Vector3& c, const Vector3& d, rpSubSimplexClosestResult& finalResult);
    int		pointOutsideOfPlane(const Vector3& p, const Vector3& a, const Vector3& b, const Vector3& c, const Vector3& d);
    bool	closestPtPointTriangle(const Vector3& p, const Vector3& a, const Vector3& b, const Vector3& c,rpSubSimplexClosestResult& result);

public:

    rpVoronoiSimplexSolver()
    :m_equalVertexThreshold(VORONOI_DEFAULT_EQUAL_VERTEX_THRESHOLD)
    {
    }
     void reset();

     void addVertex(const Vector3& w, const Vector3& p, const Vector3& q);

     void	setEqualVertexThreshold(scalar threshold)
     {
         m_equalVertexThreshold = threshold;
     }

     scalar	getEqualVertexThreshold() const
     {
         return m_equalVertexThreshold;
     }

     bool closest(Vector3& v);

     scalar maxVertex();

     bool fullSimplex() const
     {
         return (m_numVertices == 4);
     }

     int getSimplex(Vector3 *pBuf, Vector3 *qBuf, Vector3 *yBuf) const;

     bool inSimplex(const Vector3& w);

     void backup_closest(Vector3& v) ;

     bool emptySimplex() const ;

     void compute_points(Vector3& p1, Vector3& p2) ;

     int numVertices() const
     {
         return m_numVertices;
     }



};


} /* namespace real_physics */

#endif /* SOURCE_ENGIE_COLLISION_NARROWPHASE_RPVORONOISIMPLEXSOLVER_H_ */
