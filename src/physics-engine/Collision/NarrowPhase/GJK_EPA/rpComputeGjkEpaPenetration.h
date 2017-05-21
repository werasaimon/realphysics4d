/*
 * rpComputeGjkEpaPenetration.h
 *
 *  Created on: 19 нояб. 2016 г.
 *      Author: wera
 */

#ifndef SOURCE_ENGIE_COLLISION_NARROWPHASE_RPCOMPUTEGJKEPAPENETRATION_H_
#define SOURCE_ENGIE_COLLISION_NARROWPHASE_RPCOMPUTEGJKEPAPENETRATION_H_


#include <assert.h>

#include "../../../LinearMaths/mathematics.h" // Note that Vector3 might be double precision...
#include "../GJK_EPA/rpGjkCollisionDescription.h"
#include "../GJK_EPA/VoronoiSimplex/rpVoronoiSimplexSolver.h"
#include "rpGjkEpa.h"


namespace real_physics
{

#define BT_LARGE_FLOAT 1e30
#define SIMD_EPSILON 1e-6


template<typename ConvexTemplate>
bool GjkEpaCalcPenDepth(const ConvexTemplate &a,
                        const ConvexTemplate &b,
                        Vector3 &normal,
                        Vector3 &wWitnessOnA,
                        Vector3 &wWitnessOnB,
                        scalar  &wDepth)
{

    Vector3	guessVector(b.getWorldTransform().getPosition() -
                        a.getWorldTransform().getPosition());//?? why not use the GJK input?

    rpGjkEpaSolver::sResults	results;

    if(rpGjkEpaSolver::Penetration(a,b,guessVector,results))
    {
        wWitnessOnA = results.witnesses[0];
        wWitnessOnB = results.witnesses[1];

        normal = results.normal;
        wDepth = results.distance;

        return true;

    }
    else
    {
        if(rpGjkEpaSolver::Distance(a,b,guessVector,results))
        {
            wWitnessOnA = results.witnesses[0];
            wWitnessOnB = results.witnesses[1];

            normal = results.normal;
            wDepth = results.distance;

            return false;
        }
    }


    return false;

}





template<typename ConvexTemplate>
bool EpaCalcPenDepth(const ConvexTemplate &a,
                     const ConvexTemplate &b,
                     Vector3 &normal,
                     Vector3 &wWitnessOnA,
                     Vector3 &wWitnessOnB,
                     scalar  &wDepth)
{

    Vector3	guessVector(b.getWorldTransform().getPosition() -
                        a.getWorldTransform().getPosition());//?? why not use the GJK input?

    rpGjkEpaSolver::sResults	results;
    if(rpGjkEpaSolver::Penetration(a,b,guessVector,results))
    {
        wWitnessOnA = results.witnesses[0];
        wWitnessOnB = results.witnesses[1];

        normal = results.normal;
        wDepth = results.distance;

        return true;
    }

    return false;

}


template <typename ConvexTemplate, typename GjkDistanceTemplate>
int	  ComputeGjkEpaPenetrationSimplex(const ConvexTemplate& a,
                                      const ConvexTemplate& b,
                                      const rpGjkCollisionDescription& colDesc,
                                            rpVoronoiSimplexSolver& simplexSolver,
                                            GjkDistanceTemplate* distInfo)
{


    bool m_catchDegeneracies  = true;
    scalar m_cachedSeparatingDistance = 0.f;

    scalar distance=scalar(0.);
    Vector3	normalInB(scalar(0.),scalar(0.),scalar(0.));

    Vector3 pointOnA,pointOnB;
    Transform	localTransA = a.getWorldTransform();
    Transform   localTransB = b.getWorldTransform();

   // static_cast<const rpConvexShape*>(a.getCollisionShape());

    scalar marginA = 0.00;//a.getCollisionShape().getMargin();
    scalar marginB = 0.00;//b.getCollisionShape().getMargin();

    int m_curIter = 0;
    int gGjkMaxIter = colDesc.m_maxGjkIterations;//this is to catch invalid input, perhaps check for #NaN?
    Vector3 m_cachedSeparatingAxis = colDesc.m_firstDir;

    bool isValid = false;
    bool checkSimplex = false;
    bool checkPenetration = true;
    int m_degenerateSimplex = 0;

    int m_lastUsedMethod = -1;

    {
        scalar squaredDistance = BT_LARGE_FLOAT;
        scalar delta = scalar(0.);

        scalar margin = marginA + marginB;



        simplexSolver.reset();

        for ( ; ; )
            //while (true)
        {

            Vector3 seperatingAxisInA =  (localTransA.getOrientation().getInverse() * -m_cachedSeparatingAxis);
            Vector3 seperatingAxisInB =  (localTransB.getOrientation().getInverse() *  m_cachedSeparatingAxis);

            Vector3 pInA = a.getLocalSupportPointWithMargin(seperatingAxisInA);
            Vector3 qInB = b.getLocalSupportPointWithMargin(seperatingAxisInB);

            Vector3  pWorld = localTransA * (pInA);
            Vector3  qWorld = localTransB * (qInB);



            Vector3 w	= pWorld - qWorld;
            delta = m_cachedSeparatingAxis.dot(w);

            // potential exit, they don't overlap
            if ((delta > scalar(0.0)) && (delta * delta > squaredDistance * colDesc.m_maximumDistanceSquared))
            {
                m_degenerateSimplex = 10;
                checkSimplex=true;
                //checkPenetration = false;
                break;
            }

            //exit 0: the new point is already in the simplex, or we didn't come any closer
            if (simplexSolver.inSimplex(w))
            {
                m_degenerateSimplex = 1;
                checkSimplex = true;
                break;
            }
            // are we getting any closer ?
            scalar f0 = squaredDistance - delta;
            scalar f1 = squaredDistance * colDesc.m_gjkRelError2;

            if (f0 <= f1)
            {
                if (f0 <= scalar(0.))
                {
                    m_degenerateSimplex = 2;
                } else
                {
                    m_degenerateSimplex = 11;
                }
                checkSimplex = true;
                break;
            }

            //add current vertex to simplex
            simplexSolver.addVertex(w, pWorld, qWorld);
            Vector3 newCachedSeparatingAxis;

            //calculate the closest point to the origin (update vector v)
            if (!simplexSolver.closest(newCachedSeparatingAxis))
            {
                m_degenerateSimplex = 3;
                checkSimplex = true;
                break;
            }

            if(newCachedSeparatingAxis.length2()<colDesc.m_gjkRelError2)
            {
                m_cachedSeparatingAxis = newCachedSeparatingAxis;
                m_degenerateSimplex = 6;
                checkSimplex = true;
                break;
            }

            scalar previousSquaredDistance = squaredDistance;
            squaredDistance = newCachedSeparatingAxis.length2();
#if 0
            ///warning: this termination condition leads to some problems in 2d test case see Bullet/Demos/Box2dDemo
            if (squaredDistance>previousSquaredDistance)
            {
                m_degenerateSimplex = 7;
                squaredDistance = previousSquaredDistance;
                checkSimplex = false;
                break;
            }
#endif //


            //redundant m_simplexSolver->compute_points(pointOnA, pointOnB);

            //are we getting any closer ?
            if (previousSquaredDistance - squaredDistance <= SIMD_EPSILON * previousSquaredDistance)
            {
                //				m_simplexSolver->backup_closest(m_cachedSeparatingAxis);
                checkSimplex = true;
                m_degenerateSimplex = 12;

                break;
            }

            m_cachedSeparatingAxis = newCachedSeparatingAxis;

            //degeneracy, this is typically due to invalid/uninitialized worldtransforms for a btCollisionObject
            if (m_curIter++ > gGjkMaxIter)
            {
#if defined(DEBUG) || defined (_DEBUG)

                printf("btGjkPairDetector maxIter exceeded:%i\n",m_curIter);
                printf("sepAxis=(%f,%f,%f), squaredDistance = %f\n",
                       m_cachedSeparatingAxis.getX(),
                       m_cachedSeparatingAxis.getY(),
                       m_cachedSeparatingAxis.getZ(),
                       squaredDistance);
#endif

                break;

            }


            bool check = (!simplexSolver.fullSimplex());
            //bool check = (!m_simplexSolver->fullSimplex() && squaredDistance > SIMD_EPSILON * m_simplexSolver->maxVertex());

            if (!check)
            {
                //do we need this backup_closest here ?
                //				m_simplexSolver->backup_closest(m_cachedSeparatingAxis);
                m_degenerateSimplex = 13;
                break;
            }
        }

        if (checkSimplex)
        {
            simplexSolver.compute_points(pointOnA, pointOnB);
            normalInB = m_cachedSeparatingAxis;

            scalar lenSqr =m_cachedSeparatingAxis.length2();

            //valid normal
            if (lenSqr < 0.0001)
            {
                m_degenerateSimplex = 5;
            }
            if (lenSqr > SIMD_EPSILON*SIMD_EPSILON)
            {
                scalar rlen = scalar(1.) / SquareRoot(lenSqr );
                normalInB *= rlen; //normalize

                scalar s = SquareRoot(squaredDistance);

                assert(s > scalar(0.0));
                pointOnA -= m_cachedSeparatingAxis * (marginA / s);
                pointOnB += m_cachedSeparatingAxis * (marginB / s);
                distance = ((scalar(1.)/rlen) - margin);
                isValid = true;

                m_lastUsedMethod = 1;
            } else
            {
                m_lastUsedMethod = 2;
            }
        }

        bool catchDegeneratePenetrationCase =
        (m_catchDegeneracies &&  m_degenerateSimplex && ((distance+margin) < 0.01));

        //if (checkPenetration && !isValid)
        if (checkPenetration && (!isValid || catchDegeneratePenetrationCase ))
        {
            //penetration case

            //if there is no way to handle penetrations, bail out

            // Penetration depth case.
            Vector3 tmpPointOnA,tmpPointOnB;

            m_cachedSeparatingAxis.setToZero();

            float l=0;
            bool isValid2 = GjkEpaCalcPenDepth(a,b, m_cachedSeparatingAxis, tmpPointOnA, tmpPointOnB , l);

            if (isValid2)
            {
                Vector3 tmpNormalInB = tmpPointOnB-tmpPointOnA;
                scalar lenSqr = tmpNormalInB.length2();
                if (lenSqr <= (SIMD_EPSILON*SIMD_EPSILON))
                {
                    tmpNormalInB = m_cachedSeparatingAxis;
                    lenSqr = m_cachedSeparatingAxis.length2();
                }

                if (lenSqr > (SIMD_EPSILON*SIMD_EPSILON))
                {
                    tmpNormalInB /= SquareRoot(lenSqr);
                    scalar distance2 = -(tmpPointOnA-tmpPointOnB).length();
                    //only replace valid penetrations when the result is deeper (check)
                    if (!isValid || (distance2 < distance))
                    {
                        distance = distance2;
                        pointOnA = tmpPointOnA;
                        pointOnB = tmpPointOnB;
                        normalInB = tmpNormalInB;

                        isValid = true;
                        m_lastUsedMethod = 3;
                    } else
                    {
                        m_lastUsedMethod = 8;
                    }
                } else
                {
                    m_lastUsedMethod = 9;
                }
            } else

            {
                ///this is another degenerate case, where the initial GJK calculation reports a degenerate case
                ///EPA reports no penetration, and the second GJK (using the supporting vector without margin)
                ///reports a valid positive distance. Use the results of the second GJK instead of failing.
                ///thanks to Jacob.Langford for the reproduction case
                ///http://code.google.com/p/bullet/issues/detail?id=250
                if (m_cachedSeparatingAxis.length2() > scalar(0.))
                {
                    scalar distance2 = (tmpPointOnA-tmpPointOnB).length()-margin;
                    //only replace valid distances when the distance is less
                    if (!isValid || (distance2 < distance))
                    {
                        distance = distance2;
                        pointOnA = tmpPointOnA;
                        pointOnB = tmpPointOnB;
                        pointOnA -= m_cachedSeparatingAxis * marginA ;
                        pointOnB += m_cachedSeparatingAxis * marginB ;
                        normalInB = m_cachedSeparatingAxis;
                        normalInB.normalize();

                        isValid = true;
                        m_lastUsedMethod = 6;
                    } else
                    {
                        m_lastUsedMethod = 5;
                    }
                }
            }
        }
    }




    if (isValid && ((distance < 0) || (distance*distance < colDesc.m_maximumDistanceSquared)))
    {

        m_cachedSeparatingAxis = normalInB;
        m_cachedSeparatingDistance = distance;
        distInfo->m_penetrationDepth  = distance;
        distInfo->m_normal  = normalInB;
        distInfo->pBLocal   = pointOnB;
        distInfo->pALocal   = pointOnB+normalInB*distance;
        return 0;
    }
    return -m_lastUsedMethod;
}


//-------------------------------------------------------------------------------------------------------//




} /* namespace real_physics */

#endif /* SOURCE_ENGIE_COLLISION_NARROWPHASE_RPCOMPUTEGJKEPAPENETRATION_H_ */
