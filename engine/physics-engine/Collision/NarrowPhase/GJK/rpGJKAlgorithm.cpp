/*
 * rpGJKAlgorithm.cpp
 *
 *  Created on: 19 нояб. 2016 г.
 *      Author: wera
 */


// Libraries
#include "rpGJKAlgorithm.h"

#include <cassert>

#include "../../../LinearMaths/mathematics.h"
#include "../../../LinearMaths/rpLinearMtah.h"
#include "../../../LinearMaths/rpMatrix3x3.h"
#include "../../../LinearMaths/rpQuaternion.h"
#include "../../../LinearMaths/rpVector3D.h"
#include "../../rpCollisionShapeInfo.h"
#include "../../rpProxyShape.h"
#include "../../rpRaycastInfo.h"
#include "../GJK_EPA/rpGjkCollisionDescription.h"
#include "../GJK_EPA/rpComputeGjkEpaPenetration.h"
#include "rpSimplex.h"

using namespace std;

namespace real_physics
{


// Constructor
GJKAlgorithm::GJKAlgorithm()
{
}

// Destructor
GJKAlgorithm::~GJKAlgorithm()
{
}

// Compute a contact info if the two collision shapes collide.
/// This method implements the Hybrid Technique for computing the penetration depth by
/// running the GJK algorithm on original objects (without margin). If the shapes intersect
/// only in the margins, the method compute the penetration depth and contact points
/// (of enlarged objects). If the original objects (without margin) intersect, we
/// call the computePenetrationDepthForEnlargedObjects() method that run the GJK
/// algorithm on the enlarged object to obtain a simplex polytope that contains the
/// origin, they we give that simplex polytope to the EPA algorithm which will compute
/// the correct penetration depth and contact points between the enlarged objects.
bool GJKAlgorithm::testCollision(const CollisionShapeInfo& shape1Info,
                                 const CollisionShapeInfo& shape2Info ,
								 OutContactInfo& _outInfo)
{


		 //  PROFILE("GJKAlgorithm::testCollision()");

		    Vector3 suppA;             // Support point of object A
		    Vector3 suppB;             // Support point of object B
		    Vector3 w;                 // Support point of Minkowski difference A-B
		    Vector3 pA;                // Closest point of object A
		    Vector3 pB;                // Closest point of object B
		    scalar vDotw;
		    scalar prevDistSquare;

		    assert(shape1Info.collisionShape->isConvex());
		    assert(shape2Info.collisionShape->isConvex());

		    const rpConvexShape* shape1 = static_cast<const rpConvexShape*>(shape1Info.collisionShape);
		    const rpConvexShape* shape2 = static_cast<const rpConvexShape*>(shape2Info.collisionShape);

		    void** shape1CachedCollisionData;// = shape1Info.cachedCollisionData;
		    void** shape2CachedCollisionData;// = shape2Info.cachedCollisionData;

		    // Get the local-space to world-space transforms
		    const Transform transform1 = shape1Info.shapeToWorldTransform;
		    const Transform transform2 = shape2Info.shapeToWorldTransform;



		    // Initialize the margin (sum of margins of both objects)
		    scalar margin = 0.01;//shape1->getMargin() + shape2->getMargin();
		    scalar marginSquare = 0.01;//margin * margin;
		    assert(margin > 0.0);



		    //--------------------------------------------------------------------------//


			MinkowskiDifferens<const rpConvexShape> Minkowski;

			Minkowski.mConvexShape1 = shape1;
			Minkowski.mConvexShape2 = shape2;

			Minkowski.shape1CachedCollisionData = shape1CachedCollisionData;
			Minkowski.shape2CachedCollisionData = shape2CachedCollisionData;

			Minkowski.m_toshape0 = transform1;//.getInverse() * transform2;
			Minkowski.m_toshape1 = transform2;//.getOrientation().getMatrix().getTranspose() *
		                           //transform1.getOrientation().getMatrix();




		    //--------------------------------------------------------------------------//

		    // Create a simplex set
		    Simplex simplex;

		    // Get the previous point V (last cached separating axis)
		    Vector3 v = transform2.getPosition() - transform1.getPosition();


		    // Initialize the upper bound for the square distance
		    scalar distSquare = DECIMAL_LARGEST;

		    do
		    {
		        // Compute the support points for original objects (without margins) A and B
		        suppA = Minkowski.Support0(-v);
		        suppB = Minkowski.Support1( v);

		        // Compute the support point for the Minkowski difference A-B
		        w = suppA - suppB;

		        vDotw = v.dot(w);

		        // If the enlarge objects (with margins) do not intersect
		        if (vDotw > 0.0 && vDotw * vDotw > distSquare * marginSquare)
		        {

		            // Cache the current separating axis for frame coherence
		          //  mCurrentOverlappingPair->setCachedSeparatingAxis(v);

		        	this->setCachedSeparatingAxis(v=w);

		            // No intersection, we return
		        	return false;
		        }

		        // If the objects intersect only in the margins
		        if (simplex.isPointInSimplex(w) || distSquare - vDotw <= distSquare * REL_ERROR_SQUARE)
		        {

		            // Compute the closet points of both objects (without the margins)
		            simplex.computeClosestPointsOfAandB(pA, pB);

		            // Project those two points on the margins to have the closest points of both
		            // object with the margins
		            scalar dist = SquareRoot(distSquare);
		            assert(dist > 0.0);
		            pA = (pA - (shape1->getMargin() / dist) * v);
		            pB = Minkowski.m_toshape0.getInverse() * (pB + (shape2->getMargin() / dist) * v);

		            // Compute the contact info
		            Vector3 normal = /*transform1.getOrientation() */ (-v.getUnit());
		            scalar penetrationDepth = margin - dist;

					// Reject the contact if the penetration depth is negative (due too numerical errors)
		            if (penetrationDepth <= 0.0) return false;


		            // Create the contact info object
		            _outInfo =	OutContactInfo( normal, penetrationDepth, pA, pB );


		            // There is an intersection, therefore we return
		            return true;
		        }

		        // Add the new support point to the simplex
		        simplex.addPoint(w, suppA, suppB);

		        // If the simplex is affinely dependent
		        if (simplex.isAffinelyDependent())
		        {

		            // Compute the closet points of both objects (without the margins)
		            simplex.computeClosestPointsOfAandB(pA, pB);

		            // Project those two points on the margins to have the closest points of both
		            // object with the margins
		            scalar dist = SquareRoot(distSquare);
		            assert(dist > 0.0);
		            pA = (pA - (shape1->getMargin() / dist) * v);
		            pB = Minkowski.m_toshape0.getInverse() * (pB + (shape2->getMargin() / dist) * v);

		            // Compute the contact info
		            Vector3 normal = /*transform1.getOrientation() */ (-v.getUnit());
		            scalar penetrationDepth = margin - dist;

					// Reject the contact if the penetration depth is negative (due too numerical errors)
		            if (penetrationDepth <= 0.0) return false;



		            // Create the contact info object
		            _outInfo =	OutContactInfo( normal, penetrationDepth, pA, pB );


		            // There is an intersection, therefore we return
		            return true;
		        }

		        // Compute the point of the simplex closest to the origin
		        // If the computation of the closest point fail
		        if (!simplex.computeClosestPoint(v))
		        {

		            // Compute the closet points of both objects (without the margins)
		            simplex.computeClosestPointsOfAandB(pA, pB);

		            // Project those two points on the margins to have the closest points of both
		            // object with the margins
		            scalar dist = SquareRoot(distSquare);
		            assert(dist > 0.0);
		            pA = (pA - (shape1->getMargin() / dist) * v);
		            pB = Minkowski.m_toshape0.getInverse() * (pB + (shape2->getMargin() / dist) * v);

		            // Compute the contact info
		            Vector3 normal = /*transform1.getOrientation() */ (-v.getUnit());
		            scalar penetrationDepth = margin - dist;

					// Reject the contact if the penetration depth is negative (due too numerical errors)
		            if (penetrationDepth <= 0.0) return false;


		            // Create the contact info object
		            _outInfo =	OutContactInfo( normal, penetrationDepth, pA, pB );

		            // There is an intersection, therefore we return
		            return true;
		        }

		        // Store and update the squared distance of the closest point
		        prevDistSquare = distSquare;
		        distSquare = v.lengthSquare();

		        // If the distance to the closest point doesn't improve a lot
		        if (prevDistSquare - distSquare <= MACHINE_EPSILON * prevDistSquare)
		        {
		            simplex.backupClosestPointInSimplex(v);

		            // Get the new squared distance
		            distSquare = v.lengthSquare();

		            // Compute the closet points of both objects (without the margins)
		            simplex.computeClosestPointsOfAandB(pA, pB);

		            // Project those two points on the margins to have the closest points of both
		            // object with the margins
		            scalar dist = SquareRoot(distSquare);
		            assert(dist > 0.0);
		            pA = (pA - (shape1->getMargin() / dist) * v);
		            pB = Minkowski.m_toshape0.getInverse() * (pB + (shape2->getMargin() / dist) * v);

		            // Compute the contact info
		            Vector3 normal = /*transform1.getOrientation() */ (-v.getUnit());
		            scalar penetrationDepth = margin - dist;

					// Reject the contact if the penetration depth is negative (due too numerical errors)
		            if (penetrationDepth <= 0.0) return false;



		            //------------------------------------------------------------//
		            _outInfo =	OutContactInfo( normal, penetrationDepth, pA, pB );

		            // There is an intersection, therefore we return
		            return true;
		        }
		    } while(!simplex.isFull() && distSquare > MACHINE_EPSILON * simplex.getMaxLengthSquareOfAPoint());

		    // The objects (without margins) intersect. Therefore, we run the GJK algorithm
		    // again but on the enlarged objects to compute a simplex polytope that contains
		    // the origin. Then, we give that simplex polytope to the EPA algorithm to compute
		    // the correct penetration depth and contact points between the enlarged objects.
		    return computePenetrationDepthForEnlargedObjects(shape1Info, transform1,
		    		                                         shape2Info, transform2,
															 Minkowski ,
															 v , _outInfo);


}

/// This method runs the GJK algorithm on the two enlarged objects (with margin)
/// to compute a simplex polytope that contains the origin. The two objects are
/// assumed to intersect in the original objects (without margin). Therefore such
/// a polytope must exist. Then, we give that polytope to the EPA algorithm to
/// compute the correct penetration depth and contact points of the enlarged objects.
bool GJKAlgorithm::computePenetrationDepthForEnlargedObjects(const CollisionShapeInfo& shape1Info, const Transform& transform1,
                                                             const CollisionShapeInfo& shape2Info, const Transform& transform2,
															 const MinkowskiDifferens<const rpConvexShape> Minkowski ,
                                                             Vector3& v ,
															 OutContactInfo& _outInfo)
{
    //PROFILE("GJKAlgorithm::computePenetrationDepthForEnlargedObjects()");

    Simplex simplex;
    Vector3 suppA;
    Vector3 suppB;
    Vector3 w;
    scalar vDotw;
    scalar distSquare = DECIMAL_LARGEST;
    scalar prevDistSquare;

    assert(shape1Info.collisionShape->isConvex());
    assert(shape2Info.collisionShape->isConvex());

    const rpConvexShape* shape1 = static_cast<const rpConvexShape*>(shape1Info.collisionShape);
    const rpConvexShape* shape2 = static_cast<const rpConvexShape*>(shape2Info.collisionShape);

    void** shape1CachedCollisionData;// = shape1Info.cachedCollisionData;
    void** shape2CachedCollisionData;// = shape2Info.cachedCollisionData;


    do
    {
        // Compute the support points for the enlarged object A and B
        suppA = Minkowski.Support0(-v);
        suppB = Minkowski.Support1( v);

        // Compute the support point for the Minkowski difference A-B
        w = suppA - suppB;

        vDotw = v.dot(w);

        // If the enlarge objects do not intersect
        if (vDotw > 0.0)
        {

            // No intersection, we return
        	return false;
        }

        // Add the new support point to the simplex
        simplex.addPoint(w, suppA, suppB);

        if (simplex.isAffinelyDependent())
        {
        	return false;
        }

        if (!simplex.computeClosestPoint(v))
        {
        	return false;
        }

        // Store and update the square distance
        prevDistSquare = distSquare;
        distSquare = v.lengthSquare();

        if (prevDistSquare - distSquare <= MACHINE_EPSILON * prevDistSquare)
        {
        	return false;
        }

    } while(!simplex.isFull() && distSquare > MACHINE_EPSILON * simplex.getMaxLengthSquareOfAPoint());



    // Give the simplex computed with GJK algorithm to the EPA algorithm
    // which will compute the correct penetration depth and contact points
    // between the two enlarged objects
    return mAlgoEPA.computePenetrationDepthAndContactPoints(simplex,
    		                                                shape1Info, transform1,
															shape2Info, transform2,
															Minkowski ,
                                                            v, _outInfo);
}

// Use the GJK Algorithm to find if a point is inside a convex collision shape
bool GJKAlgorithm::testPointInside(const Vector3& localPoint, rpProxyShape* proxyShape)
{

    Vector3 suppA;             // Support point of object A
    Vector3 w;                 // Support point of Minkowski difference A-B
    scalar  prevDistSquare;

    assert(proxyShape->getCollisionShape()->isConvex());

    const rpConvexShape* shape = static_cast<const rpConvexShape*>(proxyShape->getCollisionShape());

    void** shapeCachedCollisionData = proxyShape->getCachedCollisionData();

    // Support point of object B (object B is a single point)
    const Vector3 suppB(localPoint);

    // Create a simplex set
    Simplex simplex;

    // Initial supporting direction
    Vector3 v(1, 1, 1);

    // Initialize the upper bound for the square distance
    scalar distSquare = DECIMAL_LARGEST;

    do
    {

        // Compute the support points for original objects (without margins) A and B
        suppA = shape->getLocalSupportPointWithoutMargin(-v, shapeCachedCollisionData);

        // Compute the support point for the Minkowski difference A-B
        w = suppA - suppB;

        // Add the new support point to the simplex
        simplex.addPoint(w, suppA, suppB);

        // If the simplex is affinely dependent
        if (simplex.isAffinelyDependent())
        {

        	return false;
        }

        // Compute the point of the simplex closest to the origin
        // If the computation of the closest point fail
        if (!simplex.computeClosestPoint(v))
        {

            return false;
        }

        // Store and update the squared distance of the closest point
        prevDistSquare = distSquare;
        distSquare = v.lengthSquare();

        // If the distance to the closest point doesn't improve a lot
        if (prevDistSquare - distSquare <= MACHINE_EPSILON * prevDistSquare)
        {

            return false;
        }

    } while(!simplex.isFull() && distSquare > MACHINE_EPSILON *
                                 simplex.getMaxLengthSquareOfAPoint());

    // The point is inside the collision shape
    return true;
}


// Ray casting algorithm agains a convex collision shape using the GJK Algorithm
/// This method implements the GJK ray casting algorithm described by Gino Van Den Bergen in
/// "Ray Casting against General Convex Objects with Application to Continuous Collision Detection".
bool GJKAlgorithm::raycast(const Ray& ray, RaycastInfo& raycastInfo ,rpProxyShape* proxyShape )
{

    assert(proxyShape->getCollisionShape()->isConvex());

    const rpConvexShape* shape = static_cast<const rpConvexShape*>(proxyShape->getCollisionShape());

    void** shapeCachedCollisionData = proxyShape->getCachedCollisionData();

    Vector3 suppA;      // Current lower bound point on the ray (starting at ray's origin)
    Vector3 suppB;      // Support point on the collision shape
    const scalar machineEpsilonSquare = MACHINE_EPSILON * MACHINE_EPSILON;
    const scalar epsilon = scalar(0.0001);



    // Convert the ray origin and direction into the local-space of the collision shape
    Vector3 rayDirection = (ray.point2 - ray.point1);

    // If the points of the segment are two close, return no hit
    if (rayDirection.lengthSquare() < machineEpsilonSquare) return false;

    Vector3 w;

    // Create a simplex set
    Simplex simplex;

    Vector3 n(scalar(0.0), scalar(0.0), scalar(0.0));
    scalar lambda = scalar(0.0);
    suppA = ray.point1;    // Current lower bound point on the ray (starting at ray's origin)
    suppB = shape->getLocalSupportPointWithoutMargin( rayDirection, shapeCachedCollisionData );
    Vector3 v = suppA - suppB;
    scalar vDotW, vDotR;
    scalar distSquare = v.lengthSquare();
    int nbIterations = 0;

    // GJK Algorithm loop
    while (distSquare > epsilon && nbIterations < MAX_ITERATIONS_GJK_RAYCAST)
    {

        // Compute the support points
        suppB = shape->getLocalSupportPointWithoutMargin(v, shapeCachedCollisionData);
        w = suppA - suppB;

        vDotW = v.dot(w);

        if (vDotW > scalar(0))
        {

            vDotR = v.dot(rayDirection);

            if (vDotR >= -machineEpsilonSquare)
            {
                return false;
            }
            else
            {

                // We have found a better lower bound for the hit point along the ray
                lambda = lambda - vDotW / vDotR;
                suppA = ray.point1 + lambda * rayDirection;
                w = suppA - suppB;
                n = v;
            }
        }

        // Add the new support point to the simplex
        if (!simplex.isPointInSimplex(w))
        {
            simplex.addPoint(w, suppA, suppB);
        }

        // Compute the closest point
        if (simplex.computeClosestPoint(v))
        {

            distSquare = v.lengthSquare();
        }
        else
        {
            distSquare = scalar(0.0);
        }

        // If the current lower bound distance is larger than the maximum raycasting distance
        if (lambda > ray.maxFraction) return false;

        nbIterations++;
    }

    // If the origin was inside the shape, we return no hit
    if (lambda < MACHINE_EPSILON) return false;

    // Compute the closet points of both objects (without the margins)
    Vector3 pointA;
    Vector3 pointB;
    simplex.computeClosestPointsOfAandB(pointA, pointB);

    // A raycast hit has been found, we fill in the raycast info
    raycastInfo.hitFraction = lambda;
    raycastInfo.worldPoint = pointB;
    raycastInfo.body = proxyShape->getBody();
    raycastInfo.proxyShape = proxyShape;

    if (n.lengthSquare() >= machineEpsilonSquare )
    { // The normal vector is valid
        raycastInfo.worldNormal = n;
    }
    else
    {  // Degenerated normal vector, we return a zero normal vector
        raycastInfo.worldNormal = Vector3(scalar(0), scalar(0), scalar(0));
    }

    return true;
}



} /* namespace real_physics */
