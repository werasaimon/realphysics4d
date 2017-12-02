#include "rpMPRAlgorithm.h"



namespace real_physics
{

/* MPR	*/
#define MPR_MAX_VERTICES	64
#define MPR_MAX_ITERATIONS	255
#define MPR_EPS		        0.00001




SIMD_INLINE void rpMPRAlgorithm::supportTransformed(const rpCollisionShapeInfo *s, const Vector3 &dir, Vector3 &result) const
{
    const Transform &t = s->getWorldTransform();
    result =  t * s->getLocalSupportPointWithMargin( t.getBasis().getInverse() * dir );
}


bool rpMPRAlgorithm::ComputeMprColliderPenetration( const rpCollisionShapeInfo *s1, const rpCollisionShapeInfo *s2, OutContactInfo& out ) const
{

    // Used variables
    Vector3 temp1;
    Vector3 v01, v02, v0;
    Vector3 v11, v12, v1;
    Vector3 v21, v22, v2;
    Vector3 v31, v32, v3;
    Vector3 v41, v42, v4;

    // Initialization of the output
    Vector3 &point       = out.pALocal;
    Vector3 &normal      = out.m_normal;
    scalar  &penetration = out.m_penetrationDepth;

    // Get the center of shape1 in world coordinates
    v01 = s1->getWorldTransform().getPosition();

    // Get the center of shape2 in world coordinates
    v02 = s2->getWorldTransform().getPosition();

    // v0 is the center of the Minkowski difference
    v0 = v02 - v01;

    // Avoid case where centers overlap - any direction is fine in this case
    if (v0.isZero())
        v0 = Vector3(MPR_EPS, 0.0f, 0.0f);

    // v1 = support in direction of origin
    normal = -v0;

    supportTransformed(s1,  v0, v11);
    supportTransformed(s2, -v0, v12);
    v1 = v12 - v11;

    if ( v1.dot(normal) <= 0.0f)
        return false;

    // v2 = support perpendicular to v1,v0
    normal = v1.cross(v0);

    if (normal.isZero())
    {
        normal = v1 - v0;
        normal.normalize();

        point = v11;
        point += v12;
        point *= 0.5f;

        penetration = (v12 - v11).dot(normal);

        return true;
    }

    supportTransformed(s1, -normal, v21);
    supportTransformed(s2,  normal, v22);
    v2 = v22 - v21;

    if ( v2.dot(normal) <= 0.0f)  return false;

    // Determine whether origin is on + or - side of plane (v1,v0,v2)
    normal = (v1 - v0).cross(v2 - v0);
    normal.normalize();

    scalar dist = normal.dot(v0);

    // If the origin is on the - side of the plane, reverse the direction of the plane
    if (dist > 0.0f)
    {
        Swap(v1, v2);
        Swap(v11, v21);
        Swap(v12, v22);
        normal = -normal;
    }

    int phase2 = 0;
    int phase1 = 0;
    bool hit = false;

    // Phase One: Identify a portal
    while ( phase1 < MPR_MAX_ITERATIONS )
    {
        phase1++;

        // Obtain the support point in a direction perpendicular to the existing plane
        // Note: This point is guaranteed to lie off the plane
        supportTransformed(s1, -normal, v31);
        supportTransformed(s2,  normal, v32);
        v3 = v32 - v31;

        if (v3.dot(normal) <= 0.0f)
            return false;

        // If origin is outside (v1,v0,v3), then eliminate v2 and loop
        temp1 = v1.cross(v3);
        if (temp1.dot(v0) < 0.0f)
        {
            v2 = v3;
            v21 = v31;
            v22 = v32;
            normal = (v1 - v0).cross(v3 - v0);
            continue;
        }

        // If origin is outside (v3,v0,v2), then eliminate v1 and loop
        temp1 = v3.cross(v2);
        if (temp1.dot(v0) < 0.0f)
        {
            v1 = v3;
            v11 = v31;
            v12 = v32;
            normal = (v3 - v0).cross(v2 - v0);
            continue;
        }

        // Phase Two: Refine the portal
        // We are now inside of a wedge...
        while (true)
        {
            phase2++;

            // Compute normal of the wedge face
            normal =  (v2 - v1).cross(v3 - v1);
            normal.normalize();

            // Can this happen? Can it be handled more cleanly?
            if (normal.isZero())
                return true;

            normal.normalize();

            // Compute distance from origin to wedge face
            scalar d = normal.dot(v1);

            // If the origin is inside the wedge, we have a hit
            if (d >= 0) hit = true;

            // Find the support point in the direction of the wedge face
            supportTransformed(s1, -normal, v41);
            supportTransformed(s2, normal, v42);
            v4 = v42 - v41;

            scalar delta = (v4 - v3).dot(normal);
            penetration =   v4.dot(normal);

            // If the boundary is thin enough or the origin is outside
            // the support plane for the newly discovered vertex, then we can terminate
            if (delta <= MPR_EPS || penetration <= 0.0f || phase2 > MPR_MAX_ITERATIONS)
            {

                if (hit)
                {
                    scalar b0 = v1.cross(v2).dot(v3);
                    scalar b1 = v3.cross(v2).dot(v0);
                    scalar b2 = v0.cross(v1).dot(v3);
                    scalar b3 = v2.cross(v1).dot(v0);

                    scalar sum = b0 + b1 + b2 + b3;

                    if (sum <= 0)
                    {
                        b0 = 0;
                        b1 = v2.cross(v3).dot(normal);
                        b2 = v3.cross(v1).dot(normal);
                        b3 = v1.cross(v2).dot(normal);

                        sum = b1 + b2 + b3;
                    }

                    scalar inv = 1.0f / sum;

                    point =  v01 * b0;
                    point += v11 * b1;
                    point += v21 * b2;
                    point += v31 * b3;

                    point += v02 * b0;
                    point += v12 * b1;
                    point += v22 * b2;
                    point += v32 * b3;

                    point *= inv * 0.5f;
                }

                return hit;
            }

            // Compute the tetrahedron dividing face (v4,v0,v3)
            temp1 = v4.cross(v0);
            temp1.normalize();
            scalar d2 = temp1.dot(v1);

            if (d2 >= 0.0f)
            {
                d2 = temp1.dot(v2);

                if (d2 >= 0.0f)
                {
                    // Inside d1 & inside d2 ==> eliminate v1
                    v1 = v4;
                    v11 = v41;
                    v12 = v42;
                }
                else
                {
                    // Inside d1 & outside d2 ==> eliminate v3
                    v3 = v4;
                    v31 = v41;
                    v32 = v42;
                }
            }
            else
            {
                d2 = temp1.dot(v3);

                if (d2 >= 0.0f)
                {
                    // Outside d1 & inside d3 ==> eliminate v2
                    v2 = v4;
                    v21 = v41;
                    v22 = v42;
                }
                else
                {
                    // Outside d1 & outside d3 ==> eliminate v1
                    v1 = v4;
                    v11 = v41;
                    v12 = v42;
                }
            }
        }
    }

    //	    // Should never get here
    return false;
}




#undef MPR_MAX_VERTICES
#undef MPR_MAX_ITERATIONS
#undef MPR_INSIDE_EPS


}
