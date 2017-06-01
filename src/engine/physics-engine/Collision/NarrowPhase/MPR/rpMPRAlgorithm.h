#ifndef RPMPR_H
#define RPMPR_H

#include "../../rpCollisionShapeInfo.h"
#include "../rpNarrowPhaseCollisionAlgorithm.h"

namespace real_physics
{
    class rpMPRAlgorithm
    {

    private:

       void supportTransformed(const rpCollisionShapeInfo *s, const Vector3 &dir, Vector3 &result) const;

    public:

       bool ComputeMprColliderPenetration( const rpCollisionShapeInfo *s1, const rpCollisionShapeInfo *s2, OutContactInfo& out ) const;



    };

}

#endif // RPMPR_H
