#ifndef RPNARROWPHASEGJKMPRALGORITHM_H
#define RPNARROWPHASEGJKMPRALGORITHM_H


#include "rpNarrowPhaseCollisionAlgorithm.h"

namespace real_physics
{


class rpNarrowPhaseGjkMprAlgorithm : public rpNarrowPhaseCollisionAlgorithm
{

private:

    /// Private copy-constructor
    rpNarrowPhaseGjkMprAlgorithm(const rpNarrowPhaseGjkMprAlgorithm& algorithm);

    /// Private assignment operator
    rpNarrowPhaseGjkMprAlgorithm& operator=(const rpNarrowPhaseGjkMprAlgorithm& algorithm);


public:

    rpNarrowPhaseGjkMprAlgorithm();
    ~rpNarrowPhaseGjkMprAlgorithm();


    /// Compute a contact info if the two bounding volume collide
    virtual bool testCollision(const rpCollisionShapeInfo &shape1Info,
                               const rpCollisionShapeInfo &shape2Info,
                               OutContactInfo& outInfo);


};


}

#endif // RPNARROWPHASEGJKMPRALGORITHM_H
