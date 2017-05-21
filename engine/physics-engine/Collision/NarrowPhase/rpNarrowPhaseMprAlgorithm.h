#ifndef RPNARROWPHASEGJKMPRALGORITHM_H
#define RPNARROWPHASEGJKMPRALGORITHM_H


#include "rpNarrowPhaseCollisionAlgorithm.h"

namespace real_physics
{


class rpNarrowPhaseMprAlgorithm : public rpNarrowPhaseCollisionAlgorithm
{

private:

    /// Private copy-constructor
    rpNarrowPhaseMprAlgorithm(const rpNarrowPhaseMprAlgorithm& algorithm);

    /// Private assignment operator
    rpNarrowPhaseMprAlgorithm& operator=(const rpNarrowPhaseMprAlgorithm& algorithm);


public:

     rpNarrowPhaseMprAlgorithm();
    ~rpNarrowPhaseMprAlgorithm();


    /// Compute a contact info if the two bounding volume collide
    virtual bool testCollision(const rpCollisionShapeInfo &shape1Info,
                               const rpCollisionShapeInfo &shape2Info,
                               OutContactInfo& outInfo);


};


}

#endif // RPNARROWPHASEGJKMPRALGORITHM_H
