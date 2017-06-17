#include "UltimateJoint.h"

namespace utility_engine
{

UltimateJoint::UltimateJoint(real_physics::rpJoint *_joint)
 : mJoint(_joint)
{
  assert(mJoint);
}


real_physics::rpJoint *UltimateJoint::getJoint() const
{
    return mJoint;
}

bool UltimateJoint::isCollisionEnabled() const
{
    assert(mJoint);
    return mJoint->isCollisionEnabled();
}

void UltimateJoint::setIsCollisionEnabled(bool isCollisionEnabled)
{
    assert(mJoint);
    mJoint->setIsCollisionEnabled(isCollisionEnabled);
}




}
