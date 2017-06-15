#include "rpIsland.h"

namespace real_physics
{


rpIsland::rpIsland(uint nbMaxBodies, uint nbMaxContactManifolds, uint nbMaxJoints)
    : mBodies(NULL),
      mContactManifolds(NULL),
      mJoints(NULL),
      mNbBodies(0),
      mNbContactManifolds(0),
      mNbJoints(0)
{

     mBodies           = new rpRigidPhysicsBody*[nbMaxBodies];
     mContactManifolds = new rpContactManifold*[nbMaxContactManifolds];
     mJoints           = new rpJoint*[nbMaxBodies];


}

rpIsland::~rpIsland()
{
    delete[] mBodies;
    delete[] mContactManifolds;
    delete[] mJoints;
}


}
