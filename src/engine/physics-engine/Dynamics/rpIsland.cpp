#include "rpIsland.h"

namespace real_physics
{


rpIsland::rpIsland(uint nbMaxBodies, uint nbMaxContactManifolds, std::map< overlappingpairid , rpContactSolver* > &_ContactSolvers)
    : mBodies(NULL),
      mContactManifolds(NULL),
      mNbBodies(0),
      mNbContactManifolds(0),
      mContactSolvers(_ContactSolvers)
{

     mBodies                = new rpRigidPhysicsBody*[nbMaxBodies];
     mContactManifolds      = new rpContactManifold*[nbMaxContactManifolds];
     mContactMapIndexesPair = new overlappingpairid[nbMaxContactManifolds];

}

rpIsland::~rpIsland()
{
    delete[] mBodies;
    delete[] mContactManifolds;
    delete   mContactMapIndexesPair;
}


}
