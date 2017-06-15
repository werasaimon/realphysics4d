#include "rpContactSolverSequentialImpulses.h"
#include "../../Body/rpRigidPhysicsBody.h"


namespace real_physics
{




rpContactSolverSequentialImpulses::rpContactSolverSequentialImpulses()
{

}

void rpContactSolverSequentialImpulses::initializeForIsland(scalar dt, rpIsland *_island)
{

    for( auto pair : mContactSolvers )
    {
        pair.second->isFakeCollid = false;
    }


    int NbContactManifolds = _island->getNbContactManifolds();
    // For each contact manifold of the island
    rpContactManifold** contactManifolds = _island->getContactManifold();
    for (uint i=0; i < NbContactManifolds; i++)
    {
        rpContactManifold* maniflod = contactManifolds[i];
        overlappingpairid pairId = rpOverlappingPair::computeID( maniflod->mShape1 , maniflod->mShape2 );
        addChekCollisionPair(pairId, maniflod);
    }


    /// delete overlapping pairs collision
    for( auto pair : mContactSolvers )
    {
        if( !pair.second->isFakeCollid )
        {
            delete pair.second;
            mContactSolvers.erase(pair.first);
        }
    }


    for( auto pair : mContactSolvers )
    {
        pair.second->initializeForIsland(dt);
    }
}

void rpContactSolverSequentialImpulses::warmStart()
{

    for( auto pair : mContactSolvers )
    {
        pair.second->warmStart();
    }

}

void rpContactSolverSequentialImpulses::solveVelocityConstraint()
{
    for( auto pair : mContactSolvers )
    {
        pair.second->solveVelocityConstraint();
    }
}

void rpContactSolverSequentialImpulses::solvePositionConstraint()
{
    for( auto pair : mContactSolvers )
    {
        pair.second->solvePositionConstraint();
    }
}



void rpContactSolverSequentialImpulses::addChekCollisionPair(overlappingpairid keyPair, rpContactManifold *maniflod)
{
    if(mContactSolvers.find(keyPair) == mContactSolvers.end())
    {

        rpRigidPhysicsBody *body1 = static_cast<rpRigidPhysicsBody*>(maniflod->mShape2->getBody());
        rpRigidPhysicsBody *body2 = static_cast<rpRigidPhysicsBody*>(maniflod->mShape1->getBody());


        rpSequentialImpulseObjectSolver *solverObject = new rpSequentialImpulseObjectSolver( body1 , body2 );

        mContactSolvers.insert( std::make_pair(keyPair, solverObject) );
    }

    mContactSolvers.find(keyPair)->second->initManiflod(maniflod);
    mContactSolvers.find(keyPair)->second->isFakeCollid = true;

}


}
