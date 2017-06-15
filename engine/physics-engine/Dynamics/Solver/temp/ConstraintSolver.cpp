#include "ConstraintSolver.h"


namespace real_physics
{


ConstraintSolver::ConstraintSolver()
 : mIsWarmStartingActive(true)
{

}

ConstraintSolver::~ConstraintSolver()
{

}

void ConstraintSolver::initializeForIsland(scalar dt, rpIsland *island)
{
    assert(island != NULL);
    assert(island->getNbBodies() > 0);
    assert(island->getNbJoints() > 0);

    // Set the current time step
    mTimeStep = dt;

    // Initialize the constraint solver data used to initialize and solve the constraints
    mConstraintSolverData.timeStep = mTimeStep;
    mConstraintSolverData.isWarmStartingActive = mIsWarmStartingActive;

    // For each joint of the island
    rpJoint** joints = island->getJoints();
    for (uint i=0; i<island->getNbJoints(); i++)
    {

        // Initialize the constraint before solving it
        joints[i]->initBeforeSolve(dt);

        // Warm-start the constraint if warm-starting is enabled
        if (mIsWarmStartingActive)
        {
            joints[i]->warmstart();
        }
    }

}

void ConstraintSolver::solveVelocityConstraints(rpIsland *island)
{

    assert(island != NULL);
    assert(island->getNbJoints() > 0);

    // For each joint of the island
    rpJoint** joints = island->getJoints();
    for (uint i=0; i<island->getNbJoints(); i++)
    {
        // Solve the constraint
        joints[i]->solveVelocityConstraint();
    }

}

void ConstraintSolver::solvePositionConstraints(rpIsland *island)
{
      assert(island != NULL);
      assert(island->getNbJoints() > 0);

      // For each joint of the island
      rpJoint** joints = island->getJoints();
      for (uint i=0; i < island->getNbJoints(); i++)
      {

          // Solve the constraint
          joints[i]->solvePositionConstraint();
      }
}


}
