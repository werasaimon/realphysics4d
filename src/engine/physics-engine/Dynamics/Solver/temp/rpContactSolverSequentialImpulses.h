#ifndef RPCONTACTSOLVERSEQUENTIALIMPULSES_H
#define RPCONTACTSOLVERSEQUENTIALIMPULSES_H

#include  <set>
#include  <map>

#include "rpSequentialImpulseObjectSolver.h"
#include "../rpIsland.h"

namespace real_physics
{

class rpContactSolverSequentialImpulses
{

   protected:


    // array map contacts solver
    std::map< overlappingpairid , rpContactSolver* > mContactSolvers;


    void addChekCollisionPair( overlappingpairid keyPair, rpContactManifold* maniflod );


   public:

      rpContactSolverSequentialImpulses();



      ///initilize solver LCP
      void  initializeForIsland( scalar dt  ,  rpIsland* _island);

      //void  initializeContactConstraints();

      /// Warm starting the in solver.
      void warmStart();

      /// solver velocity
      void solveVelocityConstraint();

      /// solver position
      void solvePositionConstraint();


      /// Store the computed impulses to use them to
      /// warm start the solver at the next iteration
      void storeImpulses();


      /// Clean up the constraint solver
      void cleanup();
};

}
#endif // RPCONTACTSOLVERSEQUENTIALIMPULSES_H
