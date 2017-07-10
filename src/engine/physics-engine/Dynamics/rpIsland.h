#ifndef RPISLAND_H
#define RPISLAND_H

#include "../Body/rpPhysicsBody.h"
#include "../Body/rpRigidPhysicsBody.h"
#include "Solver/rpContactSolverSequentialImpulseObject.h"
#include "../Collision/Manifold/rpContactManifold.h"

#include "Joint/rpJoint.h"
#include "Joint/rpBallAndSocketJoint.h"

namespace real_physics
{
    class rpIsland
    {

      private:

         //-------------------- Attributes -----------------//

         /// Array with all the bodies of the island
         rpRigidPhysicsBody** mBodies;

         /// Array with all the contact manifolds between bodies of the island
         rpContactManifold** mContactManifolds;
         overlappingpairid*  mContactMapIndexesPair;

         /// Current number of bodies in the island
         uint mNbBodies;

         /// Current number of contact manifold in the island
         uint mNbContactManifolds;

         /// array map contacts solver
         std::map< overlappingpairid , rpContactSolver* > &mContactSolvers;


         //-------------------- Methods -------------------//

         /// Private assignment operator
         rpIsland& operator=(const rpIsland& island);

         /// Private copy-constructor
         rpIsland(const rpIsland& island);


      public:


        //-------------------- Methods --------------------//

         /// Constructor
          rpIsland(uint nbMaxBodies , uint nbMaxContactManifolds ,
                   std::map<overlappingpairid, rpContactSolver* > &_ContactSolvers );

         /// Destructor
         ~rpIsland();



         /// Add a body into the island
         void addBody(rpRigidPhysicsBody* body);

         /// Add a contact manifold into the island
         void addContactManifold(rpContactManifold* contactManifold);



         /// Return the number of bodies in the island
         uint getNbBodies() const;

         /// Return the number of contact manifolds in the island
         uint getNbContactManifolds() const;



         /// Return a pointer to the array of bodies
         rpRigidPhysicsBody** getBodies();

         /// Return a pointer to the array of contact manifolds
         rpContactManifold** getContactManifold();




         ///-----------------------------------------------------///
         void warmStart( scalar timeStep )
         {
             for( uint i = 0; i < mNbContactManifolds; i++ )
             {
                 mContactSolvers[mContactMapIndexesPair[i]]->initializeForIsland(timeStep);
                 mContactSolvers[mContactMapIndexesPair[i]]->warmStart();
             }
         }

         ///-----------------------------------------------------///
         void solveVelocityConstraint()
         {
             for( uint i = 0; i < mNbContactManifolds; i++ )
             {
                 mContactSolvers[mContactMapIndexesPair[i]]->solveVelocityConstraint();
             }
         }

         ///-----------------------------------------------------///
         void solvePositionConstraint()
         {
             for( uint i = 0; i < mNbContactManifolds; i++ )
             {
                 mContactSolvers[mContactMapIndexesPair[i]]->solvePositionConstraint();
             }
         }


         ///-----------------------------------------------------///
//         void storeImpulses()
//         {
//             for( uint i = 0; i < mNbContactManifolds; i++ )
//             {
//                 mContactSolvers[mContactMapIndexesPair[i]]->storeImpulses();
//             }
//         }

         //-------------------- Friendship --------------------//
         friend class rpDynamicsWorld;

    };


    // Add a body into the island
    SIMD_INLINE void rpIsland::addBody(rpRigidPhysicsBody* body)
    {
        assert(!body->isSleeping());
        mBodies[mNbBodies] = body;
        mNbBodies++;
    }

    // Add a contact manifold into the island
    SIMD_INLINE void rpIsland::addContactManifold(rpContactManifold* contactManifold)
    {
        overlappingpairid ContactKeyPairManifold = rpOverlappingPair::computeID( contactManifold->mShape1 ,
                                                                                 contactManifold->mShape2 );

        mContactMapIndexesPair[mNbContactManifolds] = ContactKeyPairManifold;
        mContactManifolds[mNbContactManifolds] = contactManifold;
        mNbContactManifolds++;
    }


    // Return the number of bodies in the island
    SIMD_INLINE uint rpIsland::getNbBodies() const
    {
        return mNbBodies;
    }

    // Return the number of contact manifolds in the island
    SIMD_INLINE uint rpIsland::getNbContactManifolds() const
    {
        return mNbContactManifolds;
    }


    // Return a pointer to the array of bodies
    SIMD_INLINE rpRigidPhysicsBody** rpIsland::getBodies()
    {
        return mBodies;
    }

    // Return a pointer to the array of contact manifolds
    SIMD_INLINE rpContactManifold** rpIsland::getContactManifold()
    {
        return mContactManifolds;
    }



}

#endif // RPISLAND_H
