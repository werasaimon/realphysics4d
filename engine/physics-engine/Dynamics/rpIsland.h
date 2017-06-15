#ifndef RPISLAND_H
#define RPISLAND_H

#include "../Body/rpPhysicsBody.h"
#include "../Body/rpRigidPhysicsBody.h"
#include "Solver/rpContactSolverSequentialImpulseObject.h"
#include "../Collision/ContactManiflod/rpContactManifold.h"


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

         /// Array with all the joints between bodies of the island
         rpJoint** mJoints;

         /// Current number of bodies in the island
         uint mNbBodies;

         /// Current number of contact manifold in the island
         uint mNbContactManifolds;

         /// Current number of joints in the island
         uint mNbJoints;




         //-------------------- Methods -------------------//

         /// Private assignment operator
         rpIsland& operator=(const rpIsland& island);

         /// Private copy-constructor
         rpIsland(const rpIsland& island);


      public:


        //-------------------- Methods --------------------//

         /// Constructor
          rpIsland(uint nbMaxBodies, uint nbMaxContactManifolds, uint nbMaxJoints);

         /// Destructor
         ~rpIsland();



         /// Add a body into the island
         void addBody(rpRigidPhysicsBody* body);

         /// Add a contact manifold into the island
         void addContactManifold(rpContactManifold* contactManifold);

         /// Add a joint into the island
         void addJoint(rpJoint* joint);



         /// Return the number of bodies in the island
         uint getNbBodies() const;

         /// Return the number of contact manifolds in the island
         uint getNbContactManifolds() const;

         /// Return the number of joints in the island
         uint getNbJoints() const;



         /// Return a pointer to the array of bodies
         rpRigidPhysicsBody** getBodies();

         /// Return a pointer to the array of contact manifolds
         rpContactManifold** getContactManifold();

         /// Return a pointer to the array of joints
         rpJoint** getJoints();




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
        mContactManifolds[mNbContactManifolds] = contactManifold;
        mNbContactManifolds++;
    }

    // Add a joint into the island
    SIMD_INLINE void rpIsland::addJoint(rpJoint* joint)
    {
        mJoints[mNbJoints] = joint;
        mNbJoints++;
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

    // Return the number of joints in the island
    SIMD_INLINE uint rpIsland::getNbJoints() const
    {
        return mNbJoints;
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

    // Return a pointer to the array of joints
    SIMD_INLINE rpJoint** rpIsland::getJoints()
    {
        return mJoints;
    }

}

#endif // RPISLAND_H
