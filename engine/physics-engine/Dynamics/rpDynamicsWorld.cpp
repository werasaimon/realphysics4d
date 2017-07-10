/*
 * rpDynamicsWorld.cpp
 *
 *  Created on: 15 дек. 2016 г.
 *      Author: wera
 */

#include <assert.h>
#include "../Dynamics/rpDynamicsWorld.h"
#include "../Dynamics/Joint/rpJoint.h"



namespace real_physics
{




rpDynamicsWorld::rpDynamicsWorld(const Vector3& gravity)
: mGravity(gravity),
  mNbVelocitySolverIterations(DEFAULT_VELOCITY_SOLVER_NB_ITERATIONS),
  mNbPositionSolverIterations(DEFAULT_POSITION_SOLVER_NB_ITERATIONS),
  mTimer( scalar(1.0) ) ,
  mIsSleepingEnabled(SLEEPING_ENABLED) ,
  mNbIslands(0),
  mNbIslandsCapacity(0),
  mIslands(NULL),
  mNbBodiesCapacity(0)
{
    resetContactManifoldListsOfBodies();

    mTimer.start();
}


// Destroy all the  world
rpDynamicsWorld::~rpDynamicsWorld()
{
    destroy();
}


// Destroy all the  world
void rpDynamicsWorld::destroy()
{

    // Stop timer
    mTimer.stop();


    // Destroy all the joints that have not been removed
    for (auto itJoints = mPhysicsJoints.begin(); itJoints != mPhysicsJoints.end();)
    {
        std::set<rpJoint*>::iterator itToRemove = itJoints;
        ++itJoints;
        destroyJoint(*itToRemove);
    }



    // Destroy all the rigid bodies that have not been removed
    for (auto itRigidBodies = mPhysicsBodies.begin(); itRigidBodies != mPhysicsBodies.end();)
    {
        std::set<rpPhysicsBody*>::iterator itToRemove = itRigidBodies;
        ++itRigidBodies;
        destroyBody(*itToRemove);
    }



    // Destroy all pair collisions that have not been removed
    if(!mContactSolvers.empty())
    {
        for( auto pairs : mContactSolvers )
        {
            delete pairs.second;
        }

        mContactSolvers.clear();
    }


    // Destroy all pair collisions that have not been removed
    if(!mCollisionDetection.mContactOverlappingPairs.empty())
    {
        for( auto pair : mCollisionDetection.mContactOverlappingPairs )
        {
            delete pair.second;
        }

        mCollisionDetection.mContactOverlappingPairs.clear();
    }

    assert(mPhysicsJoints.size() == 0);
    assert(mPhysicsBodies.size() == 0);
    assert(mContactSolvers.empty());
    assert(mCollisionDetection.mContactOverlappingPairs.empty());


}


void rpDynamicsWorld::update(scalar timeStep)
{
    mTimer.setTimeStep(timeStep);

  if(mTimer.getIsRunning())
  {
     mTimer.update();

     while( mTimer.isPossibleToTakeStep() )
     {
         updateFixedTime(timeStep);

         // next step simulation
         mTimer.nextStep();
     }
  }

}

void rpDynamicsWorld::updateFixedTime(scalar timeStep)
{

    // Reset all the contact manifolds lists of each body
    resetContactManifoldListsOfBodies();

    //Integrate all bodies
    integrateGravity(timeStep);

    /****************************/

    //Update state bodies of broad phase
    updateBodiesState(timeStep);

    //Update BroadPhase
    updateFindContacts();

    /****************************/

    // Compute the islands (separate groups of bodies with constraints between each others)
    computeIslands();


    // Solve the contacts and constraints joint
    solve(timeStep);

    /****************************/

    // Integrate the position and orientation of each body
    integrateBodiesVelocities(timeStep);


    // Sleeping for all bodies
    if(mIsSleepingEnabled)
    {
      updateSleepingBodies(timeStep);
    }



}


void rpDynamicsWorld::updateFindContacts()
{


//    /// delete overlapping pairs collision
//    if(!mCollisionContactOverlappingPairs.empty())
//    {
//        for( auto pair : mCollisionContactOverlappingPairs )
//        {
//            delete pair.second;
//        }
//        mCollisionContactOverlappingPairs.clear();
//    }



    /// Candidate of delete
    for( auto pair : mContactSolvers )
    {
        pair.second->isCandidateInDelete = false;
    }


    /// Overlapping pairs in contact (during the current Narrow-phase collision detection)
    mCollisionDetection.computeCollisionDetectionAllPairs();



    /// New collision pair
    for( auto pair : mCollisionDetection.mContactOverlappingPairs )
    {
        for (int i = 0; i < pair.second->getContactManifoldSet().getNbContactManifolds(); ++i)
        {
            addChekCollisionPair( pair.second->getContactManifoldSet().getContactManifold(i) );
        }
    }

    /// delete overlapping pairs collision
    for( auto pair : mContactSolvers )
    {
        if( !pair.second->isCandidateInDelete )
        {
            delete pair.second;
            mContactSolvers.erase(pair.first);
        }
    }
}



void rpDynamicsWorld::solve( scalar timeStep )
{

    //---------------------------------------------------------------------//

    for( auto it = mPhysicsJoints.begin(); it != mPhysicsJoints.end(); ++it )
    {
        (*it)->initBeforeSolve(timeStep);
        (*it)->warmstart();
    }

    //---------------------------------------------------------------------//

//    for( auto pair : mContactSolvers )
//    {
//        pair.second->initializeForIsland(timeStep);
//        pair.second->warmStart();
//    }


    // For each island of the world
    for (uint islandIndex = 0; islandIndex < mNbIslands; islandIndex++)
    {
        mIslands[islandIndex]->warmStart( timeStep );
    }


	//---------------------------------------------------------------------//


    for( uint i = 0; i < mNbVelocitySolverIterations; ++i)
    {
        for( auto it = mPhysicsJoints.begin(); it != mPhysicsJoints.end(); ++it )
        {
            (*it)->solveVelocityConstraint();
        }

//        for( auto pair : mContactSolvers )
//        {
//            pair.second->solveVelocityConstraint();
//        }

        // For each island of the world
        for (uint islandIndex = 0; islandIndex < mNbIslands; islandIndex++)
        {
            mIslands[islandIndex]->solveVelocityConstraint();
        }
    }

    //---------------------------------------------------------------------//

    for( uint i = 0; i < mNbPositionSolverIterations; ++i)
    {
        for( auto it = mPhysicsJoints.begin(); it != mPhysicsJoints.end(); ++it )
        {
            (*it)->solvePositionConstraint();
        }

//        for( auto pair : mContactSolvers )
//        {
//            pair.second->solvePositionConstraint();
//        }

        // For each island of the world
        for (uint islandIndex = 0; islandIndex < mNbIslands; islandIndex++)
        {
            mIslands[islandIndex]->solvePositionConstraint();
        }
    }


//    for( auto pair : mContactSolvers )
//    {
//        pair.second->storeImpulses();
//    }


    // For each island of the world
    for (uint islandIndex = 0; islandIndex < mNbIslands; islandIndex++)
    {
        mIslands[islandIndex]->storeImpulses();
    }

}



void rpDynamicsWorld::integrateGravity(scalar timeStep)
{
	for( auto it = mPhysicsBodies.begin(); it != mPhysicsBodies.end(); ++it )
	{
		(*it)->applyGravity(mGravity * timeStep);
	}
}

void rpDynamicsWorld::integrateBodiesVelocities(scalar timeStep)
{

	for( auto it = mPhysicsBodies.begin(); it != mPhysicsBodies.end(); ++it )
	{
		(*it)->Integrate(timeStep);

	}

}


void rpDynamicsWorld::updateBodiesState(scalar timeStep)
{

	for( auto it = mPhysicsBodies.begin(); it != mPhysicsBodies.end(); ++it )
	{
        (*it)->updateBroadPhaseState();
        (*it)->updateTransformWithCenterOfMass();
	}
}


/**/
//// Put bodies to sleep if needed.
///// For each island, if all the bodies have been almost still for a long enough period of
///// time, we put all the bodies of the island to sleep.
void rpDynamicsWorld::updateSleepingBodies(scalar timeStep)
{

    const scalar sleepLinearVelocitySquare  = (DEFAULT_SLEEP_LINEAR_VELOCITY * DEFAULT_SLEEP_LINEAR_VELOCITY);
    const scalar sleepAngularVelocitySquare = (DEFAULT_SLEEP_ANGULAR_VELOCITY * DEFAULT_SLEEP_ANGULAR_VELOCITY);
    const scalar sleepAngularSplitSquare    = (DEFAULT_SLEEP_SPLIT  * DEFAULT_SLEEP_SPLIT);


    // For each island of the world
    for (uint i=0; i<mNbIslands; i++)
    {

        scalar minSleepTime = DECIMAL_LARGEST;

        // For each body of the island
        rpRigidPhysicsBody** bodies = mIslands[i]->getBodies();
        for (uint b=0; b < mIslands[i]->getNbBodies(); b++)
        {

            // Skip static bodies
            if (bodies[b]->getType() == STATIC) continue;

            // If the body is velocity is large enough to stay awake

            // If the body is velocity is large enough to stay awake
            if (bodies[b]->mLinearVelocity.lengthSquare()       >  sleepLinearVelocitySquare   ||
                bodies[b]->mAngularVelocity.lengthSquare()      >  sleepAngularVelocitySquare  ||
                bodies[b]->mSplitLinearVelocity.lengthSquare()  >  sleepAngularSplitSquare     ||
                bodies[b]->mSplitAngularVelocity.lengthSquare() >  sleepAngularSplitSquare     ||
                !bodies[b]->isAllowedToSleep())
            {
                // Reset the sleep time of the body
                bodies[b]->mSleepTime = scalar(0.0);
                minSleepTime = scalar(0.0);
            }
            else
            {  // If the body velocity is bellow the sleeping velocity threshold

                // Increase the sleep time
                bodies[b]->mSleepTime += timeStep;
                if (bodies[b]->mSleepTime < minSleepTime)
                {
                    minSleepTime = bodies[b]->mSleepTime;
                }
            }
        }

        // If the velocity of all the bodies of the island is under the
        // sleeping velocity threshold for a period of time larger than
        // the time required to become a sleeping body
        if ( minSleepTime >= DEFAULT_TIME_BEFORE_SLEEP )
        {

            // Put all the bodies of the island to sleep
            for (uint b=0; b < mIslands[i]->getNbBodies(); b++)
            {
                bodies[b]->setIsSleeping(true);
            }
        }
    }


}



// Compute the islands of awake bodies.
/// An island is an isolated group of rigid bodies that have constraints (joints or contacts)
/// between each other. This method computes the islands at each time step as follows: For each
/// awake rigid body, we run a Depth First Search (DFS) through the constraint graph of that body
/// (graph where nodes are the bodies and where the edges are the constraints between the bodies) to
/// find all the bodies that are connected with it (the bodies that share joints or contacts with
/// it). Then, we create an island with this group of connected bodies.
void rpDynamicsWorld::computeIslands()
{



    uint nbBodies = mPhysicsBodies.size();


    // Clear all the islands
    for (uint i=0; i<mNbIslands; i++)
    {
        // Call the island destructor
        // Release the allocated memory for the island
        delete mIslands[i];
    }

    // Allocate and create the array of islands
    if (mNbIslandsCapacity != nbBodies && nbBodies > 0)
    {
        if (mNbIslandsCapacity > 0)
        {
            delete[] mIslands;
        }

        mNbIslandsCapacity = nbBodies;
        mIslands = new rpIsland*[mNbIslandsCapacity];
    }

    mNbIslands = 0;
    int nbContactManifolds = 0;

    // Reset all the isAlreadyInIsland variables of bodies, joints and contact manifolds
    for (auto it = mPhysicsBodies.begin(); it != mPhysicsBodies.end(); ++it)
    {
        int nbBodyManifolds = (*it)->resetIsAlreadyInIslandAndCountManifolds();
        nbContactManifolds += nbBodyManifolds;
    }

    for (auto it = mPhysicsJoints.begin(); it !=  mPhysicsJoints.end(); ++it)
    {
        (*it)->mIsAlreadyInIsland = false;
    }


    rpRigidPhysicsBody** stackBodiesToVisit = new rpRigidPhysicsBody*[nbBodies];
    int i=0;
    for (auto it = mPhysicsBodies.begin(); it != mPhysicsBodies.end(); ++it )
    {
        rpPhysicsBody* body = *it;
        stackBodiesToVisit[i++] =    static_cast<rpRigidPhysicsBody*>(body);
    }


    // For each rigid body of the world
    for (auto it = mPhysicsBodies.begin(); it != mPhysicsBodies.end(); ++it)
    {

        rpPhysicsBody* body = *it;

        // If the body has already been added to an island, we go to the next body
        if (body->mIsAlreadyInIsland) continue;

        // If the body is static, we go to the next body
        if (body->getType() == STATIC) continue;

        // If the body is sleeping or inactive, we go to the next body
        if (body->isSleeping() || !body->isActive()) continue;

        // Reset the stack of bodies to visit
        uint stackIndex = 0;
        stackBodiesToVisit[stackIndex] = static_cast<rpRigidPhysicsBody*>(body);
        stackIndex++;
        body->mIsAlreadyInIsland = true;



        // Create the new island
        mIslands[mNbIslands] = new rpIsland( nbBodies , nbContactManifolds , mContactSolvers );



        // While there are still some bodies to visit in the stack
        while (stackIndex > 0)
        {

            // Get the next body to visit from the stack
            stackIndex--;
            rpRigidPhysicsBody* bodyToVisit = stackBodiesToVisit[stackIndex];
            assert(bodyToVisit->isActive());

            // Awake the body if it is slepping
            bodyToVisit->setIsSleeping(false);

            // Add the body into the island
            mIslands[mNbIslands]->addBody(bodyToVisit);

            // If the current body is static, we do not want to perform the DFS
            // search across that body
            if (bodyToVisit->getType() == STATIC) continue;

            // For each contact manifold in which the current body is involded
            ContactManifoldListElement* contactElement;
            for (contactElement = bodyToVisit->mContactManifoldsList; contactElement != NULL;  contactElement = contactElement->getNext())
            {
                rpContactManifold* contactManifold = contactElement->getPointer();

                assert(contactManifold->getNbContactPoints() > 0);

                // Check if the current contact manifold has already been added into an island
                if (contactManifold->isAlreadyInIsland()) continue;


                // Add the contact manifold into the island
                mIslands[mNbIslands]->addContactManifold(contactManifold);
                contactManifold->mIsAlreadyInIsland = true;


                // Get the other body of the contact manifold
                rpRigidPhysicsBody* body1 = static_cast<rpRigidPhysicsBody*>(contactManifold->getBody1());
                rpRigidPhysicsBody* body2 = static_cast<rpRigidPhysicsBody*>(contactManifold->getBody2());
                rpRigidPhysicsBody* otherBody = (body1->getID() == bodyToVisit->getID()) ? body2 : body1;

                // Check if the other body has already been added to the island
                if (otherBody->mIsAlreadyInIsland) continue;

                // Insert the other body into the stack of bodies to visit
                stackBodiesToVisit[stackIndex] = otherBody;
                stackIndex++;
                otherBody->mIsAlreadyInIsland = true;
            }



            // For each joint in which the current body is involved
            rpListElement<rpJoint>* jointElement;
            for (jointElement = bodyToVisit->mJointsList; jointElement != NULL; jointElement = jointElement->getNext())
            {
                rpJoint* joint = jointElement->getPointer();

                // Check if the current joint has already been added into an island
                if (joint->isAlreadyInIsland()) continue;

                // Add the joint into the island
                //mIslands[mNbIslands]->addJoint(joint);
                joint->mIsAlreadyInIsland = true;

                // Get the other body of the contact manifold
                rpRigidPhysicsBody* body1 = static_cast<rpRigidPhysicsBody*>(joint->getBody1());
                rpRigidPhysicsBody* body2 = static_cast<rpRigidPhysicsBody*>(joint->getBody2());
                rpRigidPhysicsBody* otherBody = (body1->getID() == bodyToVisit->getID()) ? body2 : body1;

                // Check if the other body has already been added to the island
                if (otherBody->mIsAlreadyInIsland) continue;

                // Insert the other body into the stack of bodies to visit
                stackBodiesToVisit[stackIndex] = otherBody;
                stackIndex++;
                otherBody->mIsAlreadyInIsland = true;
            }
        }



        // Reset the isAlreadyIsland variable of the static bodies so that they
        // can also be included in the other islands
        for (uint i=0; i < mIslands[mNbIslands]->mNbBodies; i++)
        {

            if (mIslands[mNbIslands]->mBodies[i]->getType() == STATIC)
            {
                mIslands[mNbIslands]->mBodies[i]->mIsAlreadyInIsland = false;
            }
        }

        mNbIslands++;



    }


    delete[] stackBodiesToVisit;



}



///add colision check pairs
void rpDynamicsWorld::addChekCollisionPair( rpContactManifold* manifold )
{

    overlappingpairid keyPair = rpOverlappingPair::computeID( manifold->mShape1 ,
                                                              manifold->mShape2 );

    if(mContactSolvers.find(keyPair) == mContactSolvers.end())
	{

        rpRigidPhysicsBody *body1 = static_cast<rpRigidPhysicsBody*>(manifold->mShape2->getBody());
        rpRigidPhysicsBody *body2 = static_cast<rpRigidPhysicsBody*>(manifold->mShape1->getBody());


        rpContactSolver *solverObject = new rpContactSolverSequentialImpulseObject( body1 , body2 );
		mContactSolvers.insert( std::make_pair(keyPair, solverObject) );
	}


    mContactSolvers[keyPair]->initManiflod(manifold);
    mContactSolvers[keyPair]->isCandidateInDelete = true;

}


rpRigidPhysicsBody* rpDynamicsWorld::createRigidBody(const Transform& transform)
{

	// Compute the body ID
	bodyindex bodyID = computeNextAvailableBodyID();

	// Largest index cannot be used (it is used for invalid index)
	assert(bodyID < std::numeric_limits<bodyindex>::max());

	// Create the rigid body
	rpRigidPhysicsBody* rigidBody = new rpRigidPhysicsBody(transform, &mCollisionDetection, bodyID);
	assert(rigidBody != NULL);


	// Add the rigid body to the physics world
	mBodies.insert(rigidBody);
	mPhysicsBodies.insert(rigidBody);


	// Return the pointer to the rigid body
	return rigidBody;

}

void rpDynamicsWorld::destroyBody(rpPhysicsBody* rigidBody)
{

    // Remove all the collision shapes of the body
    rigidBody->removeAllCollisionShapes();

    // Add the body ID to the list of free IDs
    mFreeBodiesIDs.push_back(rigidBody->getID());


    // Destroy all the joints in which the rigid body to be destroyed is involved
    JointListElement* element;
    if(rigidBody->mJointsList != NULL )
    {
      for (element = rigidBody->mJointsList; element != NULL; element = element->getNext() )
      {
           destroyJoint(element->getPointer());
           element = NULL;
      }
    }
    rigidBody->mJointsList = NULL;


    // Reset the contact manifold list of the body
    rigidBody->resetContactManifoldsList();


    // Remove the rigid body from the list of rigid bodies
    mBodies.erase(rigidBody);
    mPhysicsBodies.erase(rigidBody);


    // Call the destructor of the rigid body
    // Free the object from the memory allocator
    delete rigidBody;

}



rpJoint* rpDynamicsWorld::createJoint(const rpJointInfo& jointInfo)
{

	   rpJoint* newJoint = NULL;


	   switch (jointInfo.type)
	   {
		  case DISTANCEJOINT:
		  {
              const rpDistanceJointInfo& info = static_cast<const rpDistanceJointInfo&>(jointInfo);
              newJoint = new rpDistanceJoint(info);

			  break;
		  }


		  case BALLSOCKETJOINT:
		  {
			  const rpBallAndSocketJointInfo& info = static_cast<const rpBallAndSocketJointInfo&>(jointInfo);
			  newJoint = new rpBallAndSocketJoint(info);

			  break;
		  }


		  case HINGEJOINT:
		  {
			  const rpHingeJointInfo& info = static_cast<const rpHingeJointInfo&>(jointInfo);
			  newJoint = new rpHingeJoint(info);

			  break;
		  }

		  case SLIDERJOINT:
		  {
			  const rpSliderJointInfo& info = static_cast<const rpSliderJointInfo&>(jointInfo);
			  newJoint = new rpSliderJoint(info);

			  break;
		  }

		  case FIXEDJOINT:
		  {
			  const rpFixedJointInfo& info = static_cast<const rpFixedJointInfo&>(jointInfo);
			  newJoint = new rpFixedJoint(info);

			  break;
		  }

		default:
            cout << "is not init joint info " <<endl;
			break;
	}



	    // If the collision between the two bodies of the constraint is disabled
	    if (!jointInfo.isCollisionEnabled)
	    {
	        // Add the pair of bodies in the set of body pairs that cannot collide with each other
	        mCollisionDetection.addNoCollisionPair(jointInfo.body1, jointInfo.body2);
	    }

	    // Add the joint into the world
	      mPhysicsJoints.insert(newJoint);


	    // Add the joint into the joint list of the bodies involved in the joint
          addJointToBody(newJoint);

	    // Return the pointer to the created joint
          return newJoint;
}

void rpDynamicsWorld::destroyJoint(rpJoint* joint)
{


	assert(joint != NULL);
	// If the collision between the two bodies of the constraint was disabled
	if (!joint->isCollisionEnabled())
	{
		// Remove the pair of bodies from the set of body pairs that cannot collide with each other
		mCollisionDetection.removeNoCollisionPair(joint->getBody1(), joint->getBody2());
	}

	// Wake up the two bodies of the joint
	joint->getBody1()->setIsSleeping(false);
	joint->getBody2()->setIsSleeping(false);

	// Remove the joint from the world
	mPhysicsJoints.erase(joint);

	// Remove the joint from the joint list of the bodies involved in the joint
    joint->mBody1->removeJointFromJointsList( joint );
    joint->mBody2->removeJointFromJointsList( joint );

	size_t nbBytes = joint->getSizeInBytes();

	// Call the destructor of the joint
	delete joint;


}

void rpDynamicsWorld::addJointToBody(rpJoint *joint)
{
    assert(joint != NULL);

    // Add the joint at the beginning of the linked list of joints of the first body
    JointListElement* jointListElement1 = new JointListElement(joint , joint->mBody1->mJointsList , NULL );
    joint->mBody1->mJointsList = jointListElement1;

    // Add the joint at the beginning of the linked list of joints of the second body
    JointListElement* jointListElement2 = new JointListElement(joint , joint->mBody2->mJointsList , NULL );
    joint->mBody2->mJointsList = jointListElement2;

}



uint rpDynamicsWorld::getNbIterationsVelocitySolver() const
{
	return mNbVelocitySolverIterations;
}

void rpDynamicsWorld::setNbIterationsVelocitySolver(uint nbIterations)
{
	mNbVelocitySolverIterations = nbIterations;
}

uint rpDynamicsWorld::getNbIterationsPositionSolver() const
{
	return mNbPositionSolverIterations;
}


void rpDynamicsWorld::setNbIterationsPositionSolver(uint nbIterations)
{
	 mNbPositionSolverIterations = nbIterations;
}


} /* namespace real_physics */


