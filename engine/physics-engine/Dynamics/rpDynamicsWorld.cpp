/*
 * rpDynamicsWorld.cpp
 *
 *  Created on: 15 дек. 2016 г.
 *      Author: wera
 */

#include <assert.h>
#include "../Dynamics/rpDynamicsWorld.h"





namespace real_physics
{


bool status;

rpDynamicsWorld::rpDynamicsWorld(const Vector3& gravity)
: mGravity(gravity),
  mNbVelocitySolverIterations(DEFAULT_VELOCITY_SOLVER_NB_ITERATIONS),
  mNbPositionSolverIterations(DEFAULT_POSITION_SOLVER_NB_ITERATIONS)
{
   status = true;
}


// Destroy all the  world
rpDynamicsWorld::~rpDynamicsWorld()
{

    destroy();

    /**

        status = false;

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
        if(!mCollisionContactOverlappingPairs.empty())
        {
            for( auto pair : mCollisionContactOverlappingPairs )
            {
                delete pair.second;
            }

            mCollisionContactOverlappingPairs.clear();

        }


        assert(mPhysicsJoints.size() == 0);
        assert(mPhysicsBodies.size() == 0);
        assert(mContactSolvers.empty());
        assert(mCollisionContactOverlappingPairs.empty());

    /**/

}


// Destroy all the  world
void rpDynamicsWorld::destroy()
{

    status = false;

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
    if(!mCollisionContactOverlappingPairs.empty())
    {
        for( auto pair : mCollisionContactOverlappingPairs )
        {
            delete pair.second;
        }

        mCollisionContactOverlappingPairs.clear();
    }

    assert(mPhysicsJoints.size() == 0);
    assert(mPhysicsBodies.size() == 0);
    assert(mContactSolvers.empty());
    assert(mCollisionContactOverlappingPairs.empty());

}



void rpDynamicsWorld::update(scalar timeStep)
{
	if(!status) return;

	integrateGravity(timeStep);


	/****************************/
	updateBodiesState(timeStep);
	/****************************/
	CollidePhase();
	DynamicPhase(timeStep);
	/****************************/
	integrateBodiesVelocities(timeStep);


}


void rpDynamicsWorld::CollidePhase()
{

	for( auto pair : mContactSolvers )
	{
		pair.second->isFakeCollid = false;
	}


	/// delete overlapping pairs collision
	if(!mCollisionContactOverlappingPairs.empty() || true)
	{
		for( auto pair : mCollisionContactOverlappingPairs )
		{
			delete pair.second;
		}

		mCollisionContactOverlappingPairs.clear();
	}

	/// Overlapping pairs in contact (during the current Narrow-phase collision detection)
	//static std::map<overlappingpairid, rpOverlappingPair*> mCollisionContactOverlappingPairs;
	mCollisionDetection.computeCollisionDetection(mCollisionContactOverlappingPairs);


	for( auto pair : mCollisionContactOverlappingPairs )
	{

		for (int i = 0; i < pair.second->getContactManifoldSet().getNbContactManifolds(); ++i)
		{
			rpContactManifold* maniflod = pair.second->getContactManifoldSet().getContactManifold(i);
			overlappingpairid pairId = rpOverlappingPair::computeID( maniflod->mShape1 , maniflod->mShape2 );
			//pairKey key(maniflod->mShape1 , maniflod->mShape2);
		    addChekCollisionPair(pairId, maniflod);
		}
	}

	/// delete overlapping pairs collision
	for( auto pair : mContactSolvers )
	{
		if( !pair.second->isFakeCollid )
		{
			delete mContactSolvers.find(pair.first)->second;
			       mContactSolvers.erase(pair.first);
		}
	}

}



void rpDynamicsWorld::DynamicPhase( scalar timeStep )
{

    //---------------------------------------------------------------------//

    for( auto it = mPhysicsJoints.begin(); it != mPhysicsJoints.end(); ++it )
    {
    	(*it)->initBeforeSolve(timeStep);
    	(*it)->warmstart();
    }

    //---------------------------------------------------------------------//

	for( auto pair : mContactSolvers )
	{

		pair.second->initializeForIsland(timeStep);
	    pair.second->warmStart();

	}

	//---------------------------------------------------------------------//


	for( uint i = 0; i < mNbVelocitySolverIterations; ++i)
	{
		for( auto it = mPhysicsJoints.begin(); it != mPhysicsJoints.end(); ++it )
		{
			(*it)->solveVelocityConstraint();
		}

		for( auto pair : mContactSolvers )
		{
			pair.second->solveVelocityConstraint();
		}
	}

	//---------------------------------------------------------------------//

	for( uint i = 0; i < mNbPositionSolverIterations; ++i)
	{
		for( auto it = mPhysicsJoints.begin(); it != mPhysicsJoints.end(); ++it )
		{
			(*it)->solvePositionConstraint();
		}

		for( auto pair : mContactSolvers )
		{
			pair.second->solvePositionConstraint();
		}
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


//// Put bodies to sleep if needed.
///// For each island, if all the bodies have been almost still for a long enough period of
///// time, we put all the bodies of the island to sleep.
//void rpDynamicsWorld::updateSleepingBodies(scalar timeStep)
//{
//
//	const scalar sleepLinearVelocitySquare  = DEFAULT_SLEEP_LINEAR_VELOCITY * DEFAULT_SLEEP_LINEAR_VELOCITY;//mSleepLinearVelocity * mSleepLinearVelocity;
//    const scalar sleepAngularVelocitySquare = DEFAULT_SLEEP_ANGULAR_VELOCITY * DEFAULT_SLEEP_ANGULAR_VELOCITY;//mSleepAngularVelocity * mSleepAngularVelocity;
//    const scalar mTimeBeforeSleep           = 0.3;//DEFAULT_TIME_BEFORE_SLEEP;
//          scalar minSleepTime               = 1;// DECIMAL_LARGEST;
//
//
//
//          scalar min=0;
//    for( int i = 0; i < mPhysicBodies.size(); ++i )
//	{
//
//    	rpRigidPhysicsBody *bodies = static_cast<rpRigidPhysicsBody*>(mPhysicBodies[i]);
//    	// Skip static bodies
//        if (bodies->getType() == STATIC) continue;
//
//        // If the body is velocity is large enough to stay awake
//        if (bodies->getLinearVelocity().lengthSquare() > sleepLinearVelocitySquare   ||
//        	bodies->getAngularVelocity().lengthSquare() > sleepAngularVelocitySquare ||
//		   !bodies->isAllowedToSleep())
//        {
//
//        	// Reset the sleep time of the body
//        	bodies->mSleepTime = scalar(0.0);
//        	minSleepTime = scalar(0.0);
//        }
//        else
//        {  // If the body velocity is bellow the sleeping velocity threshold
//
//
//        	// Increase the sleep time
//        	bodies->mSleepTime += timeStep;
//
//
//        	if (bodies->mSleepTime < minSleepTime && bodies->mSleepTime != scalar(0))
//        	{
//        		minSleepTime = bodies->mSleepTime;
//        		min = bodies->mSleepTime;
//
//        	}
//        }
//
//	}
//
//    //cout << "min sleep:  " <<  min <<endl;
//
//    // If the velocity of all the bodies of the island is under the
//    // sleeping velocity threshold for a period of time larger than
//    // the time required to become a sleeping body
//    if (min >= mTimeBeforeSleep)
//    {
//    	cout << "sleep:  " << min <<endl;
//    	// Put all the bodies of the island to sleep
//    	for( int i = 0; i < mPhysicBodies.size(); ++i )
//    	{
//    		mPhysicBodies[i]->setIsSleeping(true);
//    	}
//    }
//
//}



void rpDynamicsWorld::addChekCollisionPair( overlappingpairid keyPair, rpContactManifold* maniflod )
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


		//    // Destroy all the joints in which the rigid body to be destroyed is involved
		//    JointListElement* element;
		//    for (element = rigidBody->mJointsList; element != NULL; element = element->next) {
		//        destroyJoint(element->joint);
		//    }
		//
		//    // Reset the contact manifold list of the body
		//    rigidBody->resetContactManifoldsList();


		    // Call the destructor of the rigid body
            rigidBody->~rpPhysicsBody();

		    // Remove the rigid body from the list of rigid bodies
		    mBodies.erase(rigidBody);
		    mPhysicsBodies.erase(rigidBody);

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
			cout << "is not method class" <<endl;
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
	    //addJointToBody(newJoint);

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
	//joint->mBody1->removeJointFromJointsList(mMemoryAllocator, joint);
	//joint->mBody2->removeJointFromJointsList(mMemoryAllocator, joint);

	size_t nbBytes = joint->getSizeInBytes();

	// Call the destructor of the joint
	delete joint;


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


