/*
 * rpDynamicsWorld.h
 *
 *  Created on: 15 дек. 2016 г.
 *      Author: wera
 */

#ifndef SOURCE_ENGIE_KINEMATICPHYSICS_RPDYNAMICSWORLD_H_
#define SOURCE_ENGIE_KINEMATICPHYSICS_RPDYNAMICSWORLD_H_

#include  <set>
#include  <map>

#include "../Body/Material/rpPhysicsMaterial.h"
#include "../Memory/memory.h"

#include "../Dynamics/Solver/rpSequentialImpulseObjectSolver.h"
#include "../Body/rpPhysicsBody.h"
#include "../Body/rpPhysicsObject.h"
#include "../Body/rpRigidPhysicsBody.h"

#include "Joint/rpJoint.h"
#include "Joint/rpBallAndSocketJoint.h"
#include "Joint/rpDistanceJoint.h"
#include "Joint/rpFixedJoint.h"
#include "Joint/rpHingeJoint.h"
#include "Joint/rpSliderJoint.h"

#include "rpTimer.h"


using namespace std;

namespace real_physics
{


//struct pairKey
//{
//    pairKey( rpProxyShape* b1,
//    		      rpProxyShape* b2 )
//    {
//        if (b1 < b2 )
//        {
//            body1 = b1;
//            body2 = b2;
//        }
//        else
//        {
//            body1 = b2;
//            body2 = b1;
//        }
//    }
//
//
//
//    ~pairKey()
//    {
//        //if(body1) delete body1;
//        //if(body2) delete body2;
//    }
//
//    bool operator <(const pairKey& a1) const
//    {
//        if (a1.body1 < body1)
//            return true;
//
//        if (a1.body1 == body1 && a1.body2 < body2)
//            return true;
//
//        return false;
//    }
//
//    rpProxyShape* body1;
//    rpProxyShape* body2;
//};






//*******************************************************//

class rpDynamicsWorld : public rpCollisionWorld
{
	 private:


	// -------------------- Attributes -------------------- //

    /// Update time step correctly interval
    rpTimer mTimer;

    /// True if the spleeping technique for inactive bodies is enabled
    bool mIsSleepingEnabled;

	/// Number of iterations for the velocity solver of the Sequential Impulses technique
	uint mNbVelocitySolverIterations;

	/// Number of iterations for the position solver of the Sequential Impulses technique
	uint mNbPositionSolverIterations;

	// Gravity vector for integrate gravity
	Vector3 mGravity;


	std::set<rpJoint*>       mPhysicsJoints;
    std::set<rpPhysicsBody*> mPhysicsBodies;


	// array map contacts solver
	std::map< overlappingpairid , rpContactSolver* > mContactSolvers;


    // -------------------- Methods -------------------- //


    /// Private copy-constructor
    rpDynamicsWorld(const rpDynamicsWorld& world);

    /// Private assignment operator
    rpDynamicsWorld& operator=(const rpDynamicsWorld& world);


    /// Detection for all collision pairs
    void Collision();

    /// Compute physics for all collision pairs
    void Dynamics( scalar timeStep );

	/// Integrate the garvity
	void integrateGravity( scalar timeStep );

	/// Integrate the positions and orientations of rigid bodies.
	void integrateBodiesVelocities( scalar timeStep );

	/// Update the postion/orientation of the bodies
	void updateBodiesState(  scalar timeStep );

	 /// Put bodies to sleep if needed.
	void updateSleepingBodies(scalar timeStep);


	//// Add Collision New contact Solver
	void addChekCollisionPair( overlappingpairid keyPair , rpContactManifold* maniflod );



	 public:

             rpDynamicsWorld( const Vector3& gravity );

	virtual ~rpDynamicsWorld();


    ///  Destroy
    void destroy();

	//***************************************************//

	 /// Create a rigid body into the physics world.
	rpRigidPhysicsBody* createRigidBody(const Transform& transform);


    /// Destroy a rigid body and all the joints which it belongs
	void destroyBody(rpPhysicsBody* rigidBody);


	/// Create a joint between two bodies in the world and return a pointer to the new joint
	rpJoint* createJoint(const rpJointInfo& jointInfo);


	 /// Destroy a joint
	void destroyJoint(rpJoint* joint);


    /// Add the joint to the list of joints of the two bodies involved in the joint
    void addJointToBody(rpJoint* joint);

	//***************************************************//

    ///  Update Physics simulation - Real-Time ( Semi-AntiFixed timestep )
    void update( scalar timeStep );

    ///  Update Physics simulation - Real-Time ( Fixed timestep )
    void updateFixedTime( scalar timeStep );

    /// Get the number of iterations for the velocity constraint solver
    uint getNbIterationsVelocitySolver() const;

    /// Set the number of iterations for the velocity constraint solver
    void setNbIterationsVelocitySolver(uint nbIterations);

    /// Get the number of iterations for the position constraint solver
    uint getNbIterationsPositionSolver() const;

    /// Set the number of iterations for the position constraint solver
    void setNbIterationsPositionSolver(uint nbIterations);

};


} /* namespace real_physics */

#endif /* SOURCE_ENGIE_KINEMATICPHYSICS_RPDYNAMICSWORLD_H_ */
