/*
 * rpContactSolver.h
 *
 *  Created on: 16 дек. 2016 г.
 *      Author: wera
 */

#ifndef SOURCE_ENGIE_KINEMATICPHYSICS_SOLVER_RPCONTACTSOLVER_H_
#define SOURCE_ENGIE_KINEMATICPHYSICS_SOLVER_RPCONTACTSOLVER_H_

#include "../../LinearMaths/rpLinearMtah.h"




namespace real_physics
{

class rpContactManifold;
class rpPhysicsBody;

class rpContactSolver
{

 protected:

	bool isFakeCollid;

	public:
	         rpContactSolver();
	virtual ~rpContactSolver();

	/// set add To maniflod for contact Pair
	virtual void  initManiflod( rpContactManifold * manilod ) = 0;

	///initilize solver LCP
	virtual void  initializeForIsland( scalar dt ) = 0;
	virtual void  initializeContactConstraints() = 0;

	/// Warm start the solver.
	virtual void warmStart() = 0;
	virtual void solveVelocityConstraint() = 0;
	virtual void solvePositionConstraint() = 0;


	// -------------------- Friendship -------------------- //

	friend class rpDynamicsWorld;
};

} /* namespace real_physics */

#endif /* SOURCE_ENGIE_KINEMATICPHYSICS_SOLVER_RPCONTACTSOLVER_H_ */
