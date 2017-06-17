#ifndef CONSTRAINTSOLVER_H
#define CONSTRAINTSOLVER_H



#include "../../LinearMaths/mathematics.h"
#include "../Joint/rpJoint.h"
#include "../rpIsland.h"
#include <map>
#include <set>


namespace real_physics
{



// Structure ConstraintSolverData
/**
 * This structure contains data from the constraint solver that are used to solve
 * each joint constraint.
 */
struct ConstraintSolverData
{

    public :

        /// Current time step of the simulation
        scalar timeStep;

        /// True if warm starting of the solver is active
        bool isWarmStartingActive;


/**
//        /// Array with the bodies linear velocities
//        Vector3* linearVelocities;

//        /// Array with the bodies angular velocities
//        Vector3* angularVelocities;

//        /// Reference to the bodies positions
//        Vector3* positions;

//        /// Reference to the bodies orientations
//        Quaternion* orientations;

//        /// Reference to the map that associates rigid body to their index
//        /// in the constrained velocities array
//        const std::map<RigidBody*, uint>& mapBodyToConstrainedVelocityIndex;
/**/



         ConstraintSolverData()
         {

         }


//        /// Constructor
//        ConstraintSolverData(const std::map<RigidBody*, uint>& refMapBodyToConstrainedVelocityIndex)
//                           :linearVelocities(NULL), angularVelocities(NULL),
//                            positions(NULL), orientations(NULL),
//                            mapBodyToConstrainedVelocityIndex(refMapBodyToConstrainedVelocityIndex){

//        }

};

class ConstraintSolver
{

private :

    // -------------------- Attributes -------------------- //

    /// Reference to the map that associates rigid body to their index in
    /// the constrained velocities array
    //const std::map<RigidBody*, uint>& mMapBodyToConstrainedVelocityIndex;

    /// Current time step
    scalar mTimeStep;

    /// True if the warm starting of the solver is active
    bool mIsWarmStartingActive;

    /// Constraint solver data used to initialize and solve the constraints
    ConstraintSolverData mConstraintSolverData;


    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        ConstraintSolver();

        /// Destructor
       ~ConstraintSolver();

        /// Initialize the constraint solver for a given island
        void initializeForIsland(scalar dt, rpIsland* island);

        /// Solve the constraints
        void solveVelocityConstraints(rpIsland* island);

        /// Solve the position constraints
        void solvePositionConstraints(rpIsland* island);

        /// Return true if the Non-Linear-Gauss-Seidel position correction technique is active
        bool getIsNonLinearGaussSeidelPositionCorrectionActive() const;

        /// Enable/Disable the Non-Linear-Gauss-Seidel position correction technique.
        void setIsNonLinearGaussSeidelPositionCorrectionActive(bool isActive);

};

}

#endif // CONSTRAINTSOLVER_H
