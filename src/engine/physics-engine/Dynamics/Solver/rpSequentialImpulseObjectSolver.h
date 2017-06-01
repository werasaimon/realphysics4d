/*
 * rpSequentialImpulseObjectSolver.h
 *
 *  Created on: 23 дек. 2016 г.
 *      Author: wera
 */

#ifndef SOURCE_ENGIE_KINEMATICPHYSICS_SOLVER_RPSEQUENTIALIMPULSEOBJECTSOLVER_H_
#define SOURCE_ENGIE_KINEMATICPHYSICS_SOLVER_RPSEQUENTIALIMPULSEOBJECTSOLVER_H_


// Libraries
#include "../../Collision/collision.h"
#include "../../Dynamics/Solver/rpContactSolver.h"
#include "../../Dynamics/Body/rpRigidPhysicsBody.h"

#include "../../config.h"


namespace real_physics
{



class rpSequentialImpulseObjectSolver : public rpContactSolver
{



		/// Pair body1  and body2 for collision to solver
		rpRigidPhysicsBody *mBody1 = nullptr;
		rpRigidPhysicsBody *mBody2 = nullptr;


         // Structure ContactPointSolver
         /**
          * Contact solver internal data structure that to store all the
          * information relative to a contact point
          */
         struct ContactPointSolver
         {


             /// Accumulated normal impulse
             scalar AccumulatedPenetrationImpulse;

             /// Accumulated impulse in the 1st friction direction
             scalar AccumulatedFriction1Impulse;

             /// Accumulated impulse in the 2nd friction direction
             scalar AccumulatedFriction2Impulse;

             /// Accumulated split impulse for penetration correction
             scalar AccumulatedPenetrationSplitImpulse;

             /// Accumulated rolling resistance impulse
             Vector3 AccumulatedRollingResistanceImpulse;

             /// Accumulated rolling resistance impulse
             Vector3 AccumulatedRollingResistanceSplitImpulse;

             /// Normal vector of the contact
             Vector3 normal;

             /// First friction vector in the tangent plane
             Vector3 frictionVector1;

             /// Second friction vector in the tangent plane
             Vector3 frictionVector2;

             /// Old first friction vector in the tangent plane
             Vector3 oldFrictionVector1;

             /// Old second friction vector in the tangent plane
             Vector3 oldFrictionVector2;

             /// Vector from the body 1 center to the contact point
             Vector3 r1;

             /// Vector from the body 2 center to the contact point
             Vector3 r2;

             /// Cross product of r1 with 1st friction vector
             Vector3 r1CrossT1;

             /// Cross product of r1 with 2nd friction vector
             Vector3 r1CrossT2;

             /// Cross product of r2 with 1st friction vector
             Vector3 r2CrossT1;

             /// Cross product of r2 with 2nd friction vector
             Vector3 r2CrossT2;

             /// Cross product of r1 with the contact normal
             Vector3 r1CrossN;

             /// Cross product of r2 with the contact normal
             Vector3 r2CrossN;

             /// Penetration depth
             scalar penetrationDepth;

             /// Velocity restitution bias
             scalar restitutionBias;

             /// Inverse of the matrix K for the penenetration
             scalar inversePenetrationMass;

             /// Inverse of the matrix K for the 1st friction
             scalar inverseFriction1Mass;

             /// Inverse of the matrix K for the 2nd friction
             scalar inverseFriction2Mass;

             /// True if the contact was existing last time step
             bool isRestingContact;

             /// Pointer to the external contact
             rpContactPoint* externalContact;


	};






         struct ContactManifoldSolver
         {

                     /// Index of body 1 in the constraint solver
                     uint indexBody1;

                     /// Index of body 2 in the constraint solver
                     uint indexBody2;

                     /// Inverse of the mass of body 1
                     scalar massInverseBody1;

                     // Inverse of the mass of body 2
                     scalar massInverseBody2;

                     /// Inverse inertia tensor of body 1
                     Matrix3x3 inverseInertiaTensorBody1;

                     /// Inverse inertia tensor of body 2
                     Matrix3x3 inverseInertiaTensorBody2;

                     /// Contact point constraints
                     ContactPointSolver contacts[MAX_CONTACT_POINTS_IN_MANIFOLD];


                     /************************************************/


                     /// Number of contact points
                     uint nbContacts;

                     /// True if the body 1 is of type dynamic
                     bool isBody1DynamicType;

                     /// True if the body 2 is of type dynamic
                     bool isBody2DynamicType;

                     /// Mix of the restitution factor for two bodies
                     scalar restitutionFactor;

                     /// Mix friction coefficient for the two bodies
                     scalar frictionCoefficient;

                     /// Rolling resistance factor between the two bodies
                     scalar rollingResistanceFactor;

                     /// Pointer to the external contact manifold
                     rpContactManifold* externalContactManifold;

                     // - Variables used when friction constraints are apply at the center of the manifold-//

                     /// Average normal vector of the contact manifold
                     Vector3 normal;

                     /// Point on body 1 where to apply the friction constraints
                     Vector3 frictionPointBody1;

                     /// Point on body 2 where to apply the friction constraints
                     Vector3 frictionPointBody2;

                     /// R1 vector for the friction constraints
                     Vector3 r1Friction;

                     /// R2 vector for the friction constraints
                     Vector3 r2Friction;

                     /// Cross product of r1 with 1st friction vector
                     Vector3 r1CrossT1;

                     /// Cross product of r1 with 2nd friction vector
                     Vector3 r1CrossT2;

                     /// Cross product of r2 with 1st friction vector
                     Vector3 r2CrossT1;

                     /// Cross product of r2 with 2nd friction vector
                     Vector3 r2CrossT2;

                     /// Matrix K for the first friction constraint
                     scalar inverseFriction1Mass;

                     /// Matrix K for the second friction constraint
                     scalar inverseFriction2Mass;

                     /// Matrix K for the twist friction constraint
                     scalar inverseTwistFrictionMass;

                     /// Matrix K for the rolling resistance constraint
                     Matrix3x3 inverseRollingResistance;

                     /// First friction direction at contact manifold center
                     Vector3 frictionVector1;

                     /// Second friction direction at contact manifold center
                     Vector3 frictionVector2;

                     /// Old 1st friction direction at contact manifold center
                     Vector3 oldFrictionVector1;

                     /// Old 2nd friction direction at contact manifold center
                     Vector3 oldFrictionVector2;

                     /// First friction direction impulse at manifold center
                     scalar friction1Impulse;

                     /// Second friction direction impulse at manifold center
                     scalar friction2Impulse;

                     /// Twist friction impulse at contact manifold center
                     scalar frictionTwistImpulse;

                     /// Rolling resistance impulse
                     Vector3 rollingResistanceImpulse;
                 };



         // -------------------- Constants --------------------- //
         /// Beta value for the penetration depth position correction without split impulses
         static const scalar BETA;

         /// Beta value for the penetration depth position correction with split impulses
         static const scalar BETA_SPLIT_IMPULSE;

         /// Slop distance (allowed penetration distance between bodies)
         static const scalar SLOP;


         /// Current time step
         scalar mTimeStep;


                          uint   mNbContactManifolds;
         ContactManifoldSolver  *mContactConstraints;

                       uint      mNbManiflods;
         rpContactManifold*      mContactManifolds;




         bool mIsError;

         bool atLeastOneRestingContactPoint;


         bool mIsWarmStartingActive;
         bool mIsSplitImpulseActive;
         bool mIsStaticFriction;
         bool mIsSolveFrictionAtContactManifoldCenterActive;




         /*********************************************************/
         /// Compute the collision restitution factor from the restitution factor of each body
         scalar computeMixedRestitutionFactor(rpRigidPhysicsBody*  body1,
        		                              rpRigidPhysicsBody* body2) const;

         /// Compute the mixed friction coefficient from the friction coefficient of each body
         scalar computeMixedFrictionCoefficient(rpRigidPhysicsBody*  body1,
        		                                rpRigidPhysicsBody*  body2) const;

         /// Compute th mixed rolling resistance factor between two bodies
         scalar computeMixedRollingResistance(rpRigidPhysicsBody*  body1,
        		                              rpRigidPhysicsBody* body2) const;


         /// Compute the two unit orthogonal vectors "t1" and "t2" that span the tangential friction
         /// plane for a contact manifold. The two vectors have to be
         /// such that : t1 x t2 = contactNormal.
         void computeFrictionVectors( const Vector3& deltaVelocity , ContactPointSolver& contactPoint) const;


         /// Compute the two unit orthogonal vectors "t1" and "t2" that span the tangential friction
         /// plane for a contact manifold. The two vectors have to be
         /// such that : t1 x t2 = contactNormal.
         void computeFrictionVectors( const Vector3& deltaVelocity , ContactManifoldSolver* contact ) const;


         /*********************************************************/


//         scalar CalcualteImpuls(const ContactPointSolver &contactPoint, const Vector3 &normal);
//         scalar CalcualteSplitImpuls(const ContactPointSolver &contactPoint, const Vector3 &normal);




		public:
					 rpSequentialImpulseObjectSolver( rpRigidPhysicsBody* body1 ,
							                          rpRigidPhysicsBody* body2 );
			virtual ~rpSequentialImpulseObjectSolver();



			void  initManiflod( rpContactManifold * manilod );

			void  initializeForIsland( scalar dt );
			void  initializeContactConstraints();

			 /// Warm start the solver.
			void warmStart();
			void solveVelocityConstraint();
			void solvePositionConstraint();


			// Store the computed impulses to use them to
			// warm start the solver at the next iteration
			// void storeImpulses();
			void cleanup();


			/**/

			/// Return true if the split impulses position correction technique is used for contacts
			bool isSplitImpulseActive() const;

			/// Activate or Deactivate the split impulses for contacts
			void setIsSplitImpulseActive(bool isActive);

			/// Activate or deactivate the solving of friction constraints at the center of
			/// the contact manifold instead of solving them at each contact point
			void setIsSolveFrictionAtContactManifoldCenterActive(bool isActive);


			bool isIsWarmStartingActive() const
			{
				return mIsWarmStartingActive;
			}

			void setIsWarmStartingActive(bool isWarmStartingActive)
			{
				mIsWarmStartingActive = isWarmStartingActive;
			}

			 // -------------------- Friendship -------------------- //

			friend class rpDynamicsWorld;
};


/********************************************************************************************************/


// Return true if the split impulses position correction technique is used for contacts
SIMD_INLINE  bool rpSequentialImpulseObjectSolver::isSplitImpulseActive() const
{
    return mIsSplitImpulseActive;
}

// Activate or Deactivate the split impulses for contacts
SIMD_INLINE  void rpSequentialImpulseObjectSolver::setIsSplitImpulseActive(bool isActive)
{
    mIsSplitImpulseActive = isActive;
}


// Activate or deactivate the solving of friction constraints at the center of
// the contact manifold instead of solving them at each contact point
SIMD_INLINE  void rpSequentialImpulseObjectSolver::setIsSolveFrictionAtContactManifoldCenterActive(bool isActive)
{
    mIsSolveFrictionAtContactManifoldCenterActive = isActive;
}

} /* namespace real_physics */

#endif /* SOURCE_ENGIE_KINEMATICPHYSICS_SOLVER_RPSEQUENTIALIMPULSEOBJECTSOLVER_H_ */
