/*
 * rpContactSolverSequentialImpulseObject.cpp
 *
 *  Created on: 23 дек. 2016 г.
 *      Author: wera
 */

#include "../../Dynamics/Solver/rpContactSolverSequentialImpulseObject.h"

namespace real_physics
{



// Constants initialization
const scalar rpContactSolverSequentialImpulseObject::BETA = scalar(0.2);
const scalar rpContactSolverSequentialImpulseObject::BETA_SPLIT_IMPULSE = scalar(0.2);
const scalar rpContactSolverSequentialImpulseObject::SLOP= scalar(0.01);



rpContactSolverSequentialImpulseObject::rpContactSolverSequentialImpulseObject( rpRigidPhysicsBody* body1 ,
                                                                                rpRigidPhysicsBody* body2)
:mBody1(body1), mBody2(body2),
 mIsWarmStartingActive(true),
 mIsSplitImpulseActive(true),
 mIsStaticFriction(true),
 mIsSolveFrictionAtContactManifoldCenterActive(true)
{

    //mContactManifolds = new rpContactManifold;
    mContactConstraints = new ContactManifoldSolver;


    for (uint i = 0; i < MAX_CONTACT_POINTS_IN_MANIFOLD; ++i)
    {

        // Initialize the accumulated impulses to zero
        mContactConstraints->contacts[i].AccumulatedPenetrationImpulse = scalar(0);
        mContactConstraints->contacts[i].AccumulatedFriction1Impulse   = scalar(0);
        mContactConstraints->contacts[i].AccumulatedFriction2Impulse   = scalar(0);
        mContactConstraints->contacts[i].AccumulatedRollingResistanceImpulse = Vector3::ZERO;
        mContactConstraints->contacts[i].AccumulatedRollingResistanceSplitImpulse = Vector3::ZERO;
        mContactConstraints->contacts[i].AccumulatedPenetrationSplitImpulse = scalar(0);
        mContactConstraints->contacts[i].isRestingContact = false;
    }


    // Initialize the accumulated impulses to zero
    mContactConstraints->AccumulatedFriction1Impulse = 0.0;
    mContactConstraints->AccumulatedFriction2Impulse = 0.0;
    mContactConstraints->AccumulatedFrictionTwistImpulse = 0.0;
    mContactConstraints->AccumulatedRollingResistanceImpulse = Vector3::ZERO;
    mContactConstraints->AccumulatedRollingResistanceSplitImpulse = Vector3::ZERO;

    atLeastOneRestingContactPoint = false;
}

rpContactSolverSequentialImpulseObject::~rpContactSolverSequentialImpulseObject()
{
    if (mContactConstraints != NULL)
    {
          delete   mContactConstraints;
                   mContactConstraints = NULL;
    }
}



SIMD_INLINE void rpContactSolverSequentialImpulseObject::initManiflod(rpContactManifold* manilod)
{
    mContactManifolds = manilod;
}





SIMD_INLINE void rpContactSolverSequentialImpulseObject::initializeForIsland(scalar dt)
{


    // WTF .......?
        mIsError = (mContactManifolds->mNbContactPoints > MAX_CONTACT_POINTS_IN_MANIFOLD);
    if( mIsError )
    {
        cout<<"error contact , Because (NbSizeContact > 32) !!!. WTF , bitch ?"<<endl;
        return;
    }
    //-------------------------------------------------------//


    rpRigidPhysicsBody*  body1 = (rpRigidPhysicsBody* )(mBody1);
    rpRigidPhysicsBody*  body2 = (rpRigidPhysicsBody* )(mBody2);


    mTimeStep = dt;

    //mNbContactManifolds = 1;

    rpContactManifold* externalManifold = mContactManifolds;
    ContactManifoldSolver* internalManifold = mContactConstraints;



    assert(body1 != NULL);
    assert(body2 != NULL);


    // Get the position of the two bodies
    const Vector3& x1 = body1->mCenterOfMassWorld;
    const Vector3& x2 = body2->mCenterOfMassWorld;

    // printf( "%i -h-h-h : \n" , externalManifold->getNbContactPoints() );

    // Initialize the internal contact manifold structure using the external
    // contact manifold
    internalManifold->inverseInertiaTensorBody1 = body1->mInertiaTensorWorldInverse;
    internalManifold->inverseInertiaTensorBody2 = body2->mInertiaTensorWorldInverse;
    internalManifold->massInverseBody1 = body1->mMassInverse;
    internalManifold->massInverseBody2 = body2->mMassInverse;
    internalManifold->nbContacts = externalManifold->getNbContactPoints();
    internalManifold->restitutionFactor = computeMixedRestitutionFactor(body1, body2);
    internalManifold->frictionCoefficient = computeMixedFrictionCoefficient(body1, body2);
    internalManifold->rollingResistanceFactor = computeMixedRollingResistance(body1, body2);
    internalManifold->externalContactManifold = externalManifold;
    internalManifold->isBody1DynamicType = body1->mType == DYNAMIC;
    internalManifold->isBody2DynamicType = body2->mType == DYNAMIC;



    // If we solve the friction constraints at the center of the contact manifold
    if (mIsSolveFrictionAtContactManifoldCenterActive)
    {
        internalManifold->frictionPointBody1 = Vector3::ZERO;
        internalManifold->frictionPointBody2 = Vector3::ZERO;
    }



    // For each  contact point of the contact manifold
    for (uint c=0; c<externalManifold->getNbContactPoints(); c++)
    {


        ContactPointSolver& contactPoint = internalManifold->contacts[c];
        // Get a contact point
        rpContactPoint* externalContact = externalManifold->getContactPoint(c);


        // Get the contact point on the two bodies
        Vector3 p1 = externalContact->getWorldPointOnBody1();
        Vector3 p2 = externalContact->getWorldPointOnBody2();

        contactPoint.externalContact = externalContact;
        contactPoint.normal = externalContact->getNormal();
        contactPoint.r1 = p1 - x1;
        contactPoint.r2 = p2 - x2;
        contactPoint.penetrationDepth = externalContact->getPenetrationDepth();
        contactPoint.isRestingContact = true;//externalContact->getIsRestingContact();
        //externalContact->setIsRestingContact(true);
        contactPoint.oldFrictionVector1 = externalContact->getFrictionVector1();
        contactPoint.oldFrictionVector2 = externalContact->getFrictionVector2();


        //        // Get the cached accumulated impulses from the previous step
        //        contactPoint.AccumulatedPenetrationImpulse = 0.0;
        //        contactPoint.AccumulatedFriction1Impulse = 0.0;
        //        contactPoint.AccumulatedFriction2Impulse = 0.0;
        //        contactPoint.AccumulatedRollingResistanceImpulse = Vector3::ZERO;


        // If we solve the friction constraints at the center of the contact manifold
        if (mIsSolveFrictionAtContactManifoldCenterActive)
        {
            internalManifold->frictionPointBody1 += p1;
            internalManifold->frictionPointBody2 += p2;
        }
    }


    // If we solve the friction constraints at the center of the contact manifold
    if (mIsSolveFrictionAtContactManifoldCenterActive && internalManifold->nbContacts > 0)
    {
        internalManifold->frictionPointBody1 /=static_cast<scalar>(float(internalManifold->nbContacts));
        internalManifold->frictionPointBody2 /=static_cast<scalar>(float(internalManifold->nbContacts));
        internalManifold->r1Friction = internalManifold->frictionPointBody1 - x1;
        internalManifold->r2Friction = internalManifold->frictionPointBody2 - x2;
        internalManifold->oldFrictionVector1 = externalManifold->getFrictionVector1();
        internalManifold->oldFrictionVector2 = externalManifold->getFrictionVector2();

        // If warm starting is active
        if (mIsWarmStartingActive)
        {

                //                        // Initialize the accumulated impulses with the previous step accumulated impulses
                //                        internalManifold->AccumulatedFriction1Impulse = externalManifold->getFrictionImpulse1();
                //                        internalManifold->AccumulatedFriction2Impulse = externalManifold->getFrictionImpulse2();
                //                        internalManifold->AccumulatedFrictionTwistImpulse = externalManifold->getFrictionTwistImpulse();
        }
        else
        {

            // Initialize the accumulated impulses to zero
            internalManifold->AccumulatedFriction1Impulse = 0.0;
            internalManifold->AccumulatedFriction2Impulse = 0.0;
            internalManifold->AccumulatedFrictionTwistImpulse = 0.0;
            internalManifold->AccumulatedRollingResistanceImpulse = Vector3(0, 0, 0);
            internalManifold->AccumulatedRollingResistanceSplitImpulse = Vector3(0, 0, 0);
        }
    }


    initializeContactConstraints();

}




SIMD_INLINE void rpContactSolverSequentialImpulseObject::initializeContactConstraints()
{


    rpRigidPhysicsBody*  body1 = (rpRigidPhysicsBody* )(mBody1);
    rpRigidPhysicsBody*  body2 = (rpRigidPhysicsBody* )(mBody2);


    ContactManifoldSolver* manifold = mContactConstraints;

    // Get the inertia tensors of both bodies
    Matrix3x3& I1 = manifold->inverseInertiaTensorBody1;
    Matrix3x3& I2 = manifold->inverseInertiaTensorBody2;


    // If we solve the friction constraints at the center of the contact manifold
    if (mIsSolveFrictionAtContactManifoldCenterActive)
    {
        manifold->normal = Vector3(0.0, 0.0, 0.0);
    }

    const Vector3& v1 = body1->mLinearVelocity;
    const Vector3& w1 = body1->mAngularVelocity;
    const Vector3& v2 = body2->mLinearVelocity;
    const Vector3& w2 = body2->mAngularVelocity;

    // For each contact point constraint
    for (uint i=0; i<manifold->nbContacts; i++)
    {

        ContactPointSolver& contactPoint = manifold->contacts[i];
        rpContactPoint* externalContact = contactPoint.externalContact;

        // Compute the velocity difference
          Vector3 deltaV = v2 + w2.cross(contactPoint.r2) -
                           v1 - w1.cross(contactPoint.r1);

        contactPoint.r1CrossN = contactPoint.r1.cross(contactPoint.normal);
        contactPoint.r2CrossN = contactPoint.r2.cross(contactPoint.normal);

        // Compute the inverse mass matrix K for the penetration constraint
        scalar massPenetration = manifold->massInverseBody1 +
                                 manifold->massInverseBody2 +
                                ((I1 * contactPoint.r1CrossN).cross(contactPoint.r1)).dot(contactPoint.normal) +
                                ((I2 * contactPoint.r2CrossN).cross(contactPoint.r2)).dot(contactPoint.normal);
        massPenetration > 0.0 ? contactPoint.inversePenetrationMass = scalar(1.0) / massPenetration : scalar(0.0);

        // If we do not solve the friction constraints at the center of the contact manifold
        if (!mIsSolveFrictionAtContactManifoldCenterActive)
        {

            // Compute the friction vectors
            computeFrictionVectors(deltaV, contactPoint);

            contactPoint.r1CrossT1 = contactPoint.r1.cross(contactPoint.frictionVector1);
            contactPoint.r1CrossT2 = contactPoint.r1.cross(contactPoint.frictionVector2);
            contactPoint.r2CrossT1 = contactPoint.r2.cross(contactPoint.frictionVector1);
            contactPoint.r2CrossT2 = contactPoint.r2.cross(contactPoint.frictionVector2);

            // Compute the inverse mass matrix K for the friction
            // constraints at each contact point
            scalar friction1Mass = manifold->massInverseBody1 +
                                   manifold->massInverseBody2 +
                                   ((I1 * contactPoint.r1CrossT1).cross(contactPoint.r1)).dot(contactPoint.frictionVector1) +
                                   ((I2 * contactPoint.r2CrossT1).cross(contactPoint.r2)).dot(contactPoint.frictionVector1);

            scalar friction2Mass = manifold->massInverseBody1 +
                                   manifold->massInverseBody2 +
                                  ((I1 * contactPoint.r1CrossT2).cross(contactPoint.r1)).dot(contactPoint.frictionVector2) +
                                  ((I2 * contactPoint.r2CrossT2).cross(contactPoint.r2)).dot(contactPoint.frictionVector2);


            friction1Mass > 0.0 ? contactPoint.inverseFriction1Mass = scalar(1.0) / friction1Mass : scalar(0.0);
            friction2Mass > 0.0 ? contactPoint.inverseFriction2Mass = scalar(1.0) / friction2Mass : scalar(0.0);

            scalar frictionTwistMass = contactPoint.normal.dot(manifold->inverseInertiaTensorBody1 * contactPoint.normal) +
                                       contactPoint.normal.dot(manifold->inverseInertiaTensorBody2 * contactPoint.normal);

            manifold->inverseTwistFrictionMass = scalar(1.0) / frictionTwistMass;
        }

        // Compute the restitution velocity bias "b". We compute this here instead
        // of inside the solve() method because we need to use the velocity difference
        // at the beginning of the contact. Note that if it is a resting contact (normal
        // velocity bellow a given threshold), we do not add a restitution velocity bias
        contactPoint.restitutionBias = 0.0;
        scalar deltaVDotN = deltaV.dot(contactPoint.normal);


        scalar damping =  RESTITUTION_VELOCITY_THRESHOLD * 2.0;
        scalar bounce  =  manifold->restitutionFactor;

        if ( deltaVDotN < -damping )
        {
            contactPoint.restitutionBias =  bounce * deltaVDotN;
        }

        // If the warm starting of the contact solver is active
        if (mIsWarmStartingActive)
        {
            //            // Get the cached accumulated impulses from the previous step
            //            contactPoint.AccumulatedPenetrationImpulse = externalContact->getPenetrationImpulse();
            //            contactPoint.AccumulatedFriction1Impulse = externalContact->getFrictionImpulse1();
            //            contactPoint.AccumulatedFriction2Impulse = externalContact->getFrictionImpulse2();
            //            contactPoint.AccumulatedRollingResistanceImpulse = externalContact->getRollingResistanceImpulse();

        }
        else
        {
            // Get the cached accumulated impulses from the previous step
            contactPoint.AccumulatedPenetrationImpulse = 0.0;
            contactPoint.AccumulatedFriction1Impulse = 0.0;
            contactPoint.AccumulatedFriction2Impulse = 0.0;
            contactPoint.AccumulatedRollingResistanceImpulse = Vector3::ZERO;
        }

        // Initialize the split impulses to zero
        contactPoint.AccumulatedPenetrationSplitImpulse = 0.0;
        contactPoint.AccumulatedRollingResistanceSplitImpulse = Vector3::ZERO;

        // If we solve the friction constraints at the center of the contact manifold
        if (mIsSolveFrictionAtContactManifoldCenterActive)
        {
            manifold->normal += contactPoint.normal;
        }

    }

    // Compute the inverse K matrix for the rolling resistance constraint
    manifold->inverseRollingResistance = Matrix3x3::zero();
    if (manifold->rollingResistanceFactor > 0 && (manifold->isBody1DynamicType || manifold->isBody2DynamicType))
    {
        manifold->inverseRollingResistance = manifold->inverseInertiaTensorBody1 + manifold->inverseInertiaTensorBody2;
        manifold->inverseRollingResistance = manifold->inverseRollingResistance.getInverse();
    }


    // If we solve the friction constraints at the center of the contact manifold
    if (mIsSolveFrictionAtContactManifoldCenterActive &&  manifold->nbContacts > 0)
    {

        manifold->normal.normalize();

        Vector3 deltaVFrictionPoint = v2 + w2.cross(manifold->r2Friction) -
                                      v1 - w1.cross(manifold->r1Friction);

        // Compute the friction vectors
        computeFrictionVectors(deltaVFrictionPoint, manifold);

        // Compute the inverse mass matrix K for the friction constraints at the center of
        // the contact manifold
        manifold->r1CrossT1 = manifold->r1Friction.cross(manifold->frictionVector1);
        manifold->r1CrossT2 = manifold->r1Friction.cross(manifold->frictionVector2);
        manifold->r2CrossT1 = manifold->r2Friction.cross(manifold->frictionVector1);
        manifold->r2CrossT2 = manifold->r2Friction.cross(manifold->frictionVector2);

        scalar friction1Mass = manifold->massInverseBody1 + manifold->massInverseBody2 +
                ((I1 * manifold->r1CrossT1).cross(manifold->r1Friction)).dot(manifold->frictionVector1) +
                ((I2 * manifold->r2CrossT1).cross(manifold->r2Friction)).dot(manifold->frictionVector1);

        scalar friction2Mass = manifold->massInverseBody1 + manifold->massInverseBody2 +
                ((I1 * manifold->r1CrossT2).cross(manifold->r1Friction)).dot(manifold->frictionVector2) +
                ((I2 * manifold->r2CrossT2).cross(manifold->r2Friction)).dot(manifold->frictionVector2);

        scalar frictionTwistMass = manifold->normal.dot(manifold->inverseInertiaTensorBody1 * manifold->normal) +
                                   manifold->normal.dot(manifold->inverseInertiaTensorBody2 * manifold->normal);

        friction1Mass > 0.0 ? manifold->inverseFriction1Mass = scalar(1.0)/friction1Mass : scalar(0.0);
        friction2Mass > 0.0 ? manifold->inverseFriction2Mass = scalar(1.0) / friction2Mass : scalar(0.0);
        frictionTwistMass > 0.0 ? manifold->inverseTwistFrictionMass = scalar(1.0) / frictionTwistMass : scalar(0.0);
    }

}



// Warm start the solver.
/// For each constraint, we apply the previous impulse (from the previous step)
/// at the beginning. With this technique, we will converge faster towards
/// the solution of the linear system
SIMD_INLINE void rpContactSolverSequentialImpulseObject::warmStart()
{


     if( mIsError ) return;
     //-------------------------------------------------------//


        rpRigidPhysicsBody*  body1 = (rpRigidPhysicsBody* )(mBody1);
        rpRigidPhysicsBody*  body2 = (rpRigidPhysicsBody* )(mBody2);


       atLeastOneRestingContactPoint = false;

       ContactManifoldSolver& contactManifold = *mContactConstraints;

        for (uint i = 0; i < contactManifold.nbContacts; ++i)
        {
            ContactPointSolver &contactPoint = contactManifold.contacts[i];
            rpContactPoint *cp = contactPoint.externalContact;


            Vector3 ContactPointA = (cp->getWorldPointOnBody1());
            Vector3 ContactPointB = (cp->getWorldPointOnBody2());


            scalar  &_accumulaterImpuls                        =  contactPoint.AccumulatedPenetrationImpulse;
            scalar  &_accumulaterImpulsFriction1               =  contactPoint.AccumulatedFriction1Impulse;
            scalar  &_accumulaterImpulsFriction2               =  contactPoint.AccumulatedFriction2Impulse;
            Vector3 &_accumulaterRollingResistanceImpulse      =  contactPoint.AccumulatedRollingResistanceImpulse;
            Vector3 &_accumulaterRollingResistanceSplitImpulse =  contactPoint.AccumulatedRollingResistanceSplitImpulse;


            /****************************************************************************/
            /// Penetration Distance
            scalar beta = mIsSplitImpulseActive ? BETA_SPLIT_IMPULSE : BETA;
            scalar sepp = -Abs( cp->getPenetrationDepth() );
            cp->setPenetrationDepth( -(beta/mTimeStep) * Min( scalar(0), sepp + SLOP ));
            /****************************************************************************/



            if(contactPoint.isRestingContact)
            {
                atLeastOneRestingContactPoint = true;

    //         // Project the old friction impulses (with old friction vectors) into
    //         // the new friction vectors to get the new friction impulses
                contactManifold.oldFrictionVector1 = contactManifold.frictionVector1;
                contactManifold.oldFrictionVector2 = contactManifold.frictionVector2;
                Vector3 oldFrictionImpulse = _accumulaterImpulsFriction1 * contactPoint.oldFrictionVector1 +
                                             _accumulaterImpulsFriction2 * contactPoint.oldFrictionVector2;

                _accumulaterImpulsFriction1 = oldFrictionImpulse.dot(contactPoint.frictionVector1);
                _accumulaterImpulsFriction2 = oldFrictionImpulse.dot(contactPoint.frictionVector2);

                //------------------------  accumulation impulse -----------------------------//
                body1->applyImpulse(-contactPoint.normal * _accumulaterImpuls , ContactPointA );
                body2->applyImpulse( contactPoint.normal * _accumulaterImpuls , ContactPointB );



                if (!mIsSolveFrictionAtContactManifoldCenterActive)
                {
                    //------------------------ friction accumulation impulse -----------------------------//

                    body1->applyImpulse(-contactPoint.frictionVector1 * _accumulaterImpulsFriction1 , ContactPointA);
                    body2->applyImpulse( contactPoint.frictionVector1 * _accumulaterImpulsFriction1 , ContactPointB);

                    body1->applyImpulse(-contactPoint.frictionVector2 * _accumulaterImpulsFriction2 , ContactPointA);
                    body2->applyImpulse( contactPoint.frictionVector2 * _accumulaterImpulsFriction2 , ContactPointB);



                    //------------------------------ Rolling resistance accumulate impulse --------------------------//

                    if (contactManifold.rollingResistanceFactor > 0)
                    {
                        body1->applyImpulseAngular(-_accumulaterRollingResistanceImpulse);
                        body2->applyImpulseAngular( _accumulaterRollingResistanceImpulse);


                        body1->applySplitImpulseAngular(-_accumulaterRollingResistanceSplitImpulse);
                        body2->applySplitImpulseAngular( _accumulaterRollingResistanceSplitImpulse);

                    }

                }

            }
            else
            {
                _accumulaterImpuls                        =  0;
                _accumulaterImpulsFriction1               =  0;
                _accumulaterImpulsFriction2               =  0;
                _accumulaterRollingResistanceImpulse      =  Vector3::ZERO;
                _accumulaterRollingResistanceSplitImpulse =  Vector3::ZERO;
                contactPoint.isRestingContact = true;

            }


        }



        // If we solve the friction constraints at the center of the contact manifold and there is
        // at least one resting contact point in the contact manifold
        if (mIsSolveFrictionAtContactManifoldCenterActive && atLeastOneRestingContactPoint && contactManifold.nbContacts > 0)
        {

            scalar  &_accumulaterImpulsFriction1               = contactManifold.AccumulatedFriction1Impulse;
            scalar  &_accumulaterImpulsFriction2               = contactManifold.AccumulatedFriction2Impulse;
            scalar  &_accumulatedFrictionTwistImpulse          = contactManifold.AccumulatedFrictionTwistImpulse;
            Vector3 &_accumulaterRollingResistanceImpulse      = contactManifold.AccumulatedRollingResistanceImpulse;
            Vector3 &_accumulaterRollingResistanceSplitImpulse = contactManifold.AccumulatedRollingResistanceSplitImpulse;

            // Project the old friction impulses (with old friction vectors) into the new friction
            // vectors to get the new friction impulses
            contactManifold.oldFrictionVector1 = contactManifold.frictionVector1;
            contactManifold.oldFrictionVector2 = contactManifold.frictionVector2;
            Vector3 oldFrictionImpulse =_accumulaterImpulsFriction1 * contactManifold.oldFrictionVector1 +
                                        _accumulaterImpulsFriction2 * contactManifold.oldFrictionVector2;
            _accumulaterImpulsFriction1  = oldFrictionImpulse.dot(contactManifold.frictionVector1);
            _accumulaterImpulsFriction2  = oldFrictionImpulse.dot(contactManifold.frictionVector2);


            /**

            /// ------ First friction constraint at the center of the contact manifold ------ ///
            body1->applyImpulse(-contactManifold.frictionVector1 * _accumulaterImpulsFriction1 , contactManifold.frictionPointBody1 );
            body2->applyImpulse( contactManifold.frictionVector1 * _accumulaterImpulsFriction1 , contactManifold.frictionPointBody2 );


            /// ------ Second friction constraint at the center of the contact manifold ----- ///
            body1->applyImpulse(-contactManifold.frictionVector2 *  _accumulaterImpulsFriction2 , contactManifold.frictionPointBody1 );
            body2->applyImpulse( contactManifold.frictionVector2 *  _accumulaterImpulsFriction2 , contactManifold.frictionPointBody2 );



            /**/
            // ------ First friction constraint at the center of the contact manifold ------ //

            // Compute the impulse P = J^T * lambda
            Vector3 linearImpulseBody1  = -contactManifold.frictionVector1 *_accumulaterImpulsFriction1;
            Vector3 angularImpulseBody1 = -contactManifold.r1CrossT1 *_accumulaterImpulsFriction1;
            Vector3 linearImpulseBody2  =  contactManifold.frictionVector1 *_accumulaterImpulsFriction1;
            Vector3 angularImpulseBody2 =  contactManifold.r2CrossT1 *_accumulaterImpulsFriction1;


            body1->applyImpulseLinear(linearImpulseBody1);
            body1->applyImpulseAngular(angularImpulseBody1);

            body2->applyImpulseLinear(linearImpulseBody2);
            body2->applyImpulseAngular(angularImpulseBody2);


            // ------ Second friction constraint at the center of the contact manifold ----- //

            // Compute the impulse P = J^T * lambda
            linearImpulseBody1  = -contactManifold.frictionVector2 * _accumulaterImpulsFriction2;
            angularImpulseBody1 = -contactManifold.r1CrossT2 * _accumulaterImpulsFriction2;
            linearImpulseBody2  =  contactManifold.frictionVector2 * _accumulaterImpulsFriction2;
            angularImpulseBody2 =  contactManifold.r2CrossT2 * _accumulaterImpulsFriction2;



            body1->applyImpulseLinear(linearImpulseBody1);
            body1->applyImpulseAngular(angularImpulseBody1);

            body2->applyImpulseLinear(linearImpulseBody2);
            body2->applyImpulseAngular(angularImpulseBody2);

             /**/



            // ------ Twist friction constraint at the center of the contact manifold ------ //

            // Compute the impulse P = J^T * lambda
            angularImpulseBody1 = -contactManifold.normal * _accumulatedFrictionTwistImpulse;
            angularImpulseBody2 =  contactManifold.normal * _accumulatedFrictionTwistImpulse;


            body1->applyImpulseAngular(angularImpulseBody1);
            body2->applyImpulseAngular(angularImpulseBody2);

            // ------ Rolling resistance at the center of the contact manifold ------ //

            // Compute the impulse P = J^T * lambda
            angularImpulseBody1 = -_accumulaterRollingResistanceImpulse;
            angularImpulseBody2 =  _accumulaterRollingResistanceImpulse;


            body1->applyImpulseAngular(angularImpulseBody1);
            body2->applyImpulseAngular(angularImpulseBody2);



             // ------ Split Rolling resistance at the center of the contact manifold ------ /

            // Compute the impulse P = J^T * lambda
            Vector3 angularSplitImpulseBody1 = -_accumulaterRollingResistanceSplitImpulse;
            Vector3 angularSplitImpulseBody2 =  _accumulaterRollingResistanceSplitImpulse;


            body1->applySplitImpulseAngular(angularSplitImpulseBody1);
            body2->applySplitImpulseAngular(angularSplitImpulseBody2);

        }
        else
        {  // If it is a new contact manifold

            // Initialize the accumulated impulses to zero
            contactManifold.AccumulatedFriction1Impulse = 0.0;
            contactManifold.AccumulatedFriction2Impulse = 0.0;
            contactManifold.AccumulatedFrictionTwistImpulse = 0.0;
            contactManifold.AccumulatedRollingResistanceImpulse = Vector3::ZERO;
            contactManifold.AccumulatedRollingResistanceSplitImpulse = Vector3::ZERO;
            atLeastOneRestingContactPoint = true;
        }






}

SIMD_INLINE void rpContactSolverSequentialImpulseObject::solveVelocityConstraint()
{

    if( mIsError ) return;

    //-------------------------------------------------------//


    rpRigidPhysicsBody*  body1 = (rpRigidPhysicsBody* )(mBody1);
    rpRigidPhysicsBody*  body2 = (rpRigidPhysicsBody* )(mBody2);


    ContactManifoldSolver* contactManifold = mContactConstraints;


    scalar deltaLambda;
    scalar lambdaTemp;

    scalar sumPenetrationImpulse = 0.0;
    // Get the constrained velocities
    const Vector3& v1 = body1->mLinearVelocity;
    const Vector3& w1 = body1->mAngularVelocity;
    const Vector3& v2 = body2->mLinearVelocity;
    const Vector3& w2 = body2->mAngularVelocity;

   for (uint i = 0; i < contactManifold->nbContacts; ++i)
   {

        ContactPointSolver &contactPoint = contactManifold->contacts[i];
        rpContactPoint *cp = contactPoint.externalContact;


                       // --------- Penetration --------- //
       //CContactCollision *c = &m_contacts[i];
       Vector3 ContactPointA = (cp->getWorldPointOnBody1());
       Vector3 ContactPointB = (cp->getWorldPointOnBody2());


       scalar  &_accumulaterImpuls                   = contactPoint.AccumulatedPenetrationImpulse;
       scalar  &_accumulaterImpulsFriction1          = contactPoint.AccumulatedFriction1Impulse;
       scalar  &_accumulaterImpulsFriction2          = contactPoint.AccumulatedFriction2Impulse;
       Vector3 &_accumulaterRollingResistanceImpulse = contactPoint.AccumulatedRollingResistanceImpulse;



       // Compute J*v
       Vector3 deltaV = v2 + w2.cross(contactPoint.r2) -
                        v1 - w1.cross(contactPoint.r1);
       scalar deltaVDotN = deltaV.dot(contactPoint.normal);
       scalar Jv = deltaVDotN;



       // Compute the bias "b" of the constraint
       //scalar beta = mIsSplitImpulseActive ? BETA_SPLIT_IMPULSE : BETA;
       scalar biasPenetrationDepth = 0.0;
       if (contactPoint.penetrationDepth > SLOP)
       {
          // biasPenetrationDepth = -(beta/mTimeStep) * max(0.0f, scalar(contactPoint.penetrationDepth - SLOP));
           biasPenetrationDepth = cp->getPenetrationDepth();
       }

       scalar b = biasPenetrationDepth + contactPoint.restitutionBias;

       if(mIsSplitImpulseActive)
       {
           deltaLambda = -(Jv + contactPoint.restitutionBias) * contactPoint.inversePenetrationMass;
       }
       else
       {
           deltaLambda = -(Jv + b) * contactPoint.inversePenetrationMass;
       }



       lambdaTemp = _accumulaterImpuls;
       _accumulaterImpuls = Max(_accumulaterImpuls + deltaLambda, scalar(0.0));
       deltaLambda = _accumulaterImpuls - lambdaTemp;


       body1->applyImpulse(-contactPoint.normal * deltaLambda , ContactPointA);
       body2->applyImpulse( contactPoint.normal * deltaLambda , ContactPointB);



       sumPenetrationImpulse += _accumulaterImpuls;




       // If we do not solve the friction constraints at the center of the contact manifold
       if (!mIsSolveFrictionAtContactManifoldCenterActive)
       {

           // --------- Friction 1 --------- //

           // Compute J*v
           deltaV = v2 + w2.cross(contactPoint.r2) -
                    v1 - w1.cross(contactPoint.r1);
           Jv = deltaV.dot(contactPoint.frictionVector1);

           // Compute the Lagrange multiplier lambda
           deltaLambda = -Jv;
           deltaLambda *= contactPoint.inverseFriction1Mass;
           scalar frictionLimit = contactManifold->frictionCoefficient * _accumulaterImpuls;
           lambdaTemp = _accumulaterImpulsFriction1;
           _accumulaterImpulsFriction1 = Max(-frictionLimit, Min(_accumulaterImpulsFriction1 + deltaLambda, frictionLimit));
           deltaLambda = _accumulaterImpulsFriction1 - lambdaTemp;


           body1->applyImpulse(-contactPoint.frictionVector1 * deltaLambda , ContactPointA );
           body2->applyImpulse( contactPoint.frictionVector1 * deltaLambda , ContactPointB );



           // --------- Friction 2 --------- //

           // Compute J*v
           deltaV = v2 + w2.cross(contactPoint.r2) -
                    v1 - w1.cross(contactPoint.r1);
           Jv = deltaV.dot(contactPoint.frictionVector2);


           // Compute the Lagrange multiplier lambda
           deltaLambda = -Jv;
           deltaLambda *= contactPoint.inverseFriction2Mass;
           frictionLimit = contactManifold->frictionCoefficient * _accumulaterImpuls;
           lambdaTemp = _accumulaterImpulsFriction2;
           _accumulaterImpulsFriction2 = Max(-frictionLimit, Min(_accumulaterImpulsFriction2 + deltaLambda, frictionLimit));
           deltaLambda = _accumulaterImpulsFriction2 - lambdaTemp;


           /// Apply impulse velocity
           body1->applyImpulse(-contactPoint.frictionVector2 * deltaLambda , ContactPointA );
           body2->applyImpulse( contactPoint.frictionVector2 * deltaLambda , ContactPointB );



           // --------- Rolling resistance constraint --------- //
           if ( !mIsSolveFrictionAtContactManifoldCenterActive && contactManifold->rollingResistanceFactor > 0)
           {

               // Compute J*v
               const Vector3 JvRolling = w2 - w1;

               // Compute the Lagrange multiplier lambda
               Vector3 deltaLambdaRolling = contactManifold->inverseRollingResistance * (-JvRolling);
               scalar rollingLimit = contactManifold->rollingResistanceFactor * _accumulaterImpuls;
               Vector3 lambdaTempRolling = _accumulaterRollingResistanceImpulse;
               _accumulaterRollingResistanceImpulse = Vector3::clamp(_accumulaterRollingResistanceImpulse + deltaLambdaRolling, rollingLimit);
               deltaLambdaRolling = _accumulaterRollingResistanceImpulse - lambdaTempRolling;



               body1->applyImpulseAngular( (-deltaLambdaRolling));
               body2->applyImpulseAngular( ( deltaLambdaRolling));
           }

       }

    }



   // If we solve the friction constraints at the center of the contact manifold
   if (mIsSolveFrictionAtContactManifoldCenterActive && contactManifold->nbContacts > 0)
   {


       scalar  &_accumulaterImpulsFriction1          = contactManifold->AccumulatedFriction1Impulse;
       scalar  &_accumulaterImpulsFriction2          = contactManifold->AccumulatedFriction2Impulse;
       scalar  &_accumulatedFrictionTwistImpulse     = contactManifold->AccumulatedFrictionTwistImpulse;
       Vector3 &_accumulaterRollingResistanceImpulse = contactManifold->AccumulatedRollingResistanceImpulse;

       // ------ First friction constraint at the center of the contact manifol ------ //

       // Compute J*v
       Vector3 deltaV = v2 + w2.cross(contactManifold->r2Friction)
                      - v1 - w1.cross(contactManifold->r1Friction);
       scalar Jv = deltaV.dot(contactManifold->frictionVector1);

       // Compute the Lagrange multiplier lambda
       scalar deltaLambda = -Jv * contactManifold->inverseFriction1Mass;
       scalar frictionLimit = contactManifold->frictionCoefficient * sumPenetrationImpulse;
       lambdaTemp = _accumulaterImpulsFriction1;
       _accumulaterImpulsFriction1 = Max(-frictionLimit, Min(_accumulaterImpulsFriction1 + deltaLambda, frictionLimit));
       deltaLambda = _accumulaterImpulsFriction1 - lambdaTemp;


       /**

       // Compute the impulse P=J^T * lambda
       body1->applyImpulse(-contactManifold->frictionVector1 * deltaLambda , contactManifold->frictionPointBody1 );
       body2->applyImpulse( contactManifold->frictionVector1 * deltaLambda , contactManifold->frictionPointBody2 );


       /**/
       // Compute the impulse P=J^T * lambda
       Vector3 linearImpulseBody1  = -contactManifold->frictionVector1 * deltaLambda;
       Vector3 angularImpulseBody1 = -contactManifold->r1CrossT1       * deltaLambda;
       Vector3 linearImpulseBody2  =  contactManifold->frictionVector1 * deltaLambda;
       Vector3 angularImpulseBody2 =  contactManifold->r2CrossT1       * deltaLambda;


       // Apply the impulses to the bodies of the constraint
       body1->applyImpulseLinear(linearImpulseBody1);
       body1->applyImpulseAngular(angularImpulseBody1);

       body2->applyImpulseLinear(linearImpulseBody2);
       body2->applyImpulseAngular(angularImpulseBody2);

       /**/


       // ------ Second friction constraint at the center of the contact manifol ----- //

       // Compute J*v
       deltaV = v2 + w2.cross(contactManifold->r2Friction)
              - v1 - w1.cross(contactManifold->r1Friction);
       Jv = deltaV.dot(contactManifold->frictionVector2);

       // Compute the Lagrange multiplier lambda
       deltaLambda = -Jv * contactManifold->inverseFriction2Mass;
       frictionLimit = contactManifold->frictionCoefficient * sumPenetrationImpulse;
       lambdaTemp = _accumulaterImpulsFriction2;
       _accumulaterImpulsFriction2 = Max(-frictionLimit, Min(_accumulaterImpulsFriction2 + deltaLambda, frictionLimit));
       deltaLambda = _accumulaterImpulsFriction2 - lambdaTemp;


       /**

       // Compute the impulse P=J^T * lambda
       body1->applyImpulse(-contactManifold->frictionVector2 * deltaLambda , contactManifold->frictionPointBody1 );
       body2->applyImpulse( contactManifold->frictionVector2 * deltaLambda , contactManifold->frictionPointBody2 );


       /**/

       // Compute the impulse P=J^T * lambda
       linearImpulseBody1  = -contactManifold->frictionVector2 * deltaLambda;
       angularImpulseBody1 = -contactManifold->r1CrossT2 * deltaLambda;
       linearImpulseBody2  =  contactManifold->frictionVector2 * deltaLambda;
       angularImpulseBody2 =  contactManifold->r2CrossT2 * deltaLambda;



       // Apply the impulses to the body 1  of the constraint
       body1->applyImpulseLinear(linearImpulseBody1);
       body1->applyImpulseAngular(angularImpulseBody1);

       // Apply the impulses to the body 2 of the constraint
       body2->applyImpulseLinear(linearImpulseBody2);
       body2->applyImpulseAngular(angularImpulseBody2);

       /**/


       // ------ Twist friction constraint at the center of the contact manifol ------ //

       // Compute J*v
       deltaV = w2 - w1;
       Jv = deltaV.dot(contactManifold->normal);

       deltaLambda = -Jv * (contactManifold->inverseTwistFrictionMass);
       frictionLimit = contactManifold->frictionCoefficient * sumPenetrationImpulse;
       lambdaTemp = _accumulatedFrictionTwistImpulse;
       _accumulatedFrictionTwistImpulse = Max(-frictionLimit, Min(_accumulatedFrictionTwistImpulse + deltaLambda, frictionLimit));
       deltaLambda = _accumulatedFrictionTwistImpulse - lambdaTemp;

       // Compute the impulse P=J^T * lambda
       linearImpulseBody1  =  Vector3::ZERO;
       angularImpulseBody1 = -contactManifold->normal * deltaLambda;
       linearImpulseBody2  =  Vector3::ZERO;
       angularImpulseBody2 =  contactManifold->normal * deltaLambda;



       // Apply the impulses to the bodies of the constraint
       body1->applyImpulseAngular(angularImpulseBody1);
       body2->applyImpulseAngular(angularImpulseBody2);


       /**/
       // --------- Rolling resistance constraint at the center of the contact manifold --------- //
       if (contactManifold->rollingResistanceFactor > 0)
       {

           // Compute J*v
           const Vector3 JvRolling = w2 - w1;

           // Compute the Lagrange multiplier lambda
           Vector3 deltaLambdaRolling = contactManifold->inverseRollingResistance * (-JvRolling);
           scalar rollingLimit = contactManifold->rollingResistanceFactor * sumPenetrationImpulse;
           Vector3 lambdaTempRolling = _accumulaterRollingResistanceImpulse;
           _accumulaterRollingResistanceImpulse = Vector3::clamp(_accumulaterRollingResistanceImpulse + deltaLambdaRolling, rollingLimit);
           deltaLambdaRolling = _accumulaterRollingResistanceImpulse - lambdaTempRolling;

           // Compute the impulse P=J^T * lambda
           angularImpulseBody1 = -deltaLambdaRolling;
           angularImpulseBody2 =  deltaLambdaRolling;


           // Apply the impulses to the bodies of the constraint
           body1->applyImpulseAngular(angularImpulseBody1);
           body2->applyImpulseAngular(angularImpulseBody2);

       }
       /**/
   }


}



SIMD_INLINE void rpContactSolverSequentialImpulseObject::solvePositionConstraint()
{


    if( mIsError ) return;
    //-------------------------------------------------------//


    rpRigidPhysicsBody*  body1 = (rpRigidPhysicsBody* )(mBody1);
    rpRigidPhysicsBody*  body2 = (rpRigidPhysicsBody* )(mBody2);


    ContactManifoldSolver* contactManifold = mContactConstraints;



    // Get the constrained velocities
    const Vector3& v1Split = body1->mSplitLinearVelocity;
    const Vector3& w1Split = body1->mSplitAngularVelocity;
    const Vector3& v2Split = body2->mSplitLinearVelocity;
    const Vector3& w2Split = body2->mSplitAngularVelocity;


    scalar sumPenetrationSplitImpulse = 0.0;

    for (uint i = 0; i < contactManifold->nbContacts; ++i)
    {

        ContactPointSolver &contactPoint = contactManifold->contacts[i];
        rpContactPoint *cp = contactPoint.externalContact;


        scalar  &_accumulaterPenetrationSplit = contactPoint.AccumulatedPenetrationSplitImpulse;
        Vector3 &_accumulaterRollingResistanceSplitImpulse = contactPoint.AccumulatedRollingResistanceSplitImpulse;


        // --------- Penetration --------- //
        Vector3 ContactPointA = (cp->getWorldPointOnBody1());
        Vector3 ContactPointB = (cp->getWorldPointOnBody2());

        Vector3 deltaVSplit = v2Split + w2Split.cross(contactPoint.r2) -
                              v1Split - w1Split.cross(contactPoint.r1);
        scalar JvSplit = deltaVSplit.dot(contactPoint.normal);


        scalar biasPenetrationDepth = -cp->getPenetrationDepth();
        scalar deltaLambdaSplit = -(JvSplit + biasPenetrationDepth) * contactPoint.inversePenetrationMass;

        scalar lambdaTempSplit = _accumulaterPenetrationSplit;
        _accumulaterPenetrationSplit = Max( _accumulaterPenetrationSplit + deltaLambdaSplit, scalar(0.0));
        scalar deltaLambda = _accumulaterPenetrationSplit - lambdaTempSplit;


        sumPenetrationSplitImpulse += _accumulaterPenetrationSplit;


        body1->applySplitImpulse(-contactPoint.normal * deltaLambda , ContactPointA );
        body2->applySplitImpulse( contactPoint.normal * deltaLambda , ContactPointB );


        /**/
        if( !mIsSolveFrictionAtContactManifoldCenterActive && contactManifold->rollingResistanceFactor > 0)
        {

            // Compute J*v
            const Vector3 JvRolling = w2Split  - w1Split;

            // Compute the Lagrange multiplier lambda
            Vector3 deltaLambdaRolling = contactManifold->inverseRollingResistance * (-JvRolling);
            scalar rollingLimit = contactManifold->rollingResistanceFactor * _accumulaterPenetrationSplit;
            Vector3 lambdaTempRolling = _accumulaterRollingResistanceSplitImpulse;
            _accumulaterRollingResistanceSplitImpulse = Vector3::clamp(_accumulaterRollingResistanceSplitImpulse + deltaLambdaRolling, rollingLimit);
            deltaLambdaRolling = _accumulaterRollingResistanceSplitImpulse - lambdaTempRolling;


            ///Apply the pseudo impulses
            body1->applySplitImpulseAngular(-deltaLambdaRolling);
            body2->applySplitImpulseAngular( deltaLambdaRolling);


        }
        /**/


    }


    /**/
    // If we solve the friction constraints at the center of the contact manifold
    if (mIsSolveFrictionAtContactManifoldCenterActive && contactManifold->nbContacts > 0)
    {

        Vector3 &_accumulaterRollingResistanceSplitImpulse = contactManifold->AccumulatedRollingResistanceSplitImpulse;

        //--------- Rolling resistance constraint at the center of the contact manifold ---------//
        if (contactManifold->rollingResistanceFactor > 0)
        {

            // Compute J*v
            const Vector3 JvSplitRolling = w2Split - w1Split;

            // Compute the Lagrange multiplier lambda
            Vector3 deltaLambdaRolling = contactManifold->inverseRollingResistance * (-JvSplitRolling);
            scalar rollingLimit = contactManifold->rollingResistanceFactor * sumPenetrationSplitImpulse;
            Vector3 lambdaTempRolling = _accumulaterRollingResistanceSplitImpulse;
            _accumulaterRollingResistanceSplitImpulse = Vector3::clamp(_accumulaterRollingResistanceSplitImpulse + deltaLambdaRolling, rollingLimit);
            deltaLambdaRolling = _accumulaterRollingResistanceSplitImpulse - lambdaTempRolling;


            // Apply the impulses to the bodies of the constraint
            body1->applySplitImpulseAngular(-deltaLambdaRolling);
            body2->applySplitImpulseAngular( deltaLambdaRolling);

        }
    }
    /**/

}

/**/
SIMD_INLINE void rpContactSolverSequentialImpulseObject::storeImpulses()
{


        ContactManifoldSolver& manifold = *mContactConstraints;

        for (uint i=0; i<manifold.nbContacts; i++)
        {

            ContactPointSolver& contactPoint = manifold.contacts[i];

            contactPoint.externalContact->setPenetrationImpulse(contactPoint.AccumulatedPenetrationImpulse);
            contactPoint.externalContact->setFrictionImpulse1(contactPoint.AccumulatedFriction1Impulse);
            contactPoint.externalContact->setFrictionImpulse2(contactPoint.AccumulatedFriction1Impulse);
            contactPoint.externalContact->setRollingResistanceImpulse(contactPoint.AccumulatedRollingResistanceSplitImpulse);
            contactPoint.externalContact->setFrictionVector1(contactPoint.frictionVector1);
            contactPoint.externalContact->setFrictionVector2(contactPoint.frictionVector2);

        }

        manifold.externalContactManifold->setFrictionImpulse1(manifold.AccumulatedFriction1Impulse);
        manifold.externalContactManifold->setFrictionImpulse2(manifold.AccumulatedFriction2Impulse);
        manifold.externalContactManifold->setFrictionTwistImpulse(manifold.AccumulatedFrictionTwistImpulse);
        manifold.externalContactManifold->setRollingResistanceImpulse(manifold.AccumulatedRollingResistanceImpulse);
        manifold.externalContactManifold->setFrictionVector1(manifold.frictionVector1);
        manifold.externalContactManifold->setFrictionVector2(manifold.frictionVector2);

}
/**/


/**********************************************************************************************/
SIMD_INLINE scalar rpContactSolverSequentialImpulseObject::computeMixedRestitutionFactor( rpRigidPhysicsBody*  body1 ,
                                                                                          rpRigidPhysicsBody*  body2) const
{
    scalar restitution1 = body1->getMaterial().getBounciness();
    scalar restitution2 = body2->getMaterial().getBounciness();

    // Return the largest restitution factor
    return (restitution1 > restitution2) ? restitution1 : restitution2;
}

SIMD_INLINE scalar rpContactSolverSequentialImpulseObject::computeMixedFrictionCoefficient( rpRigidPhysicsBody*  body1 ,
                                                                                            rpRigidPhysicsBody*  body2) const
{
    // Use the geometric mean to compute the mixed friction coefficient
    return SquareRoot(body1->getMaterial().getFrictionCoefficient() *
                      body2->getMaterial().getFrictionCoefficient());
}

SIMD_INLINE scalar rpContactSolverSequentialImpulseObject::computeMixedRollingResistance( rpRigidPhysicsBody*  body1 ,
                                                                                          rpRigidPhysicsBody*  body2) const
{
    return scalar(0.5f) * (body1->getMaterial().getRollingResistance() +
                           body2->getMaterial().getRollingResistance());
}



SIMD_INLINE void rpContactSolverSequentialImpulseObject::computeFrictionVectors( const Vector3& deltaVelocity, ContactPointSolver& contactPoint) const
{
    assert(contactPoint.normal.length() > 0.0);


    // Compute the velocity difference vector in the tangential plane
    Vector3 normalVelocity  = deltaVelocity.dot(contactPoint.normal) * contactPoint.normal;
    Vector3 tangentVelocity = deltaVelocity - normalVelocity;

    // If the velocty difference in the tangential plane is not zero
    scalar lengthTangenVelocity = tangentVelocity.length();
    if (lengthTangenVelocity > MACHINE_EPSILON)
    {
        // Compute the first friction vector in the direction of the tangent
        // velocity difference
        contactPoint.frictionVector1 = tangentVelocity / lengthTangenVelocity;
    }
    else
    {
        // Get any orthogonal vector to the normal as the first friction vector
        contactPoint.frictionVector1 = contactPoint.normal.getOneUnitOrthogonalVector();
    }


    // The second friction vector is computed by the cross product of the firs
    // friction vector and the contact normal
    contactPoint.frictionVector2 =contactPoint.normal.cross(contactPoint.frictionVector1).getUnit();

    /**/


    /// the Normalize  to Vector
    contactPoint.frictionVector1.normalize();
    contactPoint.frictionVector2.normalize();



}



SIMD_INLINE void rpContactSolverSequentialImpulseObject::computeFrictionVectors( const Vector3& deltaVelocity , ContactManifoldSolver* contact ) const
{
    assert(contact->normal.length() > 0.0);

    /**/
    // Compute the velocity difference vector in the tangential plane
    Vector3 normalVelocity = deltaVelocity.dot(contact->normal) * contact->normal;
    Vector3 tangentVelocity = deltaVelocity - normalVelocity;

    // If the velocty difference in the tangential plane is not zero
    scalar lengthTangenVelocity = tangentVelocity.length();
    if (lengthTangenVelocity > MACHINE_EPSILON)
    {
        // Compute the first friction vector in the direction of the tangent
        // velocity difference
        contact->frictionVector1 = tangentVelocity / lengthTangenVelocity;
    }
    else
    {
        // Get any orthogonal vector to the normal as the first friction vector
        contact->frictionVector1 = contact->normal.getOneUnitOrthogonalVector();
    }

    // The second friction vector is computed by the cross product of the firs
    // friction vector and the contact normal
    contact->frictionVector2 = contact->normal.cross(contact->frictionVector1).getUnit();

    /**/


    /// the Normalize  to Vector
    contact->frictionVector1.normalize();
    contact->frictionVector2.normalize();



}


/**********************************************************************************************/

//scalar rpContactSolverSequentialImpulseObject::CalcualteImpuls( const ContactPointSolver& contactPoint, const Vector3& normal)
//{
//	   Vector3 n1 =     -normal;
//	   Vector3 w1 =      normal ^ contactPoint.r1;
//	   Vector3 n2 =      normal;
//	   Vector3 w2 =     -normal ^ contactPoint.r2;
//
//	   scalar Impulse =    n1.dot(mBody1->getLinearVelocity())
//	                     + w1.dot(mBody1->getAngularVelocity())
//	                     + n2.dot(mBody2->getLinearVelocity())
//	                     + w2.dot(mBody2->getAngularVelocity());
//
//
//	    return Impulse;
//}
//
//
//scalar rpContactSolverSequentialImpulseObject::CalcualteSplitImpuls( const ContactPointSolver& contactPoint, const Vector3& normal )
//{
//
//	Vector3 n1 =     -normal;
//	Vector3 w1 =      normal ^ contactPoint.r1;
//	Vector3 n2 =      normal;
//	Vector3 w2 =     -normal ^ contactPoint.r2;
//	  //
//	scalar Impulse =    n1.dot(mBody1->mSplitLinearVelocity)
//	                  + w1.dot(mBody1->mSplitAngularVelocity)
//	                  + n2.dot(mBody2->mSplitLinearVelocity)
//	                  + w2.dot(mBody2->mSplitAngularVelocity);
//
//
//	     return Impulse;
//}



} /* namespace real_physics */


