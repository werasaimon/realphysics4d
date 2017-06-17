#include "rpContactSolverSequential.h"


namespace real_physics
{

const scalar OFFSET_SLOPE = scalar(0.033);



// Constants initialization
const scalar rpContactSolverSequential::BETA = scalar(0.2);
const scalar rpContactSolverSequential::BETA_SPLIT_IMPULSE = scalar(0.2);
const scalar rpContactSolverSequential::SLOP= scalar(0.01);

rpContactSolverSequential::rpContactSolverSequential()
{

}

void rpContactSolverSequential::initializeForIsland(scalar dt, rpIsland *island)
{


    assert(island != NULL);
    assert(island->getNbBodies() > 0);
    assert(island->getNbContactManifolds() > 0);
   // assert(mSplitLinearVelocities != NULL);
   // assert(mSplitAngularVelocities != NULL);

    // Set the current time step
    mTimeStep = dt;

    mNbContactManifolds = island->getNbContactManifolds();

    mContactConstraints = new ContactManifoldSolver[mNbContactManifolds];
    assert(mContactConstraints != NULL);

    // For each contact manifold of the island
    rpContactManifold** contactManifolds = island->getContactManifold();
    for (uint i=0; i<mNbContactManifolds; i++)
    {
        rpContactManifold* externalManifold = contactManifolds[i];

        rpRigidPhysicsBody*  body1 = static_cast<rpRigidPhysicsBody*>(externalManifold->getBody1());
        rpRigidPhysicsBody*  body2 = static_cast<rpRigidPhysicsBody*>(externalManifold->getBody2());


        ContactManifoldSolver* internalManifold = &mContactConstraints[i];



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
            internalManifold->frictionPointBody1 = Vector3::zero();
            internalManifold->frictionPointBody2 = Vector3::zero();
        }



        // For each  contact point of the contact manifold
        for (uint c=0; c<externalManifold->getNbContactPoints(); c++)
        {


            ContactPointSolver& contactPoint = internalManifold->contacts[c];
            // Get a contact point
            rpContactPoint* externalContact = externalManifold->getContactPoint(c);


            externalContact->setWorldPointOnBody1(externalContact->getLocalPointOnBody1());
            externalContact->setWorldPointOnBody2(externalContact->getLocalPointOnBody2());


            // Get the contact point on the two bodies
            Vector3 p1 = externalContact->getWorldPointOnBody1();
            Vector3 p2 = externalContact->getWorldPointOnBody2();

            contactPoint.externalContact = externalContact;
            contactPoint.normal = -externalContact->getNormal();
            contactPoint.r1 = p1 - x1;
            contactPoint.r2 = p2 - x2;
            contactPoint.penetrationDepth = externalContact->getPenetrationDepth();
            contactPoint.isRestingContact = externalContact->getIsRestingContact();
            externalContact->setIsRestingContact(true);
            contactPoint.oldFrictionVector1 = externalContact->getFrictionVector1();
            contactPoint.oldFrictionVector2 = externalContact->getFrictionVector2();


            // Get the cached accumulated impulses from the previous step
            contactPoint.AccumulatedPenetrationImpulse = 0.0;
            contactPoint.AccumulatedFriction1Impulse = 0.0;
            contactPoint.AccumulatedFriction2Impulse = 0.0;
            contactPoint.AccumulatedRollingResistanceImpulse = Vector3::ZERO;


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

                // Initialize the accumulated impulses with the previous step accumulated impulses
                internalManifold->AccumulatedFriction1Impulse = externalManifold->getFrictionImpulse1();
                internalManifold->AccumulatedFriction2Impulse = externalManifold->getFrictionImpulse2();
                internalManifold->AccumulatedFrictionTwistImpulse = externalManifold->getFrictionTwistImpulse();
            }
            else
            {

                // Initialize the accumulated impulses to zero
                internalManifold->AccumulatedFriction1Impulse = 0.0;
                internalManifold->AccumulatedFriction2Impulse = 0.0;
                internalManifold->AccumulatedFrictionTwistImpulse = 0.0;
                internalManifold->AccumulatedRollingResistanceImpulse = Vector3(0, 0, 0);
            }
        }

    }


    initializeContactConstraints();



//        ContactManifold* externalManifold = contactManifolds[i];

//        ContactManifoldSolver& internalManifold = mContactConstraints[i];

//        assert(externalManifold->getNbContactPoints() > 0);

//        // Get the two bodies of the contact
//        RigidBody* body1 = static_cast<RigidBody*>(externalManifold->getContactPoint(0)->getBody1());
//        RigidBody* body2 = static_cast<RigidBody*>(externalManifold->getContactPoint(0)->getBody2());
//        assert(body1 != NULL);
//        assert(body2 != NULL);

//        // Get the position of the two bodies
//        const Vector3& x1 = body1->mCenterOfMassWorld;
//        const Vector3& x2 = body2->mCenterOfMassWorld;

//        // Initialize the internal contact manifold structure using the external
//        // contact manifold
//        internalManifold.indexBody1 = mMapBodyToConstrainedVelocityIndex.find(body1)->second;
//        internalManifold.indexBody2 = mMapBodyToConstrainedVelocityIndex.find(body2)->second;
//        internalManifold.inverseInertiaTensorBody1 = body1->getInertiaTensorInverseWorld();
//        internalManifold.inverseInertiaTensorBody2 = body2->getInertiaTensorInverseWorld();
//        internalManifold.massInverseBody1 = body1->mMassInverse;
//        internalManifold.massInverseBody2 = body2->mMassInverse;
//        internalManifold.nbContacts = externalManifold->getNbContactPoints();
//        internalManifold.restitutionFactor = computeMixedRestitutionFactor(body1, body2);
//        internalManifold.frictionCoefficient = computeMixedFrictionCoefficient(body1, body2);
//        internalManifold.rollingResistanceFactor = computeMixedRollingResistance(body1, body2);
//        internalManifold.externalContactManifold = externalManifold;
//        internalManifold.isBody1DynamicType = body1->getType() == DYNAMIC;
//        internalManifold.isBody2DynamicType = body2->getType() == DYNAMIC;

//        // If we solve the friction constraints at the center of the contact manifold
//        if (mIsSolveFrictionAtContactManifoldCenterActive) {
//            internalManifold.frictionPointBody1 = Vector3::zero();
//            internalManifold.frictionPointBody2 = Vector3::zero();
//        }

//        // For each  contact point of the contact manifold
//        for (uint c=0; c<externalManifold->getNbContactPoints(); c++) {

//            ContactPointSolver& contactPoint = internalManifold.contacts[c];

//            // Get a contact point
//            ContactPoint* externalContact = externalManifold->getContactPoint(c);

//            // Get the contact point on the two bodies
//            Vector3 p1 = externalContact->getWorldPointOnBody1();
//            Vector3 p2 = externalContact->getWorldPointOnBody2();

//            contactPoint.externalContact = externalContact;
//            contactPoint.normal = externalContact->getNormal();
//            contactPoint.r1 = p1 - x1;
//            contactPoint.r2 = p2 - x2;
//            contactPoint.penetrationDepth = externalContact->getPenetrationDepth();
//            contactPoint.isRestingContact = externalContact->getIsRestingContact();
//            externalContact->setIsRestingContact(true);
//            contactPoint.oldFrictionVector1 = externalContact->getFrictionVector1();
//            contactPoint.oldFrictionVector2 = externalContact->getFrictionVector2();
//            contactPoint.penetrationImpulse = 0.0;
//            contactPoint.friction1Impulse = 0.0;
//            contactPoint.friction2Impulse = 0.0;
//            contactPoint.rollingResistanceImpulse = Vector3::zero();

//            // If we solve the friction constraints at the center of the contact manifold
//            if (mIsSolveFrictionAtContactManifoldCenterActive) {
//                internalManifold.frictionPointBody1 += p1;
//                internalManifold.frictionPointBody2 += p2;
//            }
//        }

//        // If we solve the friction constraints at the center of the contact manifold
//        if (mIsSolveFrictionAtContactManifoldCenterActive) {

//            internalManifold.frictionPointBody1 /=static_cast<decimal>(internalManifold.nbContacts);
//            internalManifold.frictionPointBody2 /=static_cast<decimal>(internalManifold.nbContacts);
//            internalManifold.r1Friction = internalManifold.frictionPointBody1 - x1;
//            internalManifold.r2Friction = internalManifold.frictionPointBody2 - x2;
//            internalManifold.oldFrictionVector1 = externalManifold->getFrictionVector1();
//            internalManifold.oldFrictionVector2 = externalManifold->getFrictionVector2();

//            // If warm starting is active
//            if (mIsWarmStartingActive) {

//                // Initialize the accumulated impulses with the previous step accumulated impulses
//                internalManifold.friction1Impulse = externalManifold->getFrictionImpulse1();
//                internalManifold.friction2Impulse = externalManifold->getFrictionImpulse2();
//                internalManifold.frictionTwistImpulse = externalManifold->getFrictionTwistImpulse();
//            }
//            else {

//                // Initialize the accumulated impulses to zero
//                internalManifold.friction1Impulse = 0.0;
//                internalManifold.friction2Impulse = 0.0;
//                internalManifold.frictionTwistImpulse = 0.0;
//                internalManifold.rollingResistanceImpulse = Vector3(0, 0, 0);
//            }
//        }
//    }

}

void rpContactSolverSequential::initializeContactConstraints()
{


    for (uint c=0; c<mNbContactManifolds; c++)
    {

        ContactManifoldSolver* manifold = &mContactConstraints[c];


        rpRigidPhysicsBody*  body1 = static_cast<rpRigidPhysicsBody*>(manifold->externalContactManifold->getBody1());
        rpRigidPhysicsBody*  body2 = static_cast<rpRigidPhysicsBody*>(manifold->externalContactManifold->getBody2());


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
                // Get the cached accumulated impulses from the previous step
                contactPoint.AccumulatedPenetrationImpulse = externalContact->getPenetrationImpulse();
                contactPoint.AccumulatedFriction1Impulse = externalContact->getFrictionImpulse1();
                contactPoint.AccumulatedFriction2Impulse = externalContact->getFrictionImpulse2();
                contactPoint.AccumulatedRollingResistanceImpulse = externalContact->getRollingResistanceImpulse();

            }

            // Initialize the split impulses to zero
            contactPoint.AccumulatedPenetrationSplitImpulse = 0.0;


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

    /**
    // For each contact constraint
     for (uint c=0; c<mNbContactManifolds; c++) {

         ContactManifoldSolver& manifold = mContactConstraints[c];

         // Get the inertia tensors of both bodies
         Matrix3x3& I1 = manifold.inverseInertiaTensorBody1;
         Matrix3x3& I2 = manifold.inverseInertiaTensorBody2;

         // If we solve the friction constraints at the center of the contact manifold
         if (mIsSolveFrictionAtContactManifoldCenterActive) {
             manifold.normal = Vector3(0.0, 0.0, 0.0);
         }

         // Get the velocities of the bodies
         const Vector3& v1 = mLinearVelocities[manifold.indexBody1];
         const Vector3& w1 = mAngularVelocities[manifold.indexBody1];
         const Vector3& v2 = mLinearVelocities[manifold.indexBody2];
         const Vector3& w2 = mAngularVelocities[manifold.indexBody2];

         // For each contact point constraint
         for (uint i=0; i<manifold.nbContacts; i++) {

             ContactPointSolver& contactPoint = manifold.contacts[i];
             ContactPoint* externalContact = contactPoint.externalContact;

             // Compute the velocity difference
             Vector3 deltaV = v2 + w2.cross(contactPoint.r2) - v1 - w1.cross(contactPoint.r1);

             contactPoint.r1CrossN = contactPoint.r1.cross(contactPoint.normal);
             contactPoint.r2CrossN = contactPoint.r2.cross(contactPoint.normal);

             // Compute the inverse mass matrix K for the penetration constraint
             decimal massPenetration = manifold.massInverseBody1 + manifold.massInverseBody2 +
                     ((I1 * contactPoint.r1CrossN).cross(contactPoint.r1)).dot(contactPoint.normal) +
                     ((I2 * contactPoint.r2CrossN).cross(contactPoint.r2)).dot(contactPoint.normal);
             massPenetration > 0.0 ? contactPoint.inversePenetrationMass = decimal(1.0) /
                                                                           massPenetration :
                                                                           decimal(0.0);

             // If we do not solve the friction constraints at the center of the contact manifold
             if (!mIsSolveFrictionAtContactManifoldCenterActive) {

                 // Compute the friction vectors
                 computeFrictionVectors(deltaV, contactPoint);

                 contactPoint.r1CrossT1 = contactPoint.r1.cross(contactPoint.frictionVector1);
                 contactPoint.r1CrossT2 = contactPoint.r1.cross(contactPoint.frictionVector2);
                 contactPoint.r2CrossT1 = contactPoint.r2.cross(contactPoint.frictionVector1);
                 contactPoint.r2CrossT2 = contactPoint.r2.cross(contactPoint.frictionVector2);

                 // Compute the inverse mass matrix K for the friction
                 // constraints at each contact point
                 decimal friction1Mass = manifold.massInverseBody1 + manifold.massInverseBody2 +
                                         ((I1 * contactPoint.r1CrossT1).cross(contactPoint.r1)).dot(
                                         contactPoint.frictionVector1) +
                                         ((I2 * contactPoint.r2CrossT1).cross(contactPoint.r2)).dot(
                                         contactPoint.frictionVector1);
                 decimal friction2Mass = manifold.massInverseBody1 + manifold.massInverseBody2 +
                                         ((I1 * contactPoint.r1CrossT2).cross(contactPoint.r1)).dot(
                                         contactPoint.frictionVector2) +
                                         ((I2 * contactPoint.r2CrossT2).cross(contactPoint.r2)).dot(
                                         contactPoint.frictionVector2);
                 friction1Mass > 0.0 ? contactPoint.inverseFriction1Mass = decimal(1.0) /
                                                                           friction1Mass :
                                                                           decimal(0.0);
                 friction2Mass > 0.0 ? contactPoint.inverseFriction2Mass = decimal(1.0) /
                                                                           friction2Mass :
                                                                           decimal(0.0);
             }

             // Compute the restitution velocity bias "b". We compute this here instead
             // of inside the solve() method because we need to use the velocity difference
             // at the beginning of the contact. Note that if it is a resting contact (normal
             // velocity bellow a given threshold), we do not add a restitution velocity bias
             contactPoint.restitutionBias = 0.0;
             decimal deltaVDotN = deltaV.dot(contactPoint.normal);
             if (deltaVDotN < -RESTITUTION_VELOCITY_THRESHOLD) {
                 contactPoint.restitutionBias = manifold.restitutionFactor * deltaVDotN;
             }

             // If the warm starting of the contact solver is active
             if (mIsWarmStartingActive) {

                 // Get the cached accumulated impulses from the previous step
                 contactPoint.penetrationImpulse = externalContact->getPenetrationImpulse();
                 contactPoint.friction1Impulse = externalContact->getFrictionImpulse1();
                 contactPoint.friction2Impulse = externalContact->getFrictionImpulse2();
                 contactPoint.rollingResistanceImpulse = externalContact->getRollingResistanceImpulse();
             }

             // Initialize the split impulses to zero
             contactPoint.penetrationSplitImpulse = 0.0;

             // If we solve the friction constraints at the center of the contact manifold
             if (mIsSolveFrictionAtContactManifoldCenterActive) {
                 manifold.normal += contactPoint.normal;
             }
         }

         // Compute the inverse K matrix for the rolling resistance constraint
         manifold.inverseRollingResistance.setToZero();
         if (manifold.rollingResistanceFactor > 0 && (manifold.isBody1DynamicType || manifold.isBody2DynamicType)) {
             manifold.inverseRollingResistance = manifold.inverseInertiaTensorBody1 + manifold.inverseInertiaTensorBody2;
             manifold.inverseRollingResistance = manifold.inverseRollingResistance.getInverse();
         }

         // If we solve the friction constraints at the center of the contact manifold
         if (mIsSolveFrictionAtContactManifoldCenterActive) {

             manifold.normal.normalize();

             Vector3 deltaVFrictionPoint = v2 + w2.cross(manifold.r2Friction) -
                                           v1 - w1.cross(manifold.r1Friction);

             // Compute the friction vectors
             computeFrictionVectors(deltaVFrictionPoint, manifold);

             // Compute the inverse mass matrix K for the friction constraints at the center of
             // the contact manifold
             manifold.r1CrossT1 = manifold.r1Friction.cross(manifold.frictionVector1);
             manifold.r1CrossT2 = manifold.r1Friction.cross(manifold.frictionVector2);
             manifold.r2CrossT1 = manifold.r2Friction.cross(manifold.frictionVector1);
             manifold.r2CrossT2 = manifold.r2Friction.cross(manifold.frictionVector2);
             decimal friction1Mass = manifold.massInverseBody1 + manifold.massInverseBody2 +
                                     ((I1 * manifold.r1CrossT1).cross(manifold.r1Friction)).dot(
                                     manifold.frictionVector1) +
                                     ((I2 * manifold.r2CrossT1).cross(manifold.r2Friction)).dot(
                                     manifold.frictionVector1);
             decimal friction2Mass = manifold.massInverseBody1 + manifold.massInverseBody2 +
                                     ((I1 * manifold.r1CrossT2).cross(manifold.r1Friction)).dot(
                                     manifold.frictionVector2) +
                                     ((I2 * manifold.r2CrossT2).cross(manifold.r2Friction)).dot(
                                     manifold.frictionVector2);
             decimal frictionTwistMass = manifold.normal.dot(manifold.inverseInertiaTensorBody1 *
                                            manifold.normal) +
                                         manifold.normal.dot(manifold.inverseInertiaTensorBody2 *
                                            manifold.normal);
             friction1Mass > 0.0 ? manifold.inverseFriction1Mass = decimal(1.0)/friction1Mass
                                                                          : decimal(0.0);
             friction2Mass > 0.0 ? manifold.inverseFriction2Mass = decimal(1.0)/friction2Mass
                                                                          : decimal(0.0);
             frictionTwistMass > 0.0 ? manifold.inverseTwistFrictionMass = decimal(1.0) /
                                                                                  frictionTwistMass :
                                                                                  decimal(0.0);
         }
     }
     /**/
}

void rpContactSolverSequential::warmStart()
{

    // Check that warm starting is active
       if (!mIsWarmStartingActive) return;


       // For each constraint
       for (uint c=0; c<mNbContactManifolds; c++)
       {

           ContactManifoldSolver& contactManifold = mContactConstraints[c];

           rpRigidPhysicsBody*  body1 = static_cast<rpRigidPhysicsBody*>(contactManifold.externalContactManifold->getBody1());
           rpRigidPhysicsBody*  body2 = static_cast<rpRigidPhysicsBody*>(contactManifold.externalContactManifold->getBody2());

           bool atLeastOneRestingContactPoint = false;


            for (uint i = 0; i < contactManifold.nbContacts; ++i)
            {
                ContactPointSolver &contactPoint = contactManifold.contacts[i];
                rpContactPoint *cp = contactPoint.externalContact;

                // If it is not a new contact (this contact was already existing at last time step)
                if (contactPoint.isRestingContact )
                {

                    atLeastOneRestingContactPoint = true;

                    Vector3 ContactPointA = (cp->getWorldPointOnBody1());
                    Vector3 ContactPointB = (cp->getWorldPointOnBody2());


                    scalar  &_accumulaterImpuls                        =  contactPoint.AccumulatedPenetrationImpulse;
                    scalar  &_accumulaterImpulsFriction1               =  contactPoint.AccumulatedFriction1Impulse;
                    scalar  &_accumulaterImpulsFriction2               =  contactPoint.AccumulatedFriction2Impulse;
                    Vector3 &_accumulaterRollingResistanceImpulse      =  contactPoint.AccumulatedRollingResistanceImpulse;
                    Vector3 &_accumulaterRollingResistanceImpulseDelta =  contactPoint.AccumulatedRollingResistanceSplitImpulse;



                    // Project the old friction impulses (with old friction vectors) into
                    // the new friction vectors to get the new friction impulses
                    contactManifold.oldFrictionVector1 = contactManifold.frictionVector1;
                    contactManifold.oldFrictionVector2 = contactManifold.frictionVector2;
                    Vector3 oldFrictionImpulse = _accumulaterImpulsFriction1 * contactPoint.oldFrictionVector1 +
                            _accumulaterImpulsFriction2 * contactPoint.oldFrictionVector2;

                    _accumulaterImpulsFriction1 = oldFrictionImpulse.dot(contactPoint.frictionVector1);
                    _accumulaterImpulsFriction2 = oldFrictionImpulse.dot(contactPoint.frictionVector2);





                    /****************************************************************************/

                    /// Penetration Distance
                    scalar beta = mIsSplitImpulseActive ? BETA_SPLIT_IMPULSE : BETA;
                    scalar sepp = -Abs( cp->getPenetrationDepth() );
                    cp->setPenetrationDepth( -(beta/mTimeStep) * Min( scalar(0), sepp + OFFSET_SLOPE ));
                    /****************************************************************************/




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


                            body1->applySplitImpulseAngular(-_accumulaterRollingResistanceImpulseDelta);
                            body2->applySplitImpulseAngular( _accumulaterRollingResistanceImpulseDelta);

                        }


                    }
                }
                else
                {  // If it is a new contact point

                    // Initialize the accumulated impulses to zero
                    contactPoint.AccumulatedPenetrationImpulse = 0.0;
                    contactPoint.AccumulatedFriction1Impulse   = 0.0;
                    contactPoint.AccumulatedFriction2Impulse   = 0.0;
                    contactPoint.AccumulatedRollingResistanceImpulse = Vector3::zero();
                }


            }



            // If we solve the friction constraints at the center of the contact manifold and there is
            // at least one resting contact point in the contact manifold

            if (mIsSolveFrictionAtContactManifoldCenterActive && atLeastOneRestingContactPoint && contactManifold.nbContacts > 0)
            {

                // Project the old friction impulses (with old friction vectors) into the new friction
                // vectors to get the new friction impulses
                contactManifold.oldFrictionVector1 = contactManifold.frictionVector1;
                contactManifold.oldFrictionVector2 = contactManifold.frictionVector2;
                Vector3 oldFrictionImpulse = contactManifold.AccumulatedFriction1Impulse * contactManifold.oldFrictionVector1 +
                                             contactManifold.AccumulatedFriction2Impulse * contactManifold.oldFrictionVector2;
                contactManifold.AccumulatedFriction1Impulse = oldFrictionImpulse.dot(contactManifold.frictionVector1);
                contactManifold.AccumulatedFriction2Impulse = oldFrictionImpulse.dot(contactManifold.frictionVector2);



                // ------ First friction constraint at the center of the contact manifold ------ //

                // Compute the impulse P = J^T * lambda
                Vector3 linearImpulseBody1 = -contactManifold.frictionVector1 * contactManifold.AccumulatedFriction1Impulse;
                Vector3 angularImpulseBody1 = -contactManifold.r1CrossT1 * contactManifold.AccumulatedFriction1Impulse;
                Vector3 linearImpulseBody2 = contactManifold.frictionVector1 * contactManifold.AccumulatedFriction1Impulse;
                Vector3 angularImpulseBody2 = contactManifold.r2CrossT1 * contactManifold.AccumulatedFriction1Impulse;


                body1->applyImpulseLinear(linearImpulseBody1);
                body1->applyImpulseAngular(angularImpulseBody1);

                body2->applyImpulseLinear(linearImpulseBody2);
                body2->applyImpulseAngular(angularImpulseBody2);


                // ------ Second friction constraint at the center of the contact manifold ----- //

                // Compute the impulse P = J^T * lambda
                linearImpulseBody1  = -contactManifold.frictionVector2 * contactManifold.AccumulatedFriction2Impulse;
                angularImpulseBody1 = -contactManifold.r1CrossT2 * contactManifold.AccumulatedFriction2Impulse;
                linearImpulseBody2  =  contactManifold.frictionVector2 * contactManifold.AccumulatedFriction2Impulse;
                angularImpulseBody2 =  contactManifold.r2CrossT2 * contactManifold.AccumulatedFriction2Impulse;




                body1->applyImpulseLinear(linearImpulseBody1);
                body1->applyImpulseAngular(angularImpulseBody1);

                body2->applyImpulseLinear(linearImpulseBody2);
                body2->applyImpulseAngular(angularImpulseBody2);



                // ------ Twist friction constraint at the center of the contact manifold ------ //

                // Compute the impulse P = J^T * lambda
                linearImpulseBody1  =  Vector3(0.0, 0.0, 0.0);
                angularImpulseBody1 = -contactManifold.normal * contactManifold.AccumulatedFrictionTwistImpulse;
                linearImpulseBody2  =  Vector3(0.0, 0.0, 0.0);
                angularImpulseBody2 =  contactManifold.normal * contactManifold.AccumulatedFrictionTwistImpulse;


                body1->applyImpulseAngular(angularImpulseBody1);
                body2->applyImpulseAngular(angularImpulseBody2);

                // ------ Rolling resistance at the center of the contact manifold ------ //

                // Compute the impulse P = J^T * lambda
                angularImpulseBody1 = -contactManifold.AccumulatedRollingResistanceImpulse;
                angularImpulseBody2 =  contactManifold.AccumulatedRollingResistanceImpulse;


                body1->applyImpulseAngular(angularImpulseBody1);
                body2->applyImpulseAngular(angularImpulseBody2);

            }
            else
            {  // If it is a new contact manifold

                // Initialize the accumulated impulses to zero
                contactManifold.AccumulatedFriction1Impulse = 0.0;
                contactManifold.AccumulatedFriction2Impulse = 0.0;
                contactManifold.AccumulatedFrictionTwistImpulse = 0.0;
                contactManifold.AccumulatedRollingResistanceImpulse = Vector3::zero();
                atLeastOneRestingContactPoint = true;
            }

       }

       /**
       // For each constraint
       for (uint c=0; c<mNbContactManifolds; c++) {

           ContactManifoldSolver& contactManifold = mContactConstraints[c];

           bool atLeastOneRestingContactPoint = false;

           for (uint i=0; i<contactManifold.nbContacts; i++) {

               ContactPointSolver& contactPoint = contactManifold.contacts[i];

               // If it is not a new contact (this contact was already existing at last time step)
               if (contactPoint.isRestingContact) {

                   atLeastOneRestingContactPoint = true;

                   // --------- Penetration --------- //

                   // Compute the impulse P = J^T * lambda
                   const Impulse impulsePenetration = computePenetrationImpulse(
                                                        contactPoint.penetrationImpulse, contactPoint);

                   // Apply the impulse to the bodies of the constraint
                   applyImpulse(impulsePenetration, contactManifold);

                   // If we do not solve the friction constraints at the center of the contact manifold
                   if (!mIsSolveFrictionAtContactManifoldCenterActive) {

                       // Project the old friction impulses (with old friction vectors) into
                       // the new friction vectors to get the new friction impulses
                       Vector3 oldFrictionImpulse = contactPoint.friction1Impulse *
                                                    contactPoint.oldFrictionVector1 +
                                                    contactPoint.friction2Impulse *
                                                    contactPoint.oldFrictionVector2;
                       contactPoint.friction1Impulse = oldFrictionImpulse.dot(
                                                          contactPoint.frictionVector1);
                       contactPoint.friction2Impulse = oldFrictionImpulse.dot(
                                                          contactPoint.frictionVector2);

                       // --------- Friction 1 --------- //

                       // Compute the impulse P = J^T * lambda
                       const Impulse impulseFriction1 = computeFriction1Impulse(
                                                          contactPoint.friction1Impulse, contactPoint);

                       // Apply the impulses to the bodies of the constraint
                       applyImpulse(impulseFriction1, contactManifold);

                       // --------- Friction 2 --------- //

                       // Compute the impulse P=J^T * lambda
                      const Impulse impulseFriction2 = computeFriction2Impulse(
                                                          contactPoint.friction2Impulse, contactPoint);

                       // Apply the impulses to the bodies of the constraint
                       applyImpulse(impulseFriction2, contactManifold);

                       // ------ Rolling resistance------ //

                       if (contactManifold.rollingResistanceFactor > 0) {

                           // Compute the impulse P = J^T * lambda
                           const Impulse impulseRollingResistance(Vector3::zero(), -contactPoint.rollingResistanceImpulse,
                                                                  Vector3::zero(), contactPoint.rollingResistanceImpulse);

                           // Apply the impulses to the bodies of the constraint
                           applyImpulse(impulseRollingResistance, contactManifold);
                       }
                   }
               }
               else {  // If it is a new contact point

                   // Initialize the accumulated impulses to zero
                   contactPoint.penetrationImpulse = 0.0;
                   contactPoint.friction1Impulse = 0.0;
                   contactPoint.friction2Impulse = 0.0;
                   contactPoint.rollingResistanceImpulse = Vector3::zero();
               }
           }

           // If we solve the friction constraints at the center of the contact manifold and there is
           // at least one resting contact point in the contact manifold
           if (mIsSolveFrictionAtContactManifoldCenterActive && atLeastOneRestingContactPoint) {

               // Project the old friction impulses (with old friction vectors) into the new friction
               // vectors to get the new friction impulses
               Vector3 oldFrictionImpulse = contactManifold.friction1Impulse *
                                            contactManifold.oldFrictionVector1 +
                                            contactManifold.friction2Impulse *
                                            contactManifold.oldFrictionVector2;
               contactManifold.friction1Impulse = oldFrictionImpulse.dot(
                                                     contactManifold.frictionVector1);
               contactManifold.friction2Impulse = oldFrictionImpulse.dot(
                                                     contactManifold.frictionVector2);

               // ------ First friction constraint at the center of the contact manifold ------ //

               // Compute the impulse P = J^T * lambda
               Vector3 linearImpulseBody1 = -contactManifold.frictionVector1 *
                                             contactManifold.friction1Impulse;
               Vector3 angularImpulseBody1 = -contactManifold.r1CrossT1 *
                                              contactManifold.friction1Impulse;
               Vector3 linearImpulseBody2 = contactManifold.frictionVector1 *
                                            contactManifold.friction1Impulse;
               Vector3 angularImpulseBody2 = contactManifold.r2CrossT1 *
                                             contactManifold.friction1Impulse;
               const Impulse impulseFriction1(linearImpulseBody1, angularImpulseBody1,
                                              linearImpulseBody2, angularImpulseBody2);

               // Apply the impulses to the bodies of the constraint
               applyImpulse(impulseFriction1, contactManifold);

               // ------ Second friction constraint at the center of the contact manifold ----- //

               // Compute the impulse P = J^T * lambda
               linearImpulseBody1 = -contactManifold.frictionVector2 *
                                     contactManifold.friction2Impulse;
               angularImpulseBody1 = -contactManifold.r1CrossT2 *
                                      contactManifold.friction2Impulse;
               linearImpulseBody2 = contactManifold.frictionVector2 *
                                    contactManifold.friction2Impulse;
               angularImpulseBody2 = contactManifold.r2CrossT2 *
                                     contactManifold.friction2Impulse;
               const Impulse impulseFriction2(linearImpulseBody1, angularImpulseBody1,
                                              linearImpulseBody2, angularImpulseBody2);

               // Apply the impulses to the bodies of the constraint
               applyImpulse(impulseFriction2, contactManifold);

               // ------ Twist friction constraint at the center of the contact manifold ------ //

               // Compute the impulse P = J^T * lambda
               linearImpulseBody1 = Vector3(0.0, 0.0, 0.0);
               angularImpulseBody1 = -contactManifold.normal * contactManifold.frictionTwistImpulse;
               linearImpulseBody2 = Vector3(0.0, 0.0, 0.0);
               angularImpulseBody2 = contactManifold.normal * contactManifold.frictionTwistImpulse;
               const Impulse impulseTwistFriction(linearImpulseBody1, angularImpulseBody1,
                                                  linearImpulseBody2, angularImpulseBody2);

               // Apply the impulses to the bodies of the constraint
               applyImpulse(impulseTwistFriction, contactManifold);

               // ------ Rolling resistance at the center of the contact manifold ------ //

               // Compute the impulse P = J^T * lambda
               angularImpulseBody1 = -contactManifold.rollingResistanceImpulse;
               angularImpulseBody2 = contactManifold.rollingResistanceImpulse;
               const Impulse impulseRollingResistance(Vector3::zero(), angularImpulseBody1,
                                                      Vector3::zero(), angularImpulseBody2);

               // Apply the impulses to the bodies of the constraint
               applyImpulse(impulseRollingResistance, contactManifold);
           }
           else {  // If it is a new contact manifold

               // Initialize the accumulated impulses to zero
               contactManifold.friction1Impulse = 0.0;
               contactManifold.friction2Impulse = 0.0;
               contactManifold.frictionTwistImpulse = 0.0;
               contactManifold.rollingResistanceImpulse = Vector3::zero();
           }
       }
       /**/
}

void rpContactSolverSequential::solveVelocityConstraint()
{


    // For each contact manifold
    for (uint c=0; c<mNbContactManifolds; c++)
    {

        ContactManifoldSolver* contactManifold = &mContactConstraints[c];


        rpRigidPhysicsBody*  body1 = static_cast<rpRigidPhysicsBody*>(contactManifold->externalContactManifold->getBody1());
        rpRigidPhysicsBody*  body2 = static_cast<rpRigidPhysicsBody*>(contactManifold->externalContactManifold->getBody2());


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
               if (contactManifold->rollingResistanceFactor > 0)
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
    }

    /**
  decimal deltaLambda;
    decimal lambdaTemp;

    // For each contact manifold
    for (uint c=0; c<mNbContactManifolds; c++) {

        ContactManifoldSolver& contactManifold = mContactConstraints[c];

        decimal sumPenetrationImpulse = 0.0;

        // Get the constrained velocities
        const Vector3& v1 = mLinearVelocities[contactManifold.indexBody1];
        const Vector3& w1 = mAngularVelocities[contactManifold.indexBody1];
        const Vector3& v2 = mLinearVelocities[contactManifold.indexBody2];
        const Vector3& w2 = mAngularVelocities[contactManifold.indexBody2];

        for (uint i=0; i<contactManifold.nbContacts; i++) {

            ContactPointSolver& contactPoint = contactManifold.contacts[i];

            // --------- Penetration --------- //

            // Compute J*v
            Vector3 deltaV = v2 + w2.cross(contactPoint.r2) - v1 - w1.cross(contactPoint.r1);
            decimal deltaVDotN = deltaV.dot(contactPoint.normal);
            decimal Jv = deltaVDotN;

            // Compute the bias "b" of the constraint
            decimal beta = mIsSplitImpulseActive ? BETA_SPLIT_IMPULSE : BETA;
            decimal biasPenetrationDepth = 0.0;
            if (contactPoint.penetrationDepth > SLOP) biasPenetrationDepth = -(beta/mTimeStep) *
                    max(0.0f, float(contactPoint.penetrationDepth - SLOP));
            decimal b = biasPenetrationDepth + contactPoint.restitutionBias;

            // Compute the Lagrange multiplier lambda
            if (mIsSplitImpulseActive) {
                deltaLambda = - (Jv + contactPoint.restitutionBias) *
                        contactPoint.inversePenetrationMass;
            }
            else {
                deltaLambda = - (Jv + b) * contactPoint.inversePenetrationMass;
            }
            lambdaTemp = contactPoint.penetrationImpulse;
            contactPoint.penetrationImpulse = std::max(contactPoint.penetrationImpulse +
                                                       deltaLambda, decimal(0.0));
            deltaLambda = contactPoint.penetrationImpulse - lambdaTemp;

            // Compute the impulse P=J^T * lambda
            const Impulse impulsePenetration = computePenetrationImpulse(deltaLambda,
                                                                         contactPoint);

            // Apply the impulse to the bodies of the constraint
            applyImpulse(impulsePenetration, contactManifold);

            sumPenetrationImpulse += contactPoint.penetrationImpulse;

            // If the split impulse position correction is active
            if (mIsSplitImpulseActive) {

                // Split impulse (position correction)
                const Vector3& v1Split = mSplitLinearVelocities[contactManifold.indexBody1];
                const Vector3& w1Split = mSplitAngularVelocities[contactManifold.indexBody1];
                const Vector3& v2Split = mSplitLinearVelocities[contactManifold.indexBody2];
                const Vector3& w2Split = mSplitAngularVelocities[contactManifold.indexBody2];
                Vector3 deltaVSplit = v2Split + w2Split.cross(contactPoint.r2) -
                        v1Split - w1Split.cross(contactPoint.r1);
                decimal JvSplit = deltaVSplit.dot(contactPoint.normal);
                decimal deltaLambdaSplit = - (JvSplit + biasPenetrationDepth) *
                        contactPoint.inversePenetrationMass;
                decimal lambdaTempSplit = contactPoint.penetrationSplitImpulse;
                contactPoint.penetrationSplitImpulse = std::max(
                            contactPoint.penetrationSplitImpulse +
                            deltaLambdaSplit, decimal(0.0));
                deltaLambda = contactPoint.penetrationSplitImpulse - lambdaTempSplit;

                // Compute the impulse P=J^T * lambda
                const Impulse splitImpulsePenetration = computePenetrationImpulse(
                            deltaLambdaSplit, contactPoint);

                applySplitImpulse(splitImpulsePenetration, contactManifold);
            }

            // If we do not solve the friction constraints at the center of the contact manifold
            if (!mIsSolveFrictionAtContactManifoldCenterActive) {

                // --------- Friction 1 --------- //

                // Compute J*v
                deltaV = v2 + w2.cross(contactPoint.r2) - v1 - w1.cross(contactPoint.r1);
                Jv = deltaV.dot(contactPoint.frictionVector1);

                // Compute the Lagrange multiplier lambda
                deltaLambda = -Jv;
                deltaLambda *= contactPoint.inverseFriction1Mass;
                decimal frictionLimit = contactManifold.frictionCoefficient *
                        contactPoint.penetrationImpulse;
                lambdaTemp = contactPoint.friction1Impulse;
                contactPoint.friction1Impulse = std::max(-frictionLimit,
                                                         std::min(contactPoint.friction1Impulse
                                                                  + deltaLambda, frictionLimit));
                deltaLambda = contactPoint.friction1Impulse - lambdaTemp;

                // Compute the impulse P=J^T * lambda
                const Impulse impulseFriction1 = computeFriction1Impulse(deltaLambda,
                                                                         contactPoint);

                // Apply the impulses to the bodies of the constraint
                applyImpulse(impulseFriction1, contactManifold);

                // --------- Friction 2 --------- //

                // Compute J*v
                deltaV = v2 + w2.cross(contactPoint.r2) - v1 - w1.cross(contactPoint.r1);
                Jv = deltaV.dot(contactPoint.frictionVector2);

                // Compute the Lagrange multiplier lambda
                deltaLambda = -Jv;
                deltaLambda *= contactPoint.inverseFriction2Mass;
                frictionLimit = contactManifold.frictionCoefficient *
                        contactPoint.penetrationImpulse;
                lambdaTemp = contactPoint.friction2Impulse;
                contactPoint.friction2Impulse = std::max(-frictionLimit,
                                                         std::min(contactPoint.friction2Impulse
                                                                  + deltaLambda, frictionLimit));
                deltaLambda = contactPoint.friction2Impulse - lambdaTemp;

                // Compute the impulse P=J^T * lambda
                const Impulse impulseFriction2 = computeFriction2Impulse(deltaLambda,
                                                                         contactPoint);

                // Apply the impulses to the bodies of the constraint
                applyImpulse(impulseFriction2, contactManifold);

                // --------- Rolling resistance constraint --------- //

                if (contactManifold.rollingResistanceFactor > 0) {

                    // Compute J*v
                    const Vector3 JvRolling = w2 - w1;

                    // Compute the Lagrange multiplier lambda
                    Vector3 deltaLambdaRolling = contactManifold.inverseRollingResistance * (-JvRolling);
                    decimal rollingLimit = contactManifold.rollingResistanceFactor * contactPoint.penetrationImpulse;
                    Vector3 lambdaTempRolling = contactPoint.rollingResistanceImpulse;
                    contactPoint.rollingResistanceImpulse = clamp(contactPoint.rollingResistanceImpulse +
                                                                         deltaLambdaRolling, rollingLimit);
                    deltaLambdaRolling = contactPoint.rollingResistanceImpulse - lambdaTempRolling;

                    // Compute the impulse P=J^T * lambda
                    const Impulse impulseRolling(Vector3::zero(), -deltaLambdaRolling,
                                                 Vector3::zero(), deltaLambdaRolling);

                    // Apply the impulses to the bodies of the constraint
                    applyImpulse(impulseRolling, contactManifold);
                }
            }
        }

        // If we solve the friction constraints at the center of the contact manifold
        if (mIsSolveFrictionAtContactManifoldCenterActive) {

            // ------ First friction constraint at the center of the contact manifol ------ //

            // Compute J*v
            Vector3 deltaV = v2 + w2.cross(contactManifold.r2Friction)
                    - v1 - w1.cross(contactManifold.r1Friction);
            decimal Jv = deltaV.dot(contactManifold.frictionVector1);

            // Compute the Lagrange multiplier lambda
            decimal deltaLambda = -Jv * contactManifold.inverseFriction1Mass;
            decimal frictionLimit = contactManifold.frictionCoefficient * sumPenetrationImpulse;
            lambdaTemp = contactManifold.friction1Impulse;
            contactManifold.friction1Impulse = std::max(-frictionLimit,
                                                        std::min(contactManifold.friction1Impulse +
                                                                 deltaLambda, frictionLimit));
            deltaLambda = contactManifold.friction1Impulse - lambdaTemp;

            // Compute the impulse P=J^T * lambda
            Vector3 linearImpulseBody1 = -contactManifold.frictionVector1 * deltaLambda;
            Vector3 angularImpulseBody1 = -contactManifold.r1CrossT1 * deltaLambda;
            Vector3 linearImpulseBody2 = contactManifold.frictionVector1 * deltaLambda;
            Vector3 angularImpulseBody2 = contactManifold.r2CrossT1 * deltaLambda;
            const Impulse impulseFriction1(linearImpulseBody1, angularImpulseBody1,
                                           linearImpulseBody2, angularImpulseBody2);

            // Apply the impulses to the bodies of the constraint
            applyImpulse(impulseFriction1, contactManifold);

            // ------ Second friction constraint at the center of the contact manifol ----- //

            // Compute J*v
            deltaV = v2 + w2.cross(contactManifold.r2Friction)
                    - v1 - w1.cross(contactManifold.r1Friction);
            Jv = deltaV.dot(contactManifold.frictionVector2);

            // Compute the Lagrange multiplier lambda
            deltaLambda = -Jv * contactManifold.inverseFriction2Mass;
            frictionLimit = contactManifold.frictionCoefficient * sumPenetrationImpulse;
            lambdaTemp = contactManifold.friction2Impulse;
            contactManifold.friction2Impulse = std::max(-frictionLimit,
                                                        std::min(contactManifold.friction2Impulse +
                                                                 deltaLambda, frictionLimit));
            deltaLambda = contactManifold.friction2Impulse - lambdaTemp;

            // Compute the impulse P=J^T * lambda
            linearImpulseBody1 = -contactManifold.frictionVector2 * deltaLambda;
            angularImpulseBody1 = -contactManifold.r1CrossT2 * deltaLambda;
            linearImpulseBody2 = contactManifold.frictionVector2 * deltaLambda;
            angularImpulseBody2 = contactManifold.r2CrossT2 * deltaLambda;
            const Impulse impulseFriction2(linearImpulseBody1, angularImpulseBody1,
                                           linearImpulseBody2, angularImpulseBody2);

            // Apply the impulses to the bodies of the constraint
            applyImpulse(impulseFriction2, contactManifold);

            // ------ Twist friction constraint at the center of the contact manifol ------ //

            // Compute J*v
            deltaV = w2 - w1;
            Jv = deltaV.dot(contactManifold.normal);

            deltaLambda = -Jv * (contactManifold.inverseTwistFrictionMass);
            frictionLimit = contactManifold.frictionCoefficient * sumPenetrationImpulse;
            lambdaTemp = contactManifold.frictionTwistImpulse;
            contactManifold.frictionTwistImpulse = std::max(-frictionLimit,
                                                            std::min(contactManifold.frictionTwistImpulse
                                                                     + deltaLambda, frictionLimit));
            deltaLambda = contactManifold.frictionTwistImpulse - lambdaTemp;

            // Compute the impulse P=J^T * lambda
            linearImpulseBody1 = Vector3(0.0, 0.0, 0.0);
            angularImpulseBody1 = -contactManifold.normal * deltaLambda;
            linearImpulseBody2 = Vector3(0.0, 0.0, 0.0);;
            angularImpulseBody2 = contactManifold.normal * deltaLambda;
            const Impulse impulseTwistFriction(linearImpulseBody1, angularImpulseBody1,
                                               linearImpulseBody2, angularImpulseBody2);

            // Apply the impulses to the bodies of the constraint
            applyImpulse(impulseTwistFriction, contactManifold);

            // --------- Rolling resistance constraint at the center of the contact manifold --------- //

            if (contactManifold.rollingResistanceFactor > 0) {

                // Compute J*v
                const Vector3 JvRolling = w2 - w1;

                // Compute the Lagrange multiplier lambda
                Vector3 deltaLambdaRolling = contactManifold.inverseRollingResistance * (-JvRolling);
                decimal rollingLimit = contactManifold.rollingResistanceFactor * sumPenetrationImpulse;
                Vector3 lambdaTempRolling = contactManifold.rollingResistanceImpulse;
                contactManifold.rollingResistanceImpulse = clamp(contactManifold.rollingResistanceImpulse +
                                                                     deltaLambdaRolling, rollingLimit);
                deltaLambdaRolling = contactManifold.rollingResistanceImpulse - lambdaTempRolling;

                // Compute the impulse P=J^T * lambda
                angularImpulseBody1 = -deltaLambdaRolling;
                angularImpulseBody2 = deltaLambdaRolling;
                const Impulse impulseRolling(Vector3::zero(), angularImpulseBody1,
                                             Vector3::zero(), angularImpulseBody2);

                // Apply the impulses to the bodies of the constraint
                applyImpulse(impulseRolling, contactManifold);
            }
        }
    }
    /**/
}

void rpContactSolverSequential::solvePositionConstraint()
{

    // For each contact manifold
    for (uint c=0; c<mNbContactManifolds; c++)
    {

        ContactManifoldSolver* contactManifold = &mContactConstraints[c];


        rpRigidPhysicsBody*  body1 = static_cast<rpRigidPhysicsBody*>(contactManifold->externalContactManifold->getBody1());
        rpRigidPhysicsBody*  body2 = static_cast<rpRigidPhysicsBody*>(contactManifold->externalContactManifold->getBody2());

        // Get the constrained velocities
        const Vector3& v1Split = body1->mSplitLinearVelocity;
        const Vector3& w1Split = body1->mSplitAngularVelocity;
        const Vector3& v2Split = body2->mSplitLinearVelocity;
        const Vector3& w2Split = body2->mSplitAngularVelocity;

        for (uint i = 0; i < contactManifold->nbContacts; ++i)
        {

            ContactPointSolver &contactPoint = contactManifold->contacts[i];
            rpContactPoint *cp = contactPoint.externalContact;


            scalar  &_accumulaterSplit = contactPoint.AccumulatedPenetrationSplitImpulse;
            Vector3 &_accumulaterRollingResistanceImpulseDelta = contactPoint.AccumulatedRollingResistanceSplitImpulse;


            // --------- Penetration --------- //
            Vector3 ContactPointA = (cp->getWorldPointOnBody1());
            Vector3 ContactPointB = (cp->getWorldPointOnBody2());

            Vector3 deltaVSplit = v2Split + w2Split.cross(contactPoint.r2) -
                                  v1Split - w1Split.cross(contactPoint.r1);
            scalar JvSplit = deltaVSplit.dot(contactPoint.normal);


            scalar biasPenetrationDepth = -cp->getPenetrationDepth();
            scalar deltaLambdaSplit = -(JvSplit + biasPenetrationDepth) * contactPoint.inversePenetrationMass;

            scalar lambdaTempSplit = _accumulaterSplit;
            _accumulaterSplit = Max( _accumulaterSplit + deltaLambdaSplit, scalar(0.0));
            scalar deltaLambda = _accumulaterSplit - lambdaTempSplit;


            body1->applySplitImpulse(-contactPoint.normal * deltaLambda , ContactPointA );
            body2->applySplitImpulse( contactPoint.normal * deltaLambda , ContactPointB );



            /**/
            if (contactManifold->rollingResistanceFactor > 0)
            {

                // Compute J*v
                const Vector3 JvRolling = w2Split  - w1Split;

                // Compute the Lagrange multiplier lambda
                Vector3 deltaLambdaRolling = contactManifold->inverseRollingResistance * (-JvRolling);
                scalar rollingLimit = contactManifold->rollingResistanceFactor * _accumulaterSplit;
                Vector3 lambdaTempRolling = _accumulaterRollingResistanceImpulseDelta;
                _accumulaterRollingResistanceImpulseDelta = Vector3::clamp(_accumulaterRollingResistanceImpulseDelta + deltaLambdaRolling, rollingLimit);
                deltaLambdaRolling = _accumulaterRollingResistanceImpulseDelta - lambdaTempRolling;


                ///Apply the pseudo impulses
                body1->applySplitImpulseAngular(-deltaLambdaRolling);
                body2->applySplitImpulseAngular( deltaLambdaRolling);


            }
            /**/

        }

    }
}


/// Store the computed impulses to use them to
/// warm start the solver at the next iteration
void rpContactSolverSequential::storeImpulses()
{
    // For each contact manifold
    for (uint c=0; c<mNbContactManifolds; c++)
    {

        ContactManifoldSolver& manifold = mContactConstraints[c];

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

}

/// Clean up the constraint solver
void rpContactSolverSequential::cleanup()
{
    if (mContactConstraints != NULL)
    {
        delete[] mContactConstraints;
        mContactConstraints = NULL;
    }
}



scalar rpContactSolverSequential::computeMixedRestitutionFactor(rpRigidPhysicsBody *body1, rpRigidPhysicsBody *body2) const
{
    scalar restitution1 = body1->getMaterial().getBounciness();
    scalar restitution2 = body2->getMaterial().getBounciness();

    // Return the largest restitution factor
    return (restitution1 > restitution2) ? restitution1 : restitution2;
}

scalar rpContactSolverSequential::computeMixedFrictionCoefficient(rpRigidPhysicsBody *body1, rpRigidPhysicsBody *body2) const
{
    // Use the geometric mean to compute the mixed friction coefficient
    return Sqrt(body1->getMaterial().getFrictionCoefficient() *
                body2->getMaterial().getFrictionCoefficient());
}

scalar rpContactSolverSequential::computeMixedRollingResistance(rpRigidPhysicsBody *body1, rpRigidPhysicsBody *body2) const
{
    return scalar(0.5f) * (body1->getMaterial().getRollingResistance() +
                           body2->getMaterial().getRollingResistance());
}

void rpContactSolverSequential::computeFrictionVectors(const Vector3 &deltaVelocity, rpContactSolverSequential::ContactPointSolver &contactPoint) const
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

void rpContactSolverSequential::computeFrictionVectors(const Vector3 &deltaVelocity, rpContactSolverSequential::ContactManifoldSolver *contact) const
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



}
