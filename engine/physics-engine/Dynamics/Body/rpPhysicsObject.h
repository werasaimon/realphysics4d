/*
 * rpPhysicsObject.h
 *
 *  Created on: 14 дек. 2016 г.
 *      Author: wera
 */

#ifndef SOURCE_ENGIE_KINEMATICPHYSICS_BODY_RPPHYSICSOBJECT_H_
#define SOURCE_ENGIE_KINEMATICPHYSICS_BODY_RPPHYSICSOBJECT_H_

#include "../../Collision/Body/rpBody.h"
#include "../../Collision/Body/rpCollisionBody.h"
#include "../../LinearMaths/mathematics.h"
#include "../../LinearMaths/rpLinearMtah.h"
#include "../../LinearMaths/rpMatrix3x3.h"
#include "../../LinearMaths/rpVector3D.h"
#include "../Material/rpPhysicsMaterial.h"

#include "../../Memory/memory.h"

namespace real_physics
{


class rpPhysicsObject : public rpCollisionBody
{

    protected:

		Transform        mWorldTransform;
		//uint             mIndexID;


        /// Private copy-constructor
        rpPhysicsObject(const rpPhysicsObject& body);

        /// Private assignment operator
        rpPhysicsObject& operator=(const rpPhysicsObject& body);

    public:

        rpPhysicsObject(const Transform& transform, rpCollisionDetection *CollideWorld, bodyindex id );


		virtual ~rpPhysicsObject();

        /// Add a collision shape to the body.
        virtual rpProxyShape* addCollisionShape(rpCollisionShape* collisionShape , scalar mass ,  const Transform& transform = Transform::identity() );

        /// Remove a collision shape from the body
        virtual void removeCollisionShape(const rpProxyShape* proxyShape);


        /// Update the broad-phase state for this body (because it has moved for instance)
        virtual void updateBroadPhaseState() const;


        /// Recompute the center of mass, total mass and inertia tensor of the body using all
        /// the collision shapes attached to the body.
        virtual void recomputeMassInformation() {}

        /// Recompute update the center of mass position
        virtual void updateTransformWithCenterOfMass() {}


        // set transform world oritation position
        void setWorldTransform(const Transform& worldTransform)
        {

            Vector3      pos  = (mStopedZeroPosition)? Vector3::ZERO : worldTransform.getPosition();
            Quaternion   quat = worldTransform.getOrientation();
            Transform    transform(pos , quat);

            setTransform(mWorldTransform = transform);
        }



        bool mStopedZeroPosition = false;
};




} /* namespace real_physics */

#endif /* SOURCE_ENGIE_KINEMATICPHYSICS_BODY_RPPHYSICSOBJECT_H_ */
