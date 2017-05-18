/*
 * rpCollisionShapeInfo.h
 *
 *  Created on: 18 нояб. 2016 г.
 *      Author: wera
 */

#ifndef SOURCE_ENGIE_COLLISION_RPCOLLISIONSHAPEINFO_H_
#define SOURCE_ENGIE_COLLISION_RPCOLLISIONSHAPEINFO_H_


#include "rpProxyShape.h"
#include  "Shapes/rpCollisionShape.h"

namespace real_physics
{



class OverlappingPair;


// Class rpCollisionShapeInfo
/**
 * This structure regroups different things about a collision shape. This is
 * used to pass information about a collision shape to a collision algorithm.
 */
struct rpCollisionShapeInfo
{

    public:

    /// Broadphase overlapping pair
    //OverlappingPair* overlappingPair;

    /// Proxy shape
    //rpProxyShape* proxyShape;

    /// Pointer to the collision shape
    const rpCollisionShape* collisionShape;

    /// Transform that maps from collision shape local-space to world-space
    const Transform shapeToWorldTransform;

    /// Cached collision data of the proxy shape
    void** cachedCollisionData;

    /// Constructor
    rpCollisionShapeInfo(const rpCollisionShape* _CollisionShape,
                         const Transform& shapeLocalToWorldTransform,
                         void** cachedData)
        : collisionShape(_CollisionShape),
          shapeToWorldTransform(shapeLocalToWorldTransform) ,
          cachedCollisionData(cachedData)
    {


    }


    // Return a local support point in a given direction with the object margin
    Vector3 getLocalSupportPointWithMargin(const Vector3 &direction ) const
    {
        return collisionShape->getLocalSupportPointWithMarginn(direction);

    }


    const Transform& getWorldTransform() const
    {
        return shapeToWorldTransform;
    }
};

} /* namespace real_physics */

#endif /* SOURCE_ENGIE_COLLISION_RPCOLLISIONSHAPEINFO_H_ */
