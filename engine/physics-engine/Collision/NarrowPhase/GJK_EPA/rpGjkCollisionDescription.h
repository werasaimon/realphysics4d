/*
 * rpGjkCollisionDescription.h
 *
 *  Created on: 19 нояб. 2016 г.
 *      Author: wera
 */

#ifndef SOURCE_ENGIE_COLLISION_NARROWPHASE_RPGJKCOLLISIONDESCRIPTION_H_
#define SOURCE_ENGIE_COLLISION_NARROWPHASE_RPGJKCOLLISIONDESCRIPTION_H_


#include "../../../LinearMaths/mathematics.h"

namespace real_physics
{

struct btGjkCollisionDescription
{
    Vector3	    m_firstDir;
    int			m_maxGjkIterations;
    scalar	    m_maximumDistanceSquared;
    scalar	    m_gjkRelError2;


    btGjkCollisionDescription()
    :m_firstDir(0,1,0),
    m_maxGjkIterations(100),
    m_maximumDistanceSquared(1e30f),
    m_gjkRelError2(1.0e-6)
    {
    }


    virtual ~btGjkCollisionDescription()
    {
    }
};
} /* namespace real_physics */

#endif /* SOURCE_ENGIE_COLLISION_NARROWPHASE_RPGJKCOLLISIONDESCRIPTION_H_ */
