/*
 * rpGyroscopic.h
 *
 *  Created on: 16 февр. 2017 г.
 *      Author: wera
 */

#ifndef SRC_PHYSICS_ENGINE_LINEARMATHS_RPGYROSCOPIC_H_
#define SRC_PHYSICS_ENGINE_LINEARMATHS_RPGYROSCOPIC_H_

#include "rpVector3D.h"
#include "rpQuaternion.h"

namespace real_physics
{

template<class T> class rpGyroscopic
{

  private:

	 // -------------------- Attributes -------------------- //

     rpQuaternion<T> mQuaternion;


  public:
             rpGyroscopic( rpQuaternion<T> quaternion );
	virtual ~rpGyroscopic();


	// -------------------- Methods -------------------- //

    void update(rpQuaternion<T> quaternion)
    {
        mQuaternion = quaternion;
    }

	rpVector3D<T> getPitchYawRoll()
	 {
	     // Store the Euler angles in radians
		 rpVector3D<T> pitchYawRoll;

         rpQuaternion<T> q(mQuaternion);

		 T sqw = q.R   * q.R;
		 T sqx = q.V.x * q.V.x;
		 T sqy = q.V.y * q.V.y;
		 T sqz = q.V.z * q.V.z;

		 // If quaternion is normalised the unit is one, otherwise it is the correction factor
		 T unit = sqx + sqy + sqz + sqw;
		 T test = q.V.x * q.V.y + q.V.z * q.R;

		 if (test > 0.4999f * unit)                                // 0.4999f OR 0.5f - EPSILON
		 {
			 // Singularity at north pole
			 pitchYawRoll.y = 2.f * atan2(q.V.x, q.R);             // Yaw
			 pitchYawRoll.x = Pi() * 0.5f;                         // Pitch
			 pitchYawRoll.z = 0.f;                                 // Roll
			 return pitchYawRoll;
		 }
		 else if (test < -0.4999f * unit)                          // -0.4999f OR -0.5f + EPSILON
		 {
			 // Singularity at south pole
			 pitchYawRoll.y = -2.f * atan2(q.V.x, q.R);            // Yaw
			 pitchYawRoll.x = -Pi() * 0.5f;                        // Pitch
			 pitchYawRoll.z = 0.f;                                 // Roll
			 return pitchYawRoll;
		 }
		 else
		 {
			 pitchYawRoll.y = atan2(2.f * q.V.y * q.R - 2.f * q.V.x * q.V.z,  sqx - sqy - sqz + sqw);      // Yaw
			 pitchYawRoll.x =  asin(2.f * test / unit);                                                    // Pitch
			 pitchYawRoll.z = atan2(2.f * q.V.x * q.R - 2.f * q.V.y * q.V.z, -sqx + sqy - sqz + sqw);      // Roll
		 }

		 return pitchYawRoll;
	 }


	rpVector3D<T> getEulerAngle(bool homogenous=true)
    {
		rpVector3D<T> euler;
		rpVector3D<T> V;

		V.x = mQuaternion->x;
		V.y = mQuaternion->y;
		V.z = mQuaternion->z;
		T R = mQuaternion->w;

		 if(homogenous)
		 {
			 euler.y   =  atan2(2*V.y*R - 2*V.x*V.z, 1 - 2*V.y*V.y - 2*V.z*V.z);
			 euler.x   =   asin(2*V.x*V.y + 2*V.z*R);
			 euler.z   =  atan2(2*V.x*R - 2*V.y*V.z, 1 - 2*V.x*V.x - 2*V.z*V.z);
		 }
		 else
		 {
			 euler.y = atan2(2 * V.y * R - 2 * V.x * V.z, R * R + V.x * V.x - V.y * V.y - V.z * V.z);
			 euler.x =  asin(2 * V.x * V.y + 2 * V.z * R );
			 euler.z = atan2(2*V.x*R - 2*V.y*V.z, R * R + V.y*V.y  - V.x*V.x - V.z*V.z);
		 }
		 return euler;
    }

};

} /* namespace real_physics */



#endif /* SRC_PHYSICS_ENGINE_LINEARMATHS_RPGYROSCOPIC_H_ */
