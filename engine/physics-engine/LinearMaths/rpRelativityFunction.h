/*
 * rpRelativityFunction.h
 *
 *  Created on: 24 февр. 2017 г.
 *      Author: wera
 */

#ifndef SRC_PHYSICS_ENGINE_LINEARMATHS_RPRELATIVITYFUNCTION_H_
#define SRC_PHYSICS_ENGINE_LINEARMATHS_RPRELATIVITYFUNCTION_H_

#include "rpVector3D.h"
#include "rpVector2D.h"

namespace real_physics
{


    //---------------------- 3D function ---------------------//

	template <typename T> T betaFunction( const rpVector3D<T>& v )
	{
	  return T( v / LIGHT_MAX_VELOCITY_C);
	}

	template <typename T> T gammaFunction( const rpVector3D<T>& v )
	{
       T gamma =  Sqrt(T(1.0) - (v.dot(v) / (LIGHT_MAX_VELOCITY_C * LIGHT_MAX_VELOCITY_C)));
         gamma = (gamma > MACHINE_EPSILON) ? gamma : MACHINE_EPSILON;
         return T(1.0) / gamma;
	}

	template <typename T> T gammaInvertFunction( const rpVector3D<T>& v )
	{
       T gamma = Sqrt(T(1.0) - (v.dot(v) / (LIGHT_MAX_VELOCITY_C * LIGHT_MAX_VELOCITY_C)));
         gamma = (gamma > MACHINE_EPSILON) ? gamma : MACHINE_EPSILON;
         return   gamma;
	}


	//---------------------- 2D function ---------------------//

/**
	template <typename T> T betaFunction( const rpVector2D<T>& v )
	{
	  return T( v / LIGHT_MAX_VELOCITY_C);
	}

	template <typename T> T gammaFunction( const rpVector2D<T>& v )
	{
	  return T(1.0) / sqrt(T(1.0) - (v.dot(v) / (LIGHT_MAX_VELOCITY_C * LIGHT_MAX_VELOCITY_C)));
	}

	template <typename T> T gammaInvertFunction( const rpVector2D<T>& v )
	{
	   return sqrt(T(1.0) - (v.dot(v) / (LIGHT_MAX_VELOCITY_C * LIGHT_MAX_VELOCITY_C)));
    }
/**/


}


#endif /* SRC_PHYSICS_ENGINE_LINEARMATHS_RPRELATIVITYFUNCTION_H_ */
