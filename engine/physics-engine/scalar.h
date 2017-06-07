/*
 * scalar.h
 *
 *  Created on: 14 янв. 2017 г.
 *      Author: wera
 */

#ifndef SOURCE_ENGIE_SCALAR_H_
#define SOURCE_ENGIE_SCALAR_H_


namespace real_physics
{

#if defined(IS_DOUBLE_PRECISION_ENABLED)   // If we are compiling for double precision
    typedef double scalar;
#else                                   // If we are compiling for single precision
    typedef float scalar;
#endif
}



#endif /* SOURCE_ENGIE_SCALAR_H_ */
