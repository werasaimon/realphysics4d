/*
 * memory
 *
 *  Created on: 21 нояб. 2016 г.
 *      Author: wera
 */

#ifndef SOURCE_ENGIE_MEMORY_MEMORY_H_
#define SOURCE_ENGIE_MEMORY_MEMORY_H_


#include "rpStack.h"
//#include "rpMemoryAllocator.h"
#include "rpAlignedallocator.h"
#include "rpAlignedobjectarray.h"
#include "rpList.h"


namespace real_physics
{

 template<class T> using rpObjectArray  = b3AlignedObjectArray<T>;

}


#endif /* SOURCE_ENGIE_MEMORY_MEMORY_H_ */
