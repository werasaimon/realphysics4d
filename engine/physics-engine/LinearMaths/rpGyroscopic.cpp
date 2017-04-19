/*
 * rpGyroscopic.cpp
 *
 *  Created on: 16 февр. 2017 г.
 *      Author: wera
 */

#include "rpGyroscopic.h"

namespace real_physics
{


template<class T> rpGyroscopic<T>::rpGyroscopic(rpQuaternion<T> quaternion)
: mQuaternion( quaternion )
{

}

template<class T> rpGyroscopic<T>::~rpGyroscopic()
{

}



} /* namespace real_physics */
