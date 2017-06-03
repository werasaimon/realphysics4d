/*
 * MeshBox.cpp
 *
 *  Created on: 6 февр. 2017 г.
 *      Author: wera
 */

#include "MeshBox.h"

namespace utility_engine
{

  MeshBox::MeshBox( const Vector3 halfSize )
 :Mesh(PRIMITIVE_MESH_BOX) ,
  mHalfSize(halfSize)
{
    this->initialization(halfSize);
}



const Vector3 MeshBox::halfSize() const
{
    return mHalfSize;
}

} /* namespace utility_engine */
