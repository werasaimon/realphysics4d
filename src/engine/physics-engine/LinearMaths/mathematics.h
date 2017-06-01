/*
 * rpMathematics.h
 *
 *  Created on: 15 нояб. 2016 г.
 *      Author: wera
 */

#ifndef LINEARMATHS_RPMATHEMATICS_H_
#define LINEARMATHS_RPMATHEMATICS_H_


#include  "rpLinearMtah.h"
#include  "rpVector2D.h"
#include  "rpVector3D.h"
#include  "rpMatrix2x2.h"
#include  "rpMatrix3x3.h"
#include  "rpMatrix4x4.h"
#include  "rpQuaternion.h"
#include  "rpTransform.h"
#include  "rpTransformUtil.h"
#include  "rpRay.h"
#include  "rpGyroscopic.h"
#include  "rpRelativityFunction.h"
#include  "rpMinkowskiVector4.h"
#include  "rpLorentzContraction.h"
#include  "rpProjectPlane.h"


#include <limits>


namespace real_physics
{

   typedef scalar scalar_type;


   typedef rpProjectPlane<scalar_type>        PojectPlane;
   typedef rpRay<scalar_type>                 Ray;
   typedef rpVector2D<scalar_type>            Vector2;
   typedef rpVector3D<scalar_type>            Vector3;
   typedef rpMatrix2x2<scalar_type>           Matrix2x2;
   typedef rpMatrix3x3<scalar_type>           Matrix3x3;
   typedef rpMatrix4x4<scalar_type>           Matrix4x4;
   typedef rpQuaternion<scalar_type>          Quaternion;
   typedef rpTransform<scalar_type>           Transform;
   typedef rpTransformUtil<scalar_type>       TransformUtil;
   typedef rpGyroscopic<scalar_type>          Gyroscopic;
   typedef rpMinkowskiVector4<scalar_type>    MinkowskiVector4;
   typedef rpLorentzContraction<scalar_type>  LorentzContraction;


	const scalar DECIMAL_SMALLEST = -std::numeric_limits<scalar>::max();
	const scalar DECIMAL_LARGEST  =  std::numeric_limits<scalar>::max();


}


#endif /* LINEARMATHS_RPMATHEMATICS_H_ */
