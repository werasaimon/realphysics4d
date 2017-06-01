/*
 * rpLorentzContraction.h
 *
 *  Created on: 25 февр. 2017 г.
 *      Author: wera
 */

#ifndef SRC_PHYSICS_ENGINE_LINEARMATHS_RPLORENTZCONTRACTION_H_
#define SRC_PHYSICS_ENGINE_LINEARMATHS_RPLORENTZCONTRACTION_H_

#include "rpRelativityFunction.h"
#include "rpMatrix2x2.h"
#include "rpMatrix3x3.h"
#include "rpMatrix4x4.h"

namespace real_physics
{


/**********************************************
 *  The shortening of a moving body in the direction of its motion,
 *  especially at speeds close to that of light.
 **********************************************/
template<class T> class rpLorentzContraction
{

  private:

	//-------------------- Attributes --------------------//

	            T  mLorentzFactor;
	rpMatrix3x3<T> mLorentzLengthTransform;



  public:

	rpLorentzContraction()
    :mLorentzLengthTransform(rpMatrix3x3<T>::identity()) ,
     mLorentzFactor(1.0)
	{

	}

	/*****************************************************
	 *  Help info to web site:  https://arxiv.org/pdf/1103.0156.pdf
	 *****************************************************/
	void updateDisplacementBoost( const rpVector3D<T>& relativityVelocity )
	{

		/// Factor gamma relativity
		T inversLoretzFactor = gammaInvertFunction(relativityVelocity);

		/// Gamma invert
		T gamma = (inversLoretzFactor - 1.0);

		/// OrtoBsis
		rpVector3D<T> n = relativityVelocity.getUnit();

		/// Lorentz matrix3x3 boost
		T inversBoostMatrix[3][3];

		inversBoostMatrix[0][0] = 1.0+(gamma*((n.x * n.x)));
		inversBoostMatrix[1][0] =     (gamma*((n.y * n.x)));
		inversBoostMatrix[2][0] =     (gamma*((n.z * n.x)));


		inversBoostMatrix[0][1] =     (gamma*((n.x * n.y)));
		inversBoostMatrix[1][1] = 1.0+(gamma*((n.y * n.y)));
		inversBoostMatrix[2][1] =     (gamma*((n.z * n.y)));;


		inversBoostMatrix[0][2] =     (gamma*((n.x * n.z)));
		inversBoostMatrix[1][2] =     (gamma*((n.y * n.z)));
		inversBoostMatrix[2][2] = 1.0+(gamma*((n.z * n.z)));


		/// Push
		mLorentzFactor          =                inversLoretzFactor;
		mLorentzLengthTransform = rpMatrix3x3<T>(inversBoostMatrix);

	}


	T getLorentzFactor() const
	{
		return mLorentzFactor;
	}

	rpVector3D<T> getLorentzHalfSize() const
	{
		return rpVector3D<T>(mLorentzLengthTransform[0][0],
				             mLorentzLengthTransform[1][1],
				             mLorentzLengthTransform[2][2]);
	}

	rpMatrix3x3<T> getLorentzMatrix() const
	{
		return mLorentzLengthTransform;
	}
};


} /* namespace real_physics */



#endif /* SRC_PHYSICS_ENGINE_LINEARMATHS_RPLORENTZCONTRACTION_H_ */
