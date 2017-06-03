/*
 * UnitScene.h
 *
 *  Created on: 16 нояб. 2016 г.
 *      Author: wera
 */

#ifndef UNITSCENE_H_
#define UNITSCENE_H_

#include "GLViewer.h"

class UnitScene
{
   public:

	// Pointer to the viewer
	Viewer* mViewer;

	UnitScene( Viewer* viewer = NULL )
	: mViewer(viewer)
	{

	}

	virtual bool Init() = 0;
	virtual void Render(float FrameTime) = 0;
	virtual void keyboard( unsigned char key ) = 0;
	virtual void Destroy() = 0;


};


#endif /* UNITSCENE_H_ */
