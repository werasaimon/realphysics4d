/*
 * rpClipingPoly.cpp
 *
 *  Created on: 2 дек. 2016 г.
 *      Author: wera
 */

#include "rpQuickClippingPolygons.h"
#include <iostream>
using namespace std;

namespace real_physics
{


//*****************************************************************//
bool rpQuickClippingPolygons::isComputeClippingToPoly()
{

	assert(mCountPolygonVertices >= 3 && mCountClipVertices >= 2);


	  Destroy();

	  Vector3 Normal0 = Vector3::planeNormal(mPolygonVertices[0],
											 mPolygonVertices[1],
											 mPolygonVertices[2]);

	  rpPlane rootPlane(Normal0, mPolygonVertices[0]);

      int _NbCountA =  mCountPolygonVertices;
	  int _NbCountB =  mCountClipVertices;

	  const Vector3* _axInputVerticesA = mPolygonVertices;
	  const Vector3* _axInputVerticesB = mClipVertices;



	  int phase = 1;
	  int maxItns = 2 * (_NbCountA + _NbCountB);
	  int inflag = UNKNOWN;

	  bool exceptions = (_NbCountB == 2);

      int &IteratorOut = mOutCountVertices;
	  int linearIterator = 0;

	  Vector3 start_point;

	  Vector3 NormalClipPlane;
	  bool    isLookFaceToFace = true;


	  int end_index_a = 0;
	  int end_index_b = 0;

	  if(_NbCountB >= 3)
	  {

          NormalClipPlane = Vector3::planeNormal( _axInputVerticesB[0],
                                                  _axInputVerticesB[1],
                                                  _axInputVerticesB[2]);

          isLookFaceToFace = (NormalClipPlane.dot(Normal0) >= 0);
	  }

	  /*************************************************/

	  for(int i = 1; ((i <= maxItns) || (phase == 2)); i++)
	  {

		  if (phase == 2)   linearIterator++;
		  if (linearIterator > (maxItns / 2)) break;


		  rpEdge EdgeA = MoveEdgeToIndex( end_index_a, INDICATOR_EDGE_POLY);
		  rpEdge EdgeB = MoveEdgeToIndex( end_index_b, INDICATOR_EDGE_CLIP_POLY);


		  Swap(EdgeA.mA, EdgeA.mB);
		  Swap(EdgeB.mA, EdgeB.mB);


		  if (!isLookFaceToFace)
		  {
			  Swap(EdgeB.mA, EdgeB.mB);
		  }


		  Vector3 EdgeNormalA = (EdgeA.mA - EdgeA.mB).cross(-Normal0);
		  Vector3 EdgeNormalB = (EdgeB.mA - EdgeB.mB).cross(-Normal0);

		  rpPlane planeA(EdgeNormalA, EdgeA.mB );
		  rpPlane planeB(EdgeNormalB, EdgeB.mB );


		  // EdgeA.Draw();
		  // EdgeB.Draw();


		  //planeA.Draw();
		  //planeB.Draw();


		  //**************************************************//

		  Vector3 crossPoint     =  planeA.vIntersectionLineVsPlane(EdgeB);
		  Vector3 crossProjPoint =  rootPlane.vProjectPointToPlane(crossPoint);


		  bool isCrossA = EdgeA.isPointOutsideLine(crossProjPoint);
		  bool isCrossB = EdgeB.isPointOutsideLine(crossPoint);

		  bool isIntersection = ( isCrossA && isCrossB );


		  int a_class = planeB.sideClassifyPointToEdge(EdgeA.mB);
		  int b_class = planeA.sideClassifyPointToEdge(EdgeB.mB);



	      //***************** exceptions **********************//

		  if((phase == 2) &&
		     (isIntersection) &&
		     (start_point == crossPoint) &&
		     (IteratorOut == 1) &&
			 (exceptions))
		  {
			  if ( b_class == LEFT )
			  {
				  addPoint(EdgeB.mA);
			  }
			  else
			  {
				  addPoint(EdgeB.mB);
			  }

			  break;
		  }

		  //**************************************************//

		  if( phase == 2 && (start_point == crossPoint)  && IteratorOut > 1 ) break;

		  //**************************************************//

		  if( isIntersection )
		  {
			  if (phase == 1)
			  {
				      addPoint((start_point = crossPoint));
					  phase = 2;

			  }
			  else if (crossPoint != start_point && phase == 2 )
			  {
				      addPoint(crossPoint);
			  }

		  }

		  //************************************************//


		  Vector3 lineDirA = EdgeA.mA - EdgeA.mB;
		  Vector3 lineDirB = EdgeB.mA - EdgeB.mB;


		  bool bPlaneAIMSaLine = planeB.isAtLookToPoly( lineDirA , a_class);
		  bool aPlaneAIMSbLine = planeA.isAtLookToPoly( lineDirB , b_class);


		  /*************************************************/

		  if( isIntersection )
		  {
			  if (a_class == RIGHT)
			  {
				  inflag = P_IS_INSIDE;
			  }
			  else if (b_class == RIGHT)
			  {
				  inflag = Q_IS_INSIDE;
			  }
			  else
			  {
				  inflag = UNKNOWN;
			  }
		  }


		  /*************************************************/

		  bool isLeftClassification = (a_class == LEFT);

		  int lClassifyIndex = nextMoveIndexToEdges(bPlaneAIMSaLine,
				                                    aPlaneAIMSbLine,
				                                    isLeftClassification,
							                        end_index_a,
										   		    end_index_b,
												    exceptions ,
												    isLookFaceToFace);


		  if( phase == 2)
		  {
			  if( lClassifyIndex == 1)
			  {
				  if ( inflag == Q_IS_INSIDE  )
				  {
					  addPoint(EdgeB.mB);
				  }
			  }
			  else if( lClassifyIndex == 2)
			  {
				  if ( inflag == P_IS_INSIDE &&  _NbCountB >= 3 )
				  {
					  /************************************************/
					  Vector3 NormalClipPlane = Vector3::planeNormal( _axInputVerticesB[0],
								                                      _axInputVerticesB[1],
								                                      _axInputVerticesB[2]);

					  rpPlane ClipPlane(NormalClipPlane, _axInputVerticesB[0]);
					  rpEdge  PolyAdgeSlip( EdgeA.mB , EdgeA.mB + Normal0);
					  Vector3 p = ClipPlane.vIntersectionLineVsPlane(PolyAdgeSlip);

					  addPoint(p);
					  /**********************************************/
				  }
			  }
		  }

	  }


	  /*************************************************/

	  if( phase == 1 )
	   {
	 	  int num_verticesA = _NbCountA;
	 	  int num_verticesB = _NbCountB;

	 	  end_index_a = (end_index_a == num_verticesA || end_index_a < num_verticesA-1) ? 0: end_index_a;
	 	  end_index_b = (end_index_b == num_verticesB || end_index_b < num_verticesB-1) ? 0: end_index_b;


	 	  Vector3 Apoint = _axInputVerticesA[end_index_a];
	 	  Vector3 Bpoint = _axInputVerticesB[end_index_b];


	 	  if (InsidePolygonSAT( Bpoint , _axInputVerticesA, num_verticesA , rootPlane))
	 	  {
	 		  for (int i = 0; i < num_verticesB; i++)
	 		  {
	 			  addPoint( _axInputVerticesB[i] );
	 		  }
	 	  }
	 	  else if ( InsidePolygonSAT(Apoint, _axInputVerticesB , num_verticesB , rootPlane ) && _NbCountB >= 3)
	 	  {
	 		  Vector3 NormalClipPlane = Vector3::planeNormal( _axInputVerticesB[0],
	 				                                          _axInputVerticesB[1],
					                                          _axInputVerticesB[2]);

	 		  rpPlane ClipPlane(NormalClipPlane, _axInputVerticesB[0]);
	 		  for (int i = 0; i < num_verticesA; i++)
	 		  {
	 			  addPoint( ClipPlane.vIntersectionLineVsPlane( rpEdge(_axInputVerticesA[i] ,
	 					                                               _axInputVerticesA[i] + Normal0)));
	 		  }
	 	  }
	   }
	  /*************************************************/

	  return (IteratorOut > 1);
}






const rpEdge rpQuickClippingPolygons::MoveEdgeToIndex(int& end_index , int indicatorEDGE) const
{

    int NbCount = 0;
	const Vector3* elements = NULL;

	if( indicatorEDGE == INDICATOR_EDGE_POLY)
	{

		NbCount  = mCountPolygonVertices;
		elements = mPolygonVertices;

	}
	else if( indicatorEDGE == INDICATOR_EDGE_CLIP_POLY)
	{

		NbCount  = mCountClipVertices;
		elements = mClipVertices;
	}

	assert( elements && NbCount > 0);

	int index0 = 0;
	int index1 = 0;
	CoretionIndex(index0, index1, NbCount , end_index);

	return rpEdge( elements[index0] ,
			       elements[index1] );
}




//****************   Help Function ****************************//

void rpQuickClippingPolygons::CoretionIndex(int& _out0, int& _out1, int _max, int& _i)
{
	int index = (_i >= 0) ? _i : _i + _max;
	int indexation = (index >= 0 && index < _max) ? index : 0;

	if (index >= _max || index < 0)
	{
		_out0 = 0;
		_i = 0;
	}

	_out0 = (indexation >= 0) ? indexation : _max - 1;
	_out1 = (index < _max && index >= 0) ? indexation - 1 : _max - 1;

	if (_out1 < 0)
	{
		_i = 0;
		_out1 = _max - 1;
	}
}



int rpQuickClippingPolygons::nextMoveIndexToEdges( bool bPlaneAIMSLookToLineA,
		                                     bool aPlaneAIMSLookToLineB,
	 	                                     bool isLeftClassification,
											 int& moveIndexA,
											 int& moveIndexB,
											 bool exceptions ,
											 bool isLookFaceToFace)
{

		if(!exceptions)
		{

			if (aPlaneAIMSLookToLineB && bPlaneAIMSLookToLineA)
			{

				if (isLeftClassification)
				{
					MoveIndex(moveIndexA , true);
				}
				else
				{
					MoveIndex(moveIndexB , isLookFaceToFace);
				}

				return 0;
			}
			else if (bPlaneAIMSLookToLineA)
			{

				MoveIndex(moveIndexA , true);
				return 2;

			}
			else if (aPlaneAIMSLookToLineB)
			{

				MoveIndex(moveIndexB , isLookFaceToFace);
				return 1;

			}

			else
			{
				if(isLeftClassification)
				{
					MoveIndex(moveIndexA , true);
				}
				else
				{
					MoveIndex(moveIndexB , isLookFaceToFace);
				}

				return 0;
			}
		}
		else
		{
			MoveIndex(moveIndexA , true);

			return 0;
		}

	return 0;
}





void rpQuickClippingPolygons::MoveIndex(int& index , bool isLookFaceToFace)
{
	 (isLookFaceToFace)? index++ : index--;
}



} /* namespace real_physics */


