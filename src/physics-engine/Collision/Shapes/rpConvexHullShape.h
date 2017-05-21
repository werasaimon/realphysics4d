/*
 * rpConvexHullShape.h
 *
 *  Created on: 22 нояб. 2016 г.
 *      Author: wera
 */

#ifndef SOURCE_ENGIE_COLLISION_SHAPES_RPCONVEXHULLSHAPE_H_
#define SOURCE_ENGIE_COLLISION_SHAPES_RPCONVEXHULLSHAPE_H_

#include "rpConvexShape.h"
#include "../NarrowPhase/GJK/rpGJKAlgorithm.h"
#include  "../NarrowPhase/rpNarrowPhaseGjkEpaAlgorithm.h"
#include "../../Geometry/geometry.h"

namespace real_physics
{



    struct rpModelConvexHull
	{
    	rpModelConvexHull( const Vector3 *axVertices , uint NbCount )
    	{

    		std::vector<Vector3> mOldVertices;
            mOldVertices.resize(NbCount);
            for (uint i = 0; i < NbCount; ++i)
            {
            	mOldVertices[i] = axVertices[i];
            }

    		mConvexHull = mQuickHull.getConvexHull( mOldVertices , true, false);
    	}


    	rpModelConvexHull( std::vector<Vector3> Vertices )
    	{
    		std::vector<Vector3> mOldVertices = Vertices;
    		mConvexHull = mQuickHull.getConvexHull( mOldVertices  , true, false);
    	}


    	rpQuickHull<scalar>  mQuickHull;
    	rpConvexHull<scalar> mConvexHull;

	};





	class rpConvexHullShape: public rpConvexShape
	{


		private:


		    // -------------------- Attributes -------------------- //

			rpQuickHull<scalar>  *mQuickHull;
			rpConvexHull<scalar> *mConvexHull;


		protected :

			/// Private copy-constructor
			rpConvexHullShape(const rpConvexHullShape& shape);

			/// Private assignment operator
			rpConvexHullShape& operator=(const rpConvexHullShape& shape);




			/// Return a local support interval ( minimum , maximum )
			void getIntervalLocal(const Vector3 &xAxis, scalar &min, scalar &max) const;


			/// Return a local support point in a given direction without the object margin
			virtual Vector3 getLocalSupportPointWithoutMargin(const Vector3& direction,
					                                          void** cachedCollisionData) const;

			/// Return true if a point is inside the collision shape
			virtual bool testPointInside(const Vector3& localPoint, rpProxyShape* proxyShape) const;

			/// Raycast method with feedback information
			virtual bool raycast(const Ray& ray, RaycastInfo& raycastInfo, rpProxyShape* proxyShape) const;

			/// Return the number of bytes used by the collision shape
			virtual size_t getSizeInBytes() const;






		public:

            rpConvexHullShape( rpModelConvexHull* initHull , scalar margin = OBJECT_MARGIN );

			virtual ~rpConvexHullShape();



			/// Set the scaling vector of the collision shape
			virtual void setLocalScaling(const Vector3& scaling);

			/// Return the local bounds of the shape in x, y and z directions
			virtual void getLocalBounds(Vector3& min, Vector3& max) const;

			/// Return the local inertia tensor of the collision shape
			virtual void computeLocalInertiaTensor(Matrix3x3& tensor, scalar mass) const;


	};

} /* namespace real_physics */

#endif /* SOURCE_ENGIE_COLLISION_SHAPES_RPCONVEXHULLSHAPE_H_ */
