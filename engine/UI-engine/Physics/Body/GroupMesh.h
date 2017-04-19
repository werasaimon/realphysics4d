#ifndef GROUPMESH_H
#define GROUPMESH_H

#include "../../Geometry/Mesh/Mesh.h"
#include "Convert.h"

namespace utility_engine
{

    struct ElementMesh
    {
        // -------------------- Attributes -------------------- //
        Mesh     *mMesh;
        Matrix4   mInitTransform;

        ElementMesh( Mesh *mesh , Matrix4 initTransform )
        : mMesh(mesh) , mInitTransform(initTransform)
        {

        }

        void updateTransformMatrix( Matrix4 matrix )
        {
            mMesh->setToIdentity();
            mMesh->setTransformMatrix(mInitTransform * matrix);
        }

    };


    class GroupMesh
    {

        private:

            //--------------------- Attributes ---------------------//

            /// Elements meshe
            std::vector<ElementMesh> mElements;

        public:

                     GroupMesh();
            virtual ~GroupMesh();


            //--------------------- Method --------------------------//

            /// Add initlization mesh to matrix
            void addInitMesh( Mesh *mesh , const Matrix4& initTransform )
            {
               mElements.push_back( ElementMesh(mesh , initTransform)  );
            }


            /// Update real-time physics translations
            void updateTransform( const Matrix4& transform )
            {
                for (int i = 0; i < mElements.size(); ++i)
                {
                    mElements[i].updateTransformMatrix( transform );
                }
            }

            /// Destroy :: ( clear )
            void destroy()
            {
                mElements.clear();
            }

            //--------------------- Value -------------------------//

            /// Elements meshe
            std::vector<ElementMesh>& getElements()
            {
                return mElements;
            }


    };

}

#endif // GROUPMESH_H
