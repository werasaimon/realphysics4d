#include "MeshPlane.h"

using namespace utility_engine;

MeshPlane::MeshPlane(float width, float height)
:Mesh(PRIMITIVE_MESH_PLANE)
{
	initialization( width , height );
}

MeshPlane::~MeshPlane()
{
	// TODO Auto-generated destructor stub
}


/* namespace utility_engine */
