#include "MeshPlane.h"

using namespace utility_engine;

MeshPlane::MeshPlane(float width, float height)
:Mesh(PRIMITIVE_MESH_PLANE)
{
	initialization( width , height );
}



/* namespace utility_engine */
