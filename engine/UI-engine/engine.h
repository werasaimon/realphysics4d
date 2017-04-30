/*
 * engine.h
 *
 *  Created on: 2 апр. 2017 г.
 *      Author: wera
 */

#ifndef SRC_ELEMENTS_ENGINE_ENGINE_H_
#define SRC_ELEMENTS_ENGINE_ENGINE_H_


// Libraries
#include "Shader/Shader.h"

#include "Texture/Texture2D.h"

#include "Light/Light.h"



#include "Geometry/Camera/camera.h"

#include "Geometry/Mesh/Mesh.h"
#include "Geometry/Mesh/Primitive/MeshPlane.h"
#include "Geometry/Mesh/Primitive/MeshTriangle.h"
#include "Geometry/Mesh/Primitive/MeshBox.h"
#include "Geometry/Mesh/Loaders/MeshReadFile3DS.h"

#include "maths/Color.h"
#include "maths/Vector2.h"
#include "maths/Vector3.h"
#include "maths/Vector4.h"
#include "maths/Matrix4.h"
#include "maths/Matrix3.h"

#include "Physics/physics.h"


namespace engine = utility_engine;
namespace realphysics_engine = utility_engine;

#endif /* SRC_ELEMENTS_ENGINE_ENGINE_H_ */
