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



#include "Camera/camera.h"

#include "Mesh/Mesh.h"
#include "Mesh/Primitive/MeshPlane.h"
#include "Mesh/Primitive/MeshTriangle.h"
#include "Mesh/Primitive/MeshBox.h"
#include "Mesh/Loaders/MeshReadFile3DS.h"

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
