CXXFLAGS =	-O2 -g -Wall -fmessage-length=0 -std=c++11
CC=g++

OBJS =		realphysics4d_make.o

LIBS = -lGL -lGLU -lglut -lGLEW 

TARGET =	bin/realphysics4d_make

#----------------------- core main -------------------------#
LDFLAGS =
SOURCES   = src/main.cpp
SOURCES  += src/examples/GLViewer.cpp
SOURCES  += src/examples/UnitSceneDemo.cpp


#-------------------- physics-engie ------------------------#
SOURCES  += src/engine/physics-engine/Collision/Body/rpBody.cpp 
SOURCES  += src/engine/physics-engine/Collision/Body/rpCollisionBody.cpp 
SOURCES  += src/engine/physics-engine/Collision/BroadPhase/rbBroadPhaseAlgorithm.cpp 
SOURCES  += src/engine/physics-engine/Collision/BroadPhase/rpDynamicAABBTree.cpp 
SOURCES  += src/engine/physics-engine/Collision/ContactManiflod/rpContactManifold.cpp 
SOURCES  += src/engine/physics-engine/Collision/ContactManiflod/rpContactManifoldSet.cpp 
SOURCES  += src/engine/physics-engine/Collision/ContactManiflod/rpContactPoint.cpp 
SOURCES  += src/engine/physics-engine/Collision/ContactManiflod/rpGenerationContactManiflodSet.cpp 
SOURCES  += src/engine/physics-engine/Collision/NarrowPhase/GJK_EPA/rpGjkEpa.cpp 
SOURCES  += src/engine/physics-engine/Collision/NarrowPhase/rpNarrowPhaseCollisionAlgorithm.cpp 
SOURCES  += src/engine/physics-engine/Collision/Shapes/rpAABB.cpp 
SOURCES  += src/engine/physics-engine/Collision/Shapes/rpBoxShape.cpp 
SOURCES  += src/engine/physics-engine/Collision/Shapes/rpCollisionShape.cpp 
SOURCES  += src/engine/physics-engine/Collision/Shapes/rpConvexHullShape.cpp 
SOURCES  += src/engine/physics-engine/Collision/Shapes/rpConvexShape.cpp 
SOURCES  += src/engine/physics-engine/Collision/Shapes/rpSphereShape.cpp 
SOURCES  += src/engine/physics-engine/Collision/Shapes/rpTriangleShape.cpp 
SOURCES  += src/engine/physics-engine/Collision/rpCollisionDetection.cpp 
SOURCES  += src/engine/physics-engine/Collision/rpCollisionWorld.cpp 
SOURCES  += src/engine/physics-engine/Collision/rpOverlappingPair.cpp 
SOURCES  += src/engine/physics-engine/Collision/rpProxyShape.cpp 
SOURCES  += src/engine/physics-engine/Collision/rpRaycastInfo.cpp 
SOURCES  += src/engine/physics-engine/Dynamics/Body/rpPhysicsBody.cpp 
SOURCES  += src/engine/physics-engine/Dynamics/Body/rpPhysicsObject.cpp 
SOURCES  += src/engine/physics-engine/Dynamics/Body/rpRigidPhysicsBody.cpp 
SOURCES  += src/engine/physics-engine/Dynamics/Joint/JointAngle/rpAngleJoint.cpp 
SOURCES  += src/engine/physics-engine/Dynamics/Joint/rpBallAndSocketJoint.cpp 
SOURCES  += src/engine/physics-engine/Dynamics/Joint/rpDistanceJoint.cpp 
SOURCES  += src/engine/physics-engine/Dynamics/Joint/rpFixedJoint.cpp 
SOURCES  += src/engine/physics-engine/Dynamics/Joint/rpHingeJoint.cpp 
SOURCES  += src/engine/physics-engine/Dynamics/Joint/rpJoint.cpp 
SOURCES  += src/engine/physics-engine/Dynamics/Joint/rpSliderJoint.cpp 
SOURCES  += src/engine/physics-engine/Dynamics/Material/rpPhysicsMaterial.cpp 
SOURCES  += src/engine/physics-engine/Dynamics/Solver/rpContactSolver.cpp 
SOURCES  += src/engine/physics-engine/Dynamics/Solver/rpSequentialImpulseObjectSolver.cpp 
SOURCES  += src/engine/physics-engine/Dynamics/rpDynamicsWorld.cpp 
SOURCES  += src/engine/physics-engine/Dynamics/rpTimer.cpp 
SOURCES  += src/engine/physics-engine/Geometry/QuickHull/QuickHull.cpp 
SOURCES  += src/engine/physics-engine/Geometry/rpGrahamScan2dConvexHull.cpp 
SOURCES  += src/engine/physics-engine/Geometry/rpPolygonClipping.cpp 
SOURCES  += src/engine/physics-engine/LinearMaths/rpGyroscopic.cpp 
SOURCES  += src/engine/physics-engine/LinearMaths/rpLorentzContraction.cpp 
SOURCES  += src/engine/physics-engine/LinearMaths/rpMatrix2x2.cpp 
SOURCES  += src/engine/physics-engine/LinearMaths/rpMatrix3x3.cpp 
SOURCES  += src/engine/physics-engine/LinearMaths/rpMatrix4x4.cpp 
SOURCES  += src/engine/physics-engine/LinearMaths/rpMinkowskiVector4.cpp 
SOURCES  += src/engine/physics-engine/LinearMaths/rpProjectPlane.cpp 
SOURCES  += src/engine/physics-engine/LinearMaths/rpQuaternion.cpp 
SOURCES  += src/engine/physics-engine/LinearMaths/rpTransform.cpp 
SOURCES  += src/engine/physics-engine/LinearMaths/rpVector2D.cpp 
SOURCES  += src/engine/physics-engine/LinearMaths/rpVector3D.cpp 
SOURCES  += src/engine/physics-engine/Memory/rpAlignedallocator.cpp 
SOURCES  += src/engine/physics-engine/Collision/NarrowPhase/rpNarrowPhaseGjkEpaAlgorithm.cpp 
SOURCES  += src/engine/physics-engine/Collision/NarrowPhase/GJK/Simplex.cpp 
SOURCES  += src/engine/physics-engine/Collision/NarrowPhase/MPR/rpMPRAlgorithm.cpp 
SOURCES  += src/engine/physics-engine/Collision/NarrowPhase/GJK/rpGJKAlgorithm.cpp 
SOURCES  += src/engine/physics-engine/Collision/NarrowPhase/GJK_EPA/VoronoiSimplex/rpVoronoiSimplexSolver.cpp 
SOURCES  += src/engine/physics-engine/Collision/NarrowPhase/rpNarrowPhaseMprAlgorithm.cpp



#----------------- opengl-utility -------------------------#
SOURCES  += src/engine/UI-engine/maths/glmath.cpp
SOURCES  += src/engine/UI-engine/maths/Vector3.cpp
SOURCES  += src/engine/UI-engine/maths/Matrix4.cpp

SOURCES  += src/engine/UI-engine/Object/Object3D.cpp
SOURCES  += src/engine/UI-engine/Camera/camera.cpp
SOURCES  += src/engine/UI-engine/Light/Light.cpp

SOURCES  += src/engine/UI-engine/Mesh/Mesh.cpp
SOURCES  += src/engine/UI-engine/Mesh/Primitive/MeshTriangle.cpp
SOURCES  += src/engine/UI-engine/Mesh/Primitive/MeshPlane.cpp
SOURCES  += src/engine/UI-engine/Mesh/Primitive/MeshBox.cpp
SOURCES  += src/engine/UI-engine/Mesh/Loaders/MeshReadFile3DS.cpp

SOURCES  += src/engine/UI-engine/Texture/Texture2D.cpp
SOURCES  += src/engine/UI-engine/Texture/TextureReaderWriter.cpp

SOURCES  += src/engine/UI-engine/Physics/Body/GroupMesh.cpp
SOURCES  += src/engine/UI-engine/Physics/Body/UltimatePhysicsBody.cpp
SOURCES  += src/engine/UI-engine/Physics/Joint/UltimateJoint.cpp
SOURCES  += src/engine/UI-engine/Physics/DynamicsWorld.cpp

SOURCES  += src/engine/UI-engine/OpenGL/Shader.cpp
SOURCES  += src/engine/UI-engine/OpenGL/FrameBufferObject.cpp
SOURCES  += src/engine/UI-engine/OpenGL/UtilityOpenGLMesh.cpp

OBJS=$(SOURCES:%.cpp=%.o) 
EXECUTABLE=hello
	

$(TARGET):	$(OBJS)
	$(CXX) -o $(TARGET) $(OBJS) $(LIBS)

all:	$(TARGET)

clean:
	rm -f $(OBJS) $(TARGET)