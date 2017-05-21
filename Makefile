CXXFLAGS =	-O2 -g -Wall -fmessage-length=0 -std=c++11

OBJS =		realphysics-engie.o

CC=g++

LIBS = -lGL -lGLU -lglut -lGLEW 

TARGET =	bin/realphysics-engine

#----------------------- core main -------------------------#
LDFLAGS =
SOURCES   = src/main.cpp
SOURCES  += src/examples/GLViewer.cpp
SOURCES  += src/examples/UnitSceneDynamics.cpp

#-------------------- physics-engie ------------------------#
#SOURCES  += src/physics-engine/Memory/rpAlignedallocator.cpp
#
#SOURCES  += src/physics-engine/LinearMaths/rpVector2D.cpp
#SOURCES  += src/physics-engine/LinearMaths/rpVector3D.cpp
#SOURCES  += src/physics-engine/LinearMaths/rpMatrix2x2.cpp
#SOURCES  += src/physics-engine/LinearMaths/rpMatrix3x3.cpp
#SOURCES  += src/physics-engine/LinearMaths/rpMatrix4x4.cpp
#SOURCES  += src/physics-engine/LinearMaths/rpQuaternion.cpp
#SOURCES  += src/physics-engine/LinearMaths/rpTransform.cpp
#SOURCES  += src/physics-engine/LinearMaths/rpMinkowskiVector4.cpp
#SOURCES  += src/physics-engine/LinearMaths/rpLorentzContraction.cpp
#SOURCES  += src/physics-engine/LinearMaths/rpProjectPlane.cpp
#SOURCES  += src/physics-engine/LinearMaths/rpGyroscopic.cpp
#
#
#
#SOURCES  += src/physics-engine/Geometry/QuickHull/QuickHull.cpp
#SOURCES  += src/physics-engine/Geometry/rpGrahamScan2dConvexHull.cpp
#SOURCES  += src/physics-engine/Geometry/rpPolygonClipping.cpp
#
#SOURCES  += src/physics-engine/Collision/Body/rpBody.cpp
#SOURCES  += src/physics-engine/Collision/Body/rpCollisionBody.cpp
#
#SOURCES  += src/physics-engine/Collision/BroadPhase/rbBroadPhaseAlgorithm.cpp
#SOURCES  += src/physics-engine/Collision/BroadPhase/rpDynamicAABBTree.cpp
#SOURCES  += src/physics-engine/Collision/ContactManiflod/rpContactManifold.cpp
#SOURCES  += src/physics-engine/Collision/ContactManiflod/rpContactManifoldSet.cpp
#SOURCES  += src/physics-engine/Collision/ContactManiflod/rpContactPoint.cpp
#SOURCES  += src/physics-engine/Collision/ContactManiflod/rpGenerationContactManiflodSet.cpp
#
#SOURCES  += src/physics-engine/Collision/NarrowPhase/GJK_EPA/rpGjkEpa.cpp
#SOURCES  += src/physics-engine/Collision/NarrowPhase/GJK_EPA/rpGJKEPAAlgorithm.cpp
#SOURCES  += src/physics-engine/Collision/NarrowPhase/GJK_EPA/rpVoronoiSimplexSolver.cpp
#
#SOURCES  += src/physics-engine/Collision/NarrowPhase/EPA/rpEdgeEPA.cpp
#SOURCES  += src/physics-engine/Collision/NarrowPhase/EPA/rpEPAAlgorithm.cpp
#SOURCES  += src/physics-engine/Collision/NarrowPhase/EPA/rpTriangleEPA.cpp
#SOURCES  += src/physics-engine/Collision/NarrowPhase/EPA/rpTrianglesStore.cpp
#
#SOURCES  += src/physics-engine/Collision/NarrowPhase/GJK/rpGJKAlgorithm.cpp
#SOURCES  += src/physics-engine/Collision/NarrowPhase/GJK/rpSimplex.cpp
#
#SOURCES  += src/physics-engine/Collision/NarrowPhase/rpNarrowPhaseCollisionAlgorithm.cpp
#
#SOURCES  += src/physics-engine/Collision/Shapes/rpAABB.cpp
#SOURCES  += src/physics-engine/Collision/Shapes/rpBoxShape.cpp
#SOURCES  += src/physics-engine/Collision/Shapes/rpCollisionShape.cpp
#SOURCES  += src/physics-engine/Collision/Shapes/rpConvexHullShape.cpp
#SOURCES  += src/physics-engine/Collision/Shapes/rpConvexShape.cpp
#SOURCES  += src/physics-engine/Collision/Shapes/rpSphereShape.cpp
#SOURCES  += src/physics-engine/Collision/Shapes/rpTriangleShape.cpp
#
#SOURCES  += src/physics-engine/Collision/rpCollisionDetection.cpp
#SOURCES  += src/physics-engine/Collision/rpCollisionWorld.cpp
#SOURCES  += src/physics-engine/Collision/rpOverlappingPair.cpp
#SOURCES  += src/physics-engine/Collision/rpProxyShape.cpp
#SOURCES  += src/physics-engine/Collision/rpRaycastInfo.cpp
#
#SOURCES  += src/physics-engine/Dynamics/Material/rpPhysicsMaterial.cpp
#
#SOURCES  += src/physics-engine/Dynamics/Body/rpPhysicsBody.cpp
#SOURCES  += src/physics-engine/Dynamics/Body/rpPhysicsObject.cpp
#SOURCES  += src/physics-engine/Dynamics/Body/rpRigidPhysicsBody.cpp
#
#SOURCES  += src/physics-engine/Dynamics/Joint/rpJoint.cpp
#SOURCES  += src/physics-engine/Dynamics/Joint/rpBallAndSocketJoint.cpp
#SOURCES  += src/physics-engine/Dynamics/Joint/rpDistanceJoint.cpp
#SOURCES  += src/physics-engine/Dynamics/Joint/rpFixedJoint.cpp
#SOURCES  += src/physics-engine/Dynamics/Joint/rpHingeJoint.cpp
#SOURCES  += src/physics-engine/Dynamics/Joint/rpSliderJoint.cpp
#SOURCES  += src/physics-engine/Dynamics/Joint/JointAngle/rpAngleJoint.cpp
#
#SOURCES  += src/physics-engine/Dynamics/Solver/rpContactSolver.cpp
#SOURCES  += src/physics-engine/Dynamics/Solver/rpSequentialImpulseObjectSolver.cpp
#
#SOURCES  += src/physics-engine/Dynamics/rpDynamicsWorld.cpp



   SOURCES  += src/physics-engine/Collision/Body/rpBody.cpp 
   SOURCES  += src/physics-engine/Collision/Body/rpCollisionBody.cpp 
   SOURCES  += src/physics-engine/Collision/BroadPhase/rbBroadPhaseAlgorithm.cpp 
   SOURCES  += src/physics-engine/Collision/BroadPhase/rpDynamicAABBTree.cpp 
   SOURCES  += src/physics-engine/Collision/ContactManiflod/rpContactManifold.cpp 
   SOURCES  += src/physics-engine/Collision/ContactManiflod/rpContactManifoldSet.cpp 
   SOURCES  += src/physics-engine/Collision/ContactManiflod/rpContactPoint.cpp 
   SOURCES  += src/physics-engine/Collision/ContactManiflod/rpGenerationContactManiflodSet.cpp 
   SOURCES  += src/physics-engine/Collision/NarrowPhase/GJK_EPA/rpGjkEpa.cpp 
   SOURCES  += src/physics-engine/Collision/NarrowPhase/rpNarrowPhaseCollisionAlgorithm.cpp 
   SOURCES  += src/physics-engine/Collision/Shapes/rpAABB.cpp 
   SOURCES  += src/physics-engine/Collision/Shapes/rpBoxShape.cpp 
   SOURCES  += src/physics-engine/Collision/Shapes/rpCollisionShape.cpp 
   SOURCES  += src/physics-engine/Collision/Shapes/rpConvexHullShape.cpp 
   SOURCES  += src/physics-engine/Collision/Shapes/rpConvexShape.cpp 
   SOURCES  += src/physics-engine/Collision/Shapes/rpSphereShape.cpp 
   SOURCES  += src/physics-engine/Collision/Shapes/rpTriangleShape.cpp 
   SOURCES  += src/physics-engine/Collision/rpCollisionDetection.cpp 
   SOURCES  += src/physics-engine/Collision/rpCollisionWorld.cpp 
   SOURCES  += src/physics-engine/Collision/rpOverlappingPair.cpp 
   SOURCES  += src/physics-engine/Collision/rpProxyShape.cpp 
   SOURCES  += src/physics-engine/Collision/rpRaycastInfo.cpp 
   SOURCES  += src/physics-engine/Dynamics/Body/rpPhysicsBody.cpp 
   SOURCES  += src/physics-engine/Dynamics/Body/rpPhysicsObject.cpp 
   SOURCES  += src/physics-engine/Dynamics/Body/rpRigidPhysicsBody.cpp 
   SOURCES  += src/physics-engine/Dynamics/Joint/JointAngle/rpAngleJoint.cpp 
   SOURCES  += src/physics-engine/Dynamics/Joint/rpBallAndSocketJoint.cpp 
   SOURCES  += src/physics-engine/Dynamics/Joint/rpDistanceJoint.cpp 
   SOURCES  += src/physics-engine/Dynamics/Joint/rpFixedJoint.cpp 
   SOURCES  += src/physics-engine/Dynamics/Joint/rpHingeJoint.cpp 
   SOURCES  += src/physics-engine/Dynamics/Joint/rpJoint.cpp 
   SOURCES  += src/physics-engine/Dynamics/Joint/rpSliderJoint.cpp 
   SOURCES  += src/physics-engine/Dynamics/Material/rpPhysicsMaterial.cpp 
   SOURCES  += src/physics-engine/Dynamics/Solver/rpContactSolver.cpp 
   SOURCES  += src/physics-engine/Dynamics/Solver/rpSequentialImpulseObjectSolver.cpp 
   SOURCES  += src/physics-engine/Dynamics/rpDynamicsWorld.cpp 
   SOURCES  += src/physics-engine/Dynamics/rpTimer.cpp 
   SOURCES  += src/physics-engine/Geometry/QuickHull/QuickHull.cpp 
   SOURCES  += src/physics-engine/Geometry/rpGrahamScan2dConvexHull.cpp 
   SOURCES  += src/physics-engine/Geometry/rpPolygonClipping.cpp 
   SOURCES  += src/physics-engine/LinearMaths/rpGyroscopic.cpp 
   SOURCES  += src/physics-engine/LinearMaths/rpLorentzContraction.cpp 
   SOURCES  += src/physics-engine/LinearMaths/rpMatrix2x2.cpp 
   SOURCES  += src/physics-engine/LinearMaths/rpMatrix3x3.cpp 
   SOURCES  += src/physics-engine/LinearMaths/rpMatrix4x4.cpp 
   SOURCES  += src/physics-engine/LinearMaths/rpMinkowskiVector4.cpp 
   SOURCES  += src/physics-engine/LinearMaths/rpProjectPlane.cpp 
   SOURCES  += src/physics-engine/LinearMaths/rpQuaternion.cpp 
   SOURCES  += src/physics-engine/LinearMaths/rpTransform.cpp 
   SOURCES  += src/physics-engine/LinearMaths/rpVector2D.cpp 
   SOURCES  += src/physics-engine/LinearMaths/rpVector3D.cpp 
   SOURCES  += src/physics-engine/Memory/rpAlignedallocator.cpp 
   SOURCES  += src/physics-engine/Collision/NarrowPhase/rpNarrowPhaseGjkEpaAlgorithm.cpp 
   SOURCES  += src/physics-engine/Collision/NarrowPhase/GJK/Simplex.cpp 
   SOURCES  += src/physics-engine/Collision/NarrowPhase/MPR/rpMPRAlgorithm.cpp 
   SOURCES  += src/physics-engine/Collision/NarrowPhase/GJK/rpGJKAlgorithm.cpp 
   SOURCES  += src/physics-engine/Collision/NarrowPhase/GJK_EPA/VoronoiSimplex/rpVoronoiSimplexSolver.cpp 
   SOURCES  += src/physics-engine/Collision/NarrowPhase/rpNarrowPhaseMprAlgorithm.cpp

#----------------- opengl-utility -------------------------#
SOURCES  += src/opengl-utility/maths/glmath.cpp
SOURCES  += src/opengl-utility/maths/Vector3.cpp
SOURCES  += src/opengl-utility/maths/Matrix4.cpp

SOURCES  += src/opengl-utility/Mesh/Object3D.cpp
SOURCES  += src/opengl-utility/Mesh/Mesh.cpp
SOURCES  += src/opengl-utility/Mesh/Primitive/MeshBox.cpp
SOURCES  += src/opengl-utility/Mesh/Primitive/MeshPlane.cpp
SOURCES  += src/opengl-utility/Mesh/Primitive/MeshTriangle.cpp
SOURCES  += src/opengl-utility/Mesh/Loaders/MeshReadFile3DS.cpp

SOURCES  += src/opengl-utility/Texture/Texture2D.cpp
SOURCES  += src/opengl-utility/Texture/TextureReaderWriter.cpp

SOURCES  += src/opengl-utility/Camera/CCameraEya.cpp

SOURCES  += src/opengl-utility/Light/Light.cpp

SOURCES  += src/opengl-utility/OpenGL/Shader.cpp
SOURCES  += src/opengl-utility/OpenGL/FrameBufferObject.cpp
SOURCES  += src/opengl-utility/OpenGL/UtilityOpenGLMesh.cpp


#-------------------- interface ---------------------------#
SOURCES  += src/element-engine/GroupMesh.cpp
SOURCES  += src/element-engine/UltimatePhysics.cpp



#-------------------- lua-Integration ---------------------#
#SOURCES  += src/lua-Integration/lua/lapi.c \
#SOURCES  += src/lua-Integration/lua/lauxlib.c \
#SOURCES  += src/lua-Integration/lua/lbaselib.c \
#SOURCES  += src/lua-Integration/lua/lbitlib.c \
#SOURCES  += src/lua-Integration/lua/lcode.c \
#SOURCES  += src/lua-Integration/lua/lcorolib.c \
#SOURCES  += src/lua-Integration/lua/lctype.c \
#SOURCES  += src/lua-Integration/lua/ldblib.c \
#SOURCES  += src/lua-Integration/lua/ldebug.c \
#SOURCES  += src/lua-Integration/lua/ldo.c \
#SOURCES  += src/lua-Integration/lua/ldump.c \
#SOURCES  += src/lua-Integration/lua/lfunc.c \
#SOURCES  += src/lua-Integration/lua/lgc.c \
#SOURCES  += src/lua-Integration/lua/linit.c \
#SOURCES  += src/lua-Integration/lua/liolib.c \
#SOURCES  += src/lua-Integration/lua/llex.c \
#SOURCES  += src/lua-Integration/lua/lmathlib.c \
#SOURCES  += src/lua-Integration/lua/lmem.c \
#SOURCES  += src/lua-Integration/lua/loadlib.c \
#SOURCES  += src/lua-Integration/lua/lobject.c \
#SOURCES  += src/lua-Integration/lua/lopcodes.c \
#SOURCES  += src/lua-Integration/lua/loslib.c \
#SOURCES  += src/lua-Integration/lua/lparser.c \
#SOURCES  += src/lua-Integration/lua/lstate.c \
#SOURCES  += src/lua-Integration/lua/lstring.c \
#SOURCES  += src/lua-Integration/lua/lstrlib.c \
#SOURCES  += src/lua-Integration/lua/ltable.c \
#SOURCES  += src/lua-Integration/lua/ltablib.c \
#SOURCES  += src/lua-Integration/lua/ltm.c \
#SOURCES  += src/lua-Integration/lua/lua.c \
#SOURCES  += src/lua-Integration/lua/luac.c \
#SOURCES  += src/lua-Integration/lua/lundump.c \
#SOURCES  += src/lua-Integration/lua/lutf8lib.c \
#SOURCES  += src/lua-Integration/lua/lvm.c \
#SOURCES  += src/lua-Integration/lua/lzio.c \




OBJS=$(SOURCES:%.cpp=%.o) 
EXECUTABLE=hello

	

$(TARGET):	$(OBJS)
	$(CXX) -o $(TARGET) $(OBJS) $(LIBS)

all:	$(TARGET)

clean:
	rm -f $(OBJS) $(TARGET)
