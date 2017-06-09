#-------------------------------------------------
#
# Project created by QtCreator 2017-05-07T02:14:31
#
#-------------------------------------------------

QT       += core gui opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = realphysics-interpreter
TEMPLATE = app



#QMAKE_CXXFLAGS += -m32
QMAKE_CXXFLAGS += -std=c++11



#Linux
linux: {

#Android
 android: {
  LIBS +=  -lGLESv1_CM -lGLESv2
}

#Linux default
 !android: {
   LIBS += -lGL -lGLU #-lGLEW
}

}


#Windows
win32: {
   LIBS += -lopengl32 -lglu32 #-lglew32
}

#Windows
win64: {
   LIBS += -lopengl32 -lglu32 #-lglew32
}


QMAKE_RPATHDIR += $ORIGIN/lib



# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += main.cpp\
        widget.cpp \
    glwidget.cpp \
    engine/lua-interpreter/luabind/class.cpp \
    engine/lua-interpreter/luabind/class_info.cpp \
    engine/lua-interpreter/luabind/class_registry.cpp \
    engine/lua-interpreter/luabind/class_rep.cpp \
    engine/lua-interpreter/luabind/create_class.cpp \
    engine/lua-interpreter/luabind/error.cpp \
    engine/lua-interpreter/luabind/exception_handler.cpp \
    engine/lua-interpreter/luabind/function.cpp \
    engine/lua-interpreter/luabind/function_introspection.cpp \
    engine/lua-interpreter/luabind/headertest.cpp \
    engine/lua-interpreter/luabind/inheritance.cpp \
    engine/lua-interpreter/luabind/link_compatibility.cpp \
    engine/lua-interpreter/luabind/object_rep.cpp \
    engine/lua-interpreter/luabind/open.cpp \
    engine/lua-interpreter/luabind/operator.cpp \
    engine/lua-interpreter/luabind/pcall.cpp \
    engine/lua-interpreter/luabind/scope.cpp \
    engine/lua-interpreter/luabind/set_package_preload.cpp \
    engine/lua-interpreter/luabind/stack_content_by_name.cpp \
    engine/lua-interpreter/luabind/weak_ref.cpp \
    engine/lua-interpreter/luabind/wrapper_base.cpp \
    engine/lua-interpreter/loadlibarylua.cpp \
    engine/lua-interpreter/loadlibaryluaopengl.cpp \
    engine/lua-interpreter/loadlibaryluaphysicsengine.cpp \
    engine/lua-interpreter/loadlibaryluashader.cpp \
    engine/lua-interpreter/loadlibaryluauiengine.cpp \
    engine/lua-interpreter/loadlibaryluavalue.cpp \
    engine/lua-interpreter/lua_integration.cpp \
    engine/lua-interpreter/utilopengl.cpp \
    engine/physics-engine/Collision/BroadPhase/rbBroadPhaseAlgorithm.cpp \
    engine/physics-engine/Collision/BroadPhase/rpDynamicAABBTree.cpp \
    engine/physics-engine/Collision/ContactManiflod/rpContactManifold.cpp \
    engine/physics-engine/Collision/ContactManiflod/rpContactManifoldSet.cpp \
    engine/physics-engine/Collision/ContactManiflod/rpContactPoint.cpp \
    engine/physics-engine/Collision/ContactManiflod/rpGenerationContactManiflodSet.cpp \
    engine/physics-engine/Collision/NarrowPhase/GJK_EPA/rpGjkEpa.cpp \
    engine/physics-engine/Collision/NarrowPhase/rpNarrowPhaseCollisionAlgorithm.cpp \
    engine/physics-engine/Collision/Shapes/rpAABB.cpp \
    engine/physics-engine/Collision/Shapes/rpBoxShape.cpp \
    engine/physics-engine/Collision/Shapes/rpCollisionShape.cpp \
    engine/physics-engine/Collision/Shapes/rpConvexHullShape.cpp \
    engine/physics-engine/Collision/Shapes/rpConvexShape.cpp \
    engine/physics-engine/Collision/Shapes/rpSphereShape.cpp \
    engine/physics-engine/Collision/Shapes/rpTriangleShape.cpp \
    engine/physics-engine/Collision/rpCollisionWorld.cpp \
    engine/physics-engine/Collision/rpOverlappingPair.cpp \
    engine/physics-engine/Collision/rpProxyShape.cpp \
    engine/physics-engine/Collision/rpRaycastInfo.cpp \
    engine/physics-engine/Dynamics/Joint/JointAngle/rpAngleJoint.cpp \
    engine/physics-engine/Dynamics/Joint/rpBallAndSocketJoint.cpp \
    engine/physics-engine/Dynamics/Joint/rpDistanceJoint.cpp \
    engine/physics-engine/Dynamics/Joint/rpFixedJoint.cpp \
    engine/physics-engine/Dynamics/Joint/rpHingeJoint.cpp \
    engine/physics-engine/Dynamics/Joint/rpJoint.cpp \
    engine/physics-engine/Dynamics/Joint/rpSliderJoint.cpp \
    engine/physics-engine/Dynamics/Solver/rpContactSolver.cpp \
    engine/physics-engine/Dynamics/Solver/rpSequentialImpulseObjectSolver.cpp \
    engine/physics-engine/Dynamics/rpDynamicsWorld.cpp \
    engine/physics-engine/Dynamics/rpTimer.cpp \
    engine/physics-engine/Geometry/QuickHull/QuickHull.cpp \
    engine/physics-engine/Geometry/rpGrahamScan2dConvexHull.cpp \
    engine/physics-engine/Geometry/rpPolygonClipping.cpp \
    engine/physics-engine/LinearMaths/rpGyroscopic.cpp \
    engine/physics-engine/LinearMaths/rpLorentzContraction.cpp \
    engine/physics-engine/LinearMaths/rpMatrix2x2.cpp \
    engine/physics-engine/LinearMaths/rpMatrix3x3.cpp \
    engine/physics-engine/LinearMaths/rpMatrix4x4.cpp \
    engine/physics-engine/LinearMaths/rpMinkowskiVector4.cpp \
    engine/physics-engine/LinearMaths/rpProjectPlane.cpp \
    engine/physics-engine/LinearMaths/rpQuaternion.cpp \
    engine/physics-engine/LinearMaths/rpTransform.cpp \
    engine/physics-engine/LinearMaths/rpVector2D.cpp \
    engine/physics-engine/LinearMaths/rpVector3D.cpp \
    engine/physics-engine/Memory/rpAlignedallocator.cpp \
    engine/UI-engine/Light/Light.cpp \
    engine/UI-engine/maths/glmath.cpp \
    engine/UI-engine/maths/Matrix4.cpp \
    engine/UI-engine/maths/Vector3.cpp \
    engine/UI-engine/Physics/Body/GroupMesh.cpp \
    engine/UI-engine/Physics/Body/UltimatePhysicsBody.cpp \
    engine/UI-engine/Physics/Joint/UltimateJoint.cpp \
    engine/UI-engine/Physics/DynamicsWorld.cpp \
    engine/UI-engine/Shader/Shader.cpp \
    engine/UI-engine/Texture/Texture2D.cpp \
    engine/lua-interpreter/lua/lapi.c \
    engine/lua-interpreter/lua/lauxlib.c \
    engine/lua-interpreter/lua/lbaselib.c \
    engine/lua-interpreter/lua/lbitlib.c \
    engine/lua-interpreter/lua/lcode.c \
    engine/lua-interpreter/lua/lcorolib.c \
    engine/lua-interpreter/lua/lctype.c \
    engine/lua-interpreter/lua/ldblib.c \
    engine/lua-interpreter/lua/ldebug.c \
    engine/lua-interpreter/lua/ldo.c \
    engine/lua-interpreter/lua/ldump.c \
    engine/lua-interpreter/lua/lfunc.c \
    engine/lua-interpreter/lua/lgc.c \
    engine/lua-interpreter/lua/linit.c \
    engine/lua-interpreter/lua/liolib.c \
    engine/lua-interpreter/lua/llex.c \
    engine/lua-interpreter/lua/lmathlib.c \
    engine/lua-interpreter/lua/lmem.c \
    engine/lua-interpreter/lua/loadlib.c \
    engine/lua-interpreter/lua/lobject.c \
    engine/lua-interpreter/lua/lopcodes.c \
    engine/lua-interpreter/lua/loslib.c \
    engine/lua-interpreter/lua/lparser.c \
    engine/lua-interpreter/lua/lstate.c \
    engine/lua-interpreter/lua/lstring.c \
    engine/lua-interpreter/lua/lstrlib.c \
    engine/lua-interpreter/lua/ltable.c \
    engine/lua-interpreter/lua/ltablib.c \
    engine/lua-interpreter/lua/ltm.c \
    engine/lua-interpreter/lua/lua.c \
    engine/lua-interpreter/lua/luac.c \
    engine/lua-interpreter/lua/lundump.c \
    engine/lua-interpreter/lua/lutf8lib.c \
    engine/lua-interpreter/lua/lvm.c \
    engine/lua-interpreter/lua/lzio.c \
    engine/lua-interpreter/lua/net.c \
    examples/UnitSceneDemo.cpp \
    examples/UnitSceneGeometry.cpp \
    examples/UnitSceneLuaInterpretationSDK.cpp \
    formrunscript.cpp \
    engine/physics-engine/Collision/NarrowPhase/rpNarrowPhaseGjkEpaAlgorithm.cpp \
    engine/physics-engine/Collision/NarrowPhase/GJK/Simplex.cpp \
    engine/physics-engine/Collision/NarrowPhase/MPR/rpMPRAlgorithm.cpp \
    engine/physics-engine/Collision/NarrowPhase/GJK/rpGJKAlgorithm.cpp \
    engine/physics-engine/Collision/NarrowPhase/GJK_EPA/VoronoiSimplex/rpVoronoiSimplexSolver.cpp \
    engine/physics-engine/Collision/NarrowPhase/rpNarrowPhaseMprAlgorithm.cpp \
    engine/UI-engine/Camera/camera.cpp \
    engine/UI-engine/Camera/CCameraEya.cpp \
    engine/UI-engine/Mesh/Loaders/MeshReadFile3DS.cpp \
    engine/UI-engine/Mesh/Primitive/MeshBox.cpp \
    engine/UI-engine/Mesh/Primitive/MeshPlane.cpp \
    engine/UI-engine/Mesh/Primitive/MeshTriangle.cpp \
    engine/UI-engine/Mesh/Mesh.cpp \
    engine/UI-engine/Object/Object3D.cpp \
    engine/UI-engine/Open_GL_/UtilityOpenGL.cpp \
    engine/UI-engine/Open_GL_/GLUtilityGeometry.cpp \
    engine/physics-engine/Body/Material/rpPhysicsMaterial.cpp \
    engine/physics-engine/Body/rpBody.cpp \
    engine/physics-engine/Body/rpCollisionBody.cpp \
    engine/physics-engine/Body/rpPhysicsBody.cpp \
    engine/physics-engine/Body/rpPhysicsObject.cpp \
    engine/physics-engine/Body/rpRigidPhysicsBody.cpp \
    engine/physics-engine/Collision/rpContactManager.cpp

HEADERS  += widget.h \
    glwidget.h \
    engine/lua-interpreter/lua/lapi.h \
    engine/lua-interpreter/lua/lauxlib.h \
    engine/lua-interpreter/lua/lcode.h \
    engine/lua-interpreter/lua/lctype.h \
    engine/lua-interpreter/lua/ldebug.h \
    engine/lua-interpreter/lua/ldo.h \
    engine/lua-interpreter/lua/lfunc.h \
    engine/lua-interpreter/lua/lgc.h \
    engine/lua-interpreter/lua/llex.h \
    engine/lua-interpreter/lua/llimits.h \
    engine/lua-interpreter/lua/lmem.h \
    engine/lua-interpreter/lua/lobject.h \
    engine/lua-interpreter/lua/lopcodes.h \
    engine/lua-interpreter/lua/lparser.h \
    engine/lua-interpreter/lua/lprefix.h \
    engine/lua-interpreter/lua/lstate.h \
    engine/lua-interpreter/lua/lstring.h \
    engine/lua-interpreter/lua/ltable.h \
    engine/lua-interpreter/lua/ltm.h \
    engine/lua-interpreter/lua/lua.h \
    engine/lua-interpreter/lua/lua.hpp \
    engine/lua-interpreter/lua/luaconf.h \
    engine/lua-interpreter/lua/lualib.h \
    engine/lua-interpreter/lua/lundump.h \
    engine/lua-interpreter/lua/lvm.h \
    engine/lua-interpreter/lua/lzio.h \
    engine/lua-interpreter/luabind/detail/conversion_policies/conversion_base.hpp \
    engine/lua-interpreter/luabind/detail/conversion_policies/conversion_policies.hpp \
    engine/lua-interpreter/luabind/detail/conversion_policies/enum_converter.hpp \
    engine/lua-interpreter/luabind/detail/conversion_policies/function_converter.hpp \
    engine/lua-interpreter/luabind/detail/conversion_policies/lua_proxy_converter.hpp \
    engine/lua-interpreter/luabind/detail/conversion_policies/native_converter.hpp \
    engine/lua-interpreter/luabind/detail/conversion_policies/pointer_converter.hpp \
    engine/lua-interpreter/luabind/detail/conversion_policies/reference_converter.hpp \
    engine/lua-interpreter/luabind/detail/conversion_policies/value_converter.hpp \
    engine/lua-interpreter/luabind/detail/call.hpp \
    engine/lua-interpreter/luabind/detail/call_function.hpp \
    engine/lua-interpreter/luabind/detail/call_member.hpp \
    engine/lua-interpreter/luabind/detail/call_shared.hpp \
    engine/lua-interpreter/luabind/detail/call_traits.hpp \
    engine/lua-interpreter/luabind/detail/class_registry.hpp \
    engine/lua-interpreter/luabind/detail/class_rep.hpp \
    engine/lua-interpreter/luabind/detail/constructor.hpp \
    engine/lua-interpreter/luabind/detail/conversion_policies.hpp \
    engine/lua-interpreter/luabind/detail/conversion_storage.hpp \
    engine/lua-interpreter/luabind/detail/convert_to_lua.hpp \
    engine/lua-interpreter/luabind/detail/crtp_iterator.hpp \
    engine/lua-interpreter/luabind/detail/debug.hpp \
    engine/lua-interpreter/luabind/detail/decorate_type.hpp \
    engine/lua-interpreter/luabind/detail/deduce_signature.hpp \
    engine/lua-interpreter/luabind/detail/enum_maker.hpp \
    engine/lua-interpreter/luabind/detail/format_signature.hpp \
    engine/lua-interpreter/luabind/detail/garbage_collector.hpp \
    engine/lua-interpreter/luabind/detail/has_get_pointer.hpp \
    engine/lua-interpreter/luabind/detail/inheritance.hpp \
    engine/lua-interpreter/luabind/detail/instance_holder.hpp \
    engine/lua-interpreter/luabind/detail/link_compatibility.hpp \
    engine/lua-interpreter/luabind/detail/make_instance.hpp \
    engine/lua-interpreter/luabind/detail/meta.hpp \
    engine/lua-interpreter/luabind/detail/most_derived.hpp \
    engine/lua-interpreter/luabind/detail/object.hpp \
    engine/lua-interpreter/luabind/detail/object_rep.hpp \
    engine/lua-interpreter/luabind/detail/open.hpp \
    engine/lua-interpreter/luabind/detail/operator_id.hpp \
    engine/lua-interpreter/luabind/detail/other.hpp \
    engine/lua-interpreter/luabind/detail/pcall.hpp \
    engine/lua-interpreter/luabind/detail/pointee_sizeof.hpp \
    engine/lua-interpreter/luabind/detail/pointee_typeid.hpp \
    engine/lua-interpreter/luabind/detail/policy.hpp \
    engine/lua-interpreter/luabind/detail/primitives.hpp \
    engine/lua-interpreter/luabind/detail/property.hpp \
    engine/lua-interpreter/luabind/detail/ref.hpp \
    engine/lua-interpreter/luabind/detail/signature_match.hpp \
    engine/lua-interpreter/luabind/detail/stack_utils.hpp \
    engine/lua-interpreter/luabind/detail/typetraits.hpp \
    engine/lua-interpreter/luabind/detail/yes_no.hpp \
    engine/lua-interpreter/luabind/adopt_policy.hpp \
    engine/lua-interpreter/luabind/back_reference.hpp \
    engine/lua-interpreter/luabind/back_reference_fwd.hpp \
    engine/lua-interpreter/luabind/class.hpp \
    engine/lua-interpreter/luabind/class_info.hpp \
    engine/lua-interpreter/luabind/config.hpp \
    engine/lua-interpreter/luabind/container_policy.hpp \
    engine/lua-interpreter/luabind/copy_policy.hpp \
    engine/lua-interpreter/luabind/dependency_policy.hpp \
    engine/lua-interpreter/luabind/discard_result_policy.hpp \
    engine/lua-interpreter/luabind/error.hpp \
    engine/lua-interpreter/luabind/error_callback_fun.hpp \
    engine/lua-interpreter/luabind/exception_handler.hpp \
    engine/lua-interpreter/luabind/from_stack.hpp \
    engine/lua-interpreter/luabind/function.hpp \
    engine/lua-interpreter/luabind/function_introspection.hpp \
    engine/lua-interpreter/luabind/get_main_thread.hpp \
    engine/lua-interpreter/luabind/handle.hpp \
    engine/lua-interpreter/luabind/iterator_policy.hpp \
    engine/lua-interpreter/luabind/lua_argument_proxy.hpp \
    engine/lua-interpreter/luabind/lua_include.hpp \
    engine/lua-interpreter/luabind/lua_index_proxy.hpp \
    engine/lua-interpreter/luabind/lua_iterator_proxy.hpp \
    engine/lua-interpreter/luabind/lua_proxy.hpp \
    engine/lua-interpreter/luabind/lua_proxy_interface.hpp \
    engine/lua-interpreter/luabind/lua_state_fwd.hpp \
    engine/lua-interpreter/luabind/luabind.hpp \
    engine/lua-interpreter/luabind/make_function.hpp \
    engine/lua-interpreter/luabind/nil.hpp \
    engine/lua-interpreter/luabind/no_dependency.hpp \
    engine/lua-interpreter/luabind/object.hpp \
    engine/lua-interpreter/luabind/open.hpp \
    engine/lua-interpreter/luabind/operator.hpp \
    engine/lua-interpreter/luabind/out_value_policy.hpp \
    engine/lua-interpreter/luabind/pointer_traits.hpp \
    engine/lua-interpreter/luabind/prefix.hpp \
    engine/lua-interpreter/luabind/raw_policy.hpp \
    engine/lua-interpreter/luabind/return_reference_to_policy.hpp \
    engine/lua-interpreter/luabind/scope.hpp \
    engine/lua-interpreter/luabind/set_package_preload.hpp \
    engine/lua-interpreter/luabind/shared_ptr_converter.hpp \
    engine/lua-interpreter/luabind/tag_function.hpp \
    engine/lua-interpreter/luabind/typeid.hpp \
    engine/lua-interpreter/luabind/version.hpp \
    engine/lua-interpreter/luabind/weak_ref.hpp \
    engine/lua-interpreter/luabind/wrapper_base.hpp \
    engine/lua-interpreter/luabind/yield_policy.hpp \
    engine/lua-interpreter/loadlibarylua.h \
    engine/lua-interpreter/loadlibaryluaopengl.h \
    engine/lua-interpreter/loadlibaryluaphysicsengine.h \
    engine/lua-interpreter/loadlibaryluashader.h \
    engine/lua-interpreter/loadlibaryluauiengine.h \
    engine/lua-interpreter/loadlibaryluavalue.h \
    engine/lua-interpreter/lua_integration.h \
    engine/lua-interpreter/lualibary.h \
    engine/lua-interpreter/utilopengl.h \
    engine/physics-engine/Collision/BroadPhase/rbBroadPhaseAlgorithm.h \
    engine/physics-engine/Collision/BroadPhase/rpDynamicAABBTree.h \
    engine/physics-engine/Collision/ContactManiflod/maniflod.h \
    engine/physics-engine/Collision/ContactManiflod/rpContactManifold.h \
    engine/physics-engine/Collision/ContactManiflod/rpContactManifoldSet.h \
    engine/physics-engine/Collision/ContactManiflod/rpContactPoint.h \
    engine/physics-engine/Collision/ContactManiflod/rpGenerationContactManiflodSet.h \
    engine/physics-engine/Collision/NarrowPhase/GJK_EPA/rpComputeGjkEpaPenetration.h \
    engine/physics-engine/Collision/NarrowPhase/GJK_EPA/rpGjkCollisionDescription.h \
    engine/physics-engine/Collision/NarrowPhase/GJK_EPA/rpGjkEpa.h \
    engine/physics-engine/Collision/NarrowPhase/rpNarrowPhaseCollisionAlgorithm.h \
    engine/physics-engine/Collision/Shapes/rpAABB.h \
    engine/physics-engine/Collision/Shapes/rpBoxShape.h \
    engine/physics-engine/Collision/Shapes/rpCollisionShape.h \
    engine/physics-engine/Collision/Shapes/rpConvexHullShape.h \
    engine/physics-engine/Collision/Shapes/rpConvexShape.h \
    engine/physics-engine/Collision/Shapes/rpSphereShape.h \
    engine/physics-engine/Collision/Shapes/rpTriangleShape.h \
    engine/physics-engine/Collision/collision.h \
    engine/physics-engine/Collision/rpCollisionShapeInfo.h \
    engine/physics-engine/Collision/rpCollisionWorld.h \
    engine/physics-engine/Collision/rpOverlappingPair.h \
    engine/physics-engine/Collision/rpProxyShape.h \
    engine/physics-engine/Collision/rpRaycastInfo.h \
    engine/physics-engine/Dynamics/Joint/JointAngle/rpAngleJoint.h \
    engine/physics-engine/Dynamics/Joint/rpBallAndSocketJoint.h \
    engine/physics-engine/Dynamics/Joint/rpDistanceJoint.h \
    engine/physics-engine/Dynamics/Joint/rpFixedJoint.h \
    engine/physics-engine/Dynamics/Joint/rpHingeJoint.h \
    engine/physics-engine/Dynamics/Joint/rpJoint.h \
    engine/physics-engine/Dynamics/Joint/rpSliderJoint.h \
    engine/physics-engine/Dynamics/Solver/rpContactSolver.h \
    engine/physics-engine/Dynamics/Solver/rpSequentialImpulseObjectSolver.h \
    engine/physics-engine/Dynamics/dynamics.h \
    engine/physics-engine/Dynamics/rpDynamicsWorld.h \
    engine/physics-engine/Dynamics/rpTimer.h \
    engine/physics-engine/Geometry/QuickHull/Structs/Mesh.hpp \
    engine/physics-engine/Geometry/QuickHull/Structs/Plane.hpp \
    engine/physics-engine/Geometry/QuickHull/Structs/Pool.hpp \
    engine/physics-engine/Geometry/QuickHull/Structs/Ray.hpp \
    engine/physics-engine/Geometry/QuickHull/Structs/Vector3.hpp \
    engine/physics-engine/Geometry/QuickHull/Structs/VertexDataSource.hpp \
    engine/physics-engine/Geometry/QuickHull/ConvexHull.hpp \
    engine/physics-engine/Geometry/QuickHull/HalfEdgeMesh.hpp \
    engine/physics-engine/Geometry/QuickHull/MathUtils.hpp \
    engine/physics-engine/Geometry/QuickHull/QuickHull.hpp \
    engine/physics-engine/Geometry/QuickHull/Types.hpp \
    engine/physics-engine/Geometry/geometry.h \
    engine/physics-engine/Geometry/rpGrahamScan2dConvexHull.h \
    engine/physics-engine/Geometry/rpPolygonClipping.h \
    engine/physics-engine/LinearMaths/mathematics.h \
    engine/physics-engine/LinearMaths/rpGyroscopic.h \
    engine/physics-engine/LinearMaths/rpLinearMtah.h \
    engine/physics-engine/LinearMaths/rpLorentzContraction.h \
    engine/physics-engine/LinearMaths/rpMatrix2x2.h \
    engine/physics-engine/LinearMaths/rpMatrix3x3.h \
    engine/physics-engine/LinearMaths/rpMatrix4x4.h \
    engine/physics-engine/LinearMaths/rpMinkowskiVector4.h \
    engine/physics-engine/LinearMaths/rpProjectPlane.h \
    engine/physics-engine/LinearMaths/rpQuaternion.h \
    engine/physics-engine/LinearMaths/rpRay.h \
    engine/physics-engine/LinearMaths/rpRelativityFunction.h \
    engine/physics-engine/LinearMaths/rpTransform.h \
    engine/physics-engine/LinearMaths/rpTransformUtil.h \
    engine/physics-engine/LinearMaths/rpVector2D.h \
    engine/physics-engine/LinearMaths/rpVector3D.h \
    engine/physics-engine/Memory/memory.h \
    engine/physics-engine/Memory/rpAlignedallocator.h \
    engine/physics-engine/Memory/rpAlignedobjectarray.h \
    engine/physics-engine/Memory/rpList.h \
    engine/physics-engine/Memory/rpStack.h \
    engine/physics-engine/config.h \
    engine/physics-engine/physics.h \
    engine/physics-engine/realphysics.h \
    engine/physics-engine/scalar.h \
    engine/UI-engine/Light/Light.h \
    engine/UI-engine/maths/Color.h \
    engine/UI-engine/maths/definitions.h \
    engine/UI-engine/maths/glmath.h \
    engine/UI-engine/maths/Matrix3.h \
    engine/UI-engine/maths/Matrix4.h \
    engine/UI-engine/maths/Vector2.h \
    engine/UI-engine/maths/Vector3.h \
    engine/UI-engine/maths/Vector4.h \
    engine/UI-engine/Physics/Body/Convert.h \
    engine/UI-engine/Physics/Body/GroupMesh.h \
    engine/UI-engine/Physics/Body/UltimatePhysicsBody.h \
    engine/UI-engine/Physics/Joint/UltimateJoint.h \
    engine/UI-engine/Physics/DynamicsWorld.h \
    engine/UI-engine/Physics/physics.h \
    engine/UI-engine/Shader/Shader.h \
    engine/UI-engine/Texture/Texture2D.h \
    engine/UI-engine/engine.h \
    examples/UnitScene.h \
    examples/UnitSceneDemo.h \
    examples/UnitSceneGeometry.h \
    examples/UnitSceneLuaInterpretationSDK.h \
    formrunscript.h \
    engine/physics-engine/Collision/NarrowPhase/rpNarrowPhaseGjkEpaAlgorithm.h \
    engine/physics-engine/Collision/NarrowPhase/GJK/Simplex.h \
    engine/physics-engine/Collision/NarrowPhase/MPR/rpMPRAlgorithm.h \
    engine/physics-engine/Collision/NarrowPhase/GJK/rpGJKAlgorithm.h \
    engine/physics-engine/Collision/NarrowPhase/GJK_EPA/VoronoiSimplex/rpVoronoiSimplexSolver.h \
    engine/physics-engine/Collision/NarrowPhase/GJK_EPA/VoronoiSimplex/rpSimplexSolverInterface.h \
    engine/physics-engine/Collision/NarrowPhase/rpNarrowPhaseMprAlgorithm.h \
    engine/UI-engine/Camera/camera.h \
    engine/UI-engine/Camera/CCameraEya.h \
    engine/UI-engine/Mesh/Loaders/MeshReadFile3DS.h \
    engine/UI-engine/Mesh/Primitive/MeshBox.h \
    engine/UI-engine/Mesh/Primitive/MeshPlane.h \
    engine/UI-engine/Mesh/Primitive/MeshTriangle.h \
    engine/UI-engine/Mesh/Mesh.h \
    engine/UI-engine/Object/Object3D.h \
    engine/UI-engine/Open_GL_/UtilityOpenGL.h \
    engine/UI-engine/Open_GL_/GLUtilityGeometry.h \
    engine/engine.h \
    engine/physics-engine/Body/Material/rpPhysicsMaterial.h \
    engine/physics-engine/Body/rpBody.h \
    engine/physics-engine/Body/rpCollisionBody.h \
    engine/physics-engine/Body/rpPhysicsBody.h \
    engine/physics-engine/Body/rpPhysicsObject.h \
    engine/physics-engine/Body/rpRigidPhysicsBody.h \
    engine/physics-engine/Collision/rpContactManager.h

FORMS    += widget.ui \
    formrunscript.ui

RESOURCES += \
    files.qrc \
    shaders.qrc \
    scripts.qrc


#CONFIG += mobility
#MOBILITY =

