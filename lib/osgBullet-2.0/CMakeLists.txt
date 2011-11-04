include_directories(${BULLET_DIR}/src ${OSG_INCLUDE_DIRS} ${OSGWORKS_DIR} ${OSGBULLET_DIR})

set(OSGBULLET_SOURCES
    osgbCollision/ComputeCylinderVisitor.cpp
    osgbCollision/ComputeTriMeshVisitor.cpp
    osgbCollision/Utils.cpp
    osgbCollision/Chart.cpp
    osgbCollision/BoundingCylinder.cpp
    osgbCollision/BoundingCone.cpp
    osgbCollision/CollectVerticesVisitor.cpp
    osgbCollision/Version.cpp
    osgbCollision/CollisionShapes.cpp
    osgbCollision/VertexAggOp.cpp
    osgbCollision/GLDebugDrawer.cpp
    osgbCollision/ComputeShapeVisitor.cpp
    osgbDynamics/PhysicsState.cpp
    osgbDynamics/RigidBody.cpp
    osgbDynamics/PhysicsThread.cpp
    osgbDynamics/RigidBodyAnimation.cpp
    osgbDynamics/TripleBuffer.cpp
    osgbDynamics/GroundPlane.cpp
    osgbDynamics/CreationRecord.cpp
    osgbDynamics/MotionState.cpp
    osgbInteraction/HandTestEventHandler.cpp
    osgbInteraction/ArticulationRecord.cpp
    osgbInteraction/LaunchHandler.cpp
    osgbInteraction/SaveRestoreHandler.cpp
    osgbInteraction/HandNode.cpp
    osgbInteraction/DragHandler.cpp
#    osgbInteraction/p5support.cpp
    osgbInteraction/GestureHandler.cpp
)

add_library(osgBullet ${OSGBULLET_SOURCES})
target_link_libraries(osgBullet osgWorks ${BULLET_LIBS} ${OSG_LIBS})
