include_directories(${OSG_INCLUDE_DIRS} ${OSGWORKS_DIR})

set(OSGWORKS_SOURCES
    osgwQuery/QueryStats.cpp
    osgwQuery/QueryObject.cpp
    osgwQuery/QueryBenchmarks.cpp
    osgwQuery/QueryBin.cpp
#    osgwQuery/QueryDrawCB.cpp
    osgwQuery/QueryUtils.cpp
    osgwQuery/QueryAPI.cpp
    osgwQuery/QueryComputation.cpp
    osgwControls/SliderControl.cpp
    osgwTools/InsertRemove.cpp
    osgwTools/CountStateSets.cpp
    osgwTools/Uniqifier.cpp
    osgwTools/CallbackSupport.cpp
    osgwTools/CollapseLOD.cpp
    osgwTools/ForceFlattenTransforms.cpp
    osgwTools/CountsVisitor.cpp
    osgwTools/ReducerOp.cpp
    osgwTools/PluginLoader.cpp
    osgwTools/AbsoluteModelTransform.cpp
    osgwTools/RemoveProgram.cpp
    osgwTools/Transform.cpp
    osgwTools/FBOUtils.cpp
    osgwTools/RemoveData.cpp
    osgwTools/NodeUtils.cpp
    osgwTools/FindNamedNode.cpp
    osgwTools/DataLoader.cpp
    osgwTools/TransparencyUtils.cpp
    osgwTools/LODCreationNodeVisitor.cpp
    osgwTools/GeometryModifier.cpp
    osgwTools/ScreenCapture.cpp
    osgwTools/ShortEdgeOp.cpp
    osgwTools/GeometryOperation.cpp
    osgwTools/NodePathUtils.cpp
    osgwTools/TangentSpaceOp.cpp
    osgwTools/StateSetUtils.cpp
    osgwTools/ReadFile.cpp
    osgwTools/ParallelVisitor.cpp
    osgwTools/SimplifierOp.cpp
    osgwTools/Quat.cpp
    osgwTools/Version.cpp
    osgwTools/DecimatorOp.cpp
    osgwTools/RemoveLOD.cpp
    osgwTools/CameraConfigObject.cpp
    osgwTools/RefID.cpp
    osgwTools/Shapes.cpp
    osgwTools/PrimitiveSetConversion.cpp
)

add_library(osgWorks ${OSGWORKS_SOURCES})
target_link_libraries(osgWorks ${OSG_LIBS})
