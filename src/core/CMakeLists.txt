# occlusa_core: platform-independent domain logic. No GUI or OpenGL dependencies.
add_library(occlusa_core STATIC
    AppConfig.cpp
    CommandLine.cpp
    Dental.cpp
    Geometry.cpp
    IsoSurface.cpp
    KdTree.cpp
    Log.cpp
    Mesh.cpp
    MeshBvh.cpp
    MeshTopology.cpp
    Platform.cpp
    Registration.cpp
    StlIO.cpp
    Time.cpp
    Uuid.cpp
    Volume.cpp
    Workflow.cpp
    crown/Bridge.cpp
    crown/CrownBuilder.cpp
    crown/InsertionAxis.cpp
    crown/Margin.cpp
    crown/ToothLibrary.cpp
    dicom/DicomCodecs.cpp
    dicom/DicomDataset.cpp
    dicom/DicomSeries.cpp
    dicom/DicomWriter.cpp
)
target_include_directories(occlusa_core PUBLIC ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(occlusa_core PUBLIC occlusa_glm Threads::Threads PRIVATE occlusa_json occlusa_manifold)
occlusa_set_compile_options(occlusa_core)
if(WIN32)
    target_link_libraries(occlusa_core PRIVATE shell32 ole32 advapi32)
endif()
