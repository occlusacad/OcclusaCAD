# occlusa_gfx: OpenGL 3.3 core rendering (meshes, volume ray casting, MPR slices).
add_library(occlusa_gfx STATIC
    Camera.cpp
    GL.cpp
    GlObjects.cpp
    Renderers.cpp
)
target_include_directories(occlusa_gfx PUBLIC ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(occlusa_gfx PUBLIC occlusa_core occlusa_glad)
occlusa_set_compile_options(occlusa_gfx)
