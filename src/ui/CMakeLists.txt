# occlusa_ui: application shell (GLFW + Dear ImGui), themes, fonts, dialogs, widgets.
add_library(occlusa_ui STATIC
    App.cpp
    HeadlessContext.cpp
    FileDialog.cpp
    Fonts.cpp
    Task.cpp
    Theme.cpp
    ToothChart.cpp
    Widgets.cpp
    StbImpl.cpp
)
target_include_directories(occlusa_ui PUBLIC ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(occlusa_ui PUBLIC occlusa_core occlusa_gfx occlusa_imgui occlusa_imguizmo glfw PRIVATE nfd occlusa_stb)
occlusa_embed_resources(occlusa_ui occlusa_resources
    ${PROJECT_SOURCE_DIR}/third_party/fonts/Inter-Regular.ttf
    ${PROJECT_SOURCE_DIR}/third_party/fonts/Inter-SemiBold.ttf)
occlusa_set_compile_options(occlusa_ui)
if(WIN32)
    target_link_libraries(occlusa_ui PRIVATE dwmapi)
endif()

# Headless (off-screen) screenshots for automated UI checks: Linux + EGL only.
if(UNIX AND NOT APPLE)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(OCCLUSA_EGL QUIET IMPORTED_TARGET egl)
        if(OCCLUSA_EGL_FOUND)
            set_source_files_properties(HeadlessContext.cpp PROPERTIES COMPILE_DEFINITIONS OCCLUSACAD_HAVE_EGL)
            target_link_libraries(occlusa_ui PRIVATE PkgConfig::OCCLUSA_EGL)
        endif()
    endif()
endif()
