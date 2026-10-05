# Vendored third-party dependencies. All are under permissive licenses; see
# third_party/README.md for the inventory.

set(CMAKE_FOLDER third_party)

# Third-party code is compiled without our warning flags.

# --- GLFW (zlib) -------------------------------------------------------------
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
add_subdirectory(glfw EXCLUDE_FROM_ALL)

# --- glad GL loader (generated, public domain / MIT) -------------------------
add_library(occlusa_glad INTERFACE)
target_include_directories(occlusa_glad SYSTEM INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/glad/include)

# --- GLM (MIT) ---------------------------------------------------------------
add_library(occlusa_glm INTERFACE)
target_include_directories(occlusa_glm SYSTEM INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/glm)
target_compile_definitions(occlusa_glm INTERFACE GLM_ENABLE_EXPERIMENTAL GLM_FORCE_SILENT_WARNINGS)

# --- nlohmann/json (MIT) -----------------------------------------------------
add_library(occlusa_json INTERFACE)
target_include_directories(occlusa_json SYSTEM INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/nlohmann)

# --- doctest (MIT) -----------------------------------------------------------
add_library(occlusa_doctest INTERFACE)
target_include_directories(occlusa_doctest SYSTEM INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/doctest)

# --- stb (public domain / MIT) -----------------------------------------------
add_library(occlusa_stb INTERFACE)
target_include_directories(occlusa_stb SYSTEM INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/stb)

# --- SQLite (public domain) --------------------------------------------------
add_library(occlusa_sqlite3 STATIC sqlite3/sqlite3.c)
target_include_directories(occlusa_sqlite3 SYSTEM PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/sqlite3)
target_compile_definitions(occlusa_sqlite3 PRIVATE
    SQLITE_THREADSAFE=1
    SQLITE_DQS=0
    SQLITE_DEFAULT_FOREIGN_KEYS=1
    SQLITE_OMIT_LOAD_EXTENSION
    SQLITE_OMIT_DEPRECATED
    SQLITE_DEFAULT_MEMSTATUS=0)
target_link_libraries(occlusa_sqlite3 PRIVATE Threads::Threads)
if(UNIX AND NOT APPLE)
    target_link_libraries(occlusa_sqlite3 PRIVATE m)
endif()

# --- Dear ImGui, docking branch (MIT) ----------------------------------------
add_library(occlusa_imgui STATIC
    imgui/imgui.cpp
    imgui/imgui_demo.cpp
    imgui/imgui_draw.cpp
    imgui/imgui_tables.cpp
    imgui/imgui_widgets.cpp
    imgui/backends/imgui_impl_glfw.cpp
    imgui/backends/imgui_impl_opengl3.cpp
    imgui/misc/cpp/imgui_stdlib.cpp)
target_include_directories(occlusa_imgui SYSTEM PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui/backends
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui/misc/cpp)
target_compile_definitions(occlusa_imgui PUBLIC IMGUI_DISABLE_OBSOLETE_FUNCTIONS)
target_link_libraries(occlusa_imgui PUBLIC glfw)

# --- ImGuizmo (MIT) ----------------------------------------------------------
add_library(occlusa_imguizmo STATIC imguizmo/ImGuizmo.cpp)
target_include_directories(occlusa_imguizmo SYSTEM PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/imguizmo)
target_link_libraries(occlusa_imguizmo PUBLIC occlusa_imgui)

# --- nativefiledialog-extended (zlib) ----------------------------------------
if(UNIX AND NOT APPLE)
    # Use xdg-desktop-portal over D-Bus rather than pulling in GTK.
    set(NFD_PORTAL ON CACHE BOOL "" FORCE)
endif()
set(NFD_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(NFD_INSTALL OFF CACHE BOOL "" FORCE)
add_subdirectory(nfd EXCLUDE_FROM_ALL)

# Manifold (Apache-2.0): robust mesh Booleans (bridge unions). Core only, serial backend,
# no 2D cross sections, so it needs no further dependencies.
file(GLOB OCCLUSA_MANIFOLD_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/manifold/src/*.cpp)
add_library(occlusa_manifold STATIC ${OCCLUSA_MANIFOLD_SRCS})
target_include_directories(occlusa_manifold SYSTEM PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/manifold/include)
target_compile_definitions(occlusa_manifold PUBLIC MANIFOLD_PAR=-1)
target_compile_features(occlusa_manifold PUBLIC cxx_std_17)
set_target_properties(occlusa_manifold PROPERTIES POSITION_INDEPENDENT_CODE ON)
if(MSVC)
    target_compile_options(occlusa_manifold PRIVATE /bigobj /W0)
else()
    target_compile_options(occlusa_manifold PRIVATE -w)
endif()
