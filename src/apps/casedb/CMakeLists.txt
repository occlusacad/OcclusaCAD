add_executable(OcclusaCADDB
    main.cpp
    DbApp.cpp
)
target_link_libraries(OcclusaCADDB PRIVATE occlusa_core occlusa_db occlusa_ui)
occlusa_set_compile_options(OcclusaCADDB)
occlusa_gui_executable(OcclusaCADDB)
