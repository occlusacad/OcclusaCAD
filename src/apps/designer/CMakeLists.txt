add_executable(OcclusaCAD
    main.cpp
    CrownSteps.cpp
    Document.cpp
    StepScanAlignment.cpp
    StepsBasic.cpp
    DesignerApp.cpp
    Views.cpp
)
target_link_libraries(OcclusaCAD PRIVATE occlusa_core occlusa_db occlusa_gfx occlusa_ui occlusa_json)
occlusa_set_compile_options(OcclusaCAD)
occlusa_gui_executable(OcclusaCAD)
