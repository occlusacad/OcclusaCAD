add_executable(occlusa_tests
    main.cpp
    test_stl.cpp
    test_toothlib.cpp
    test_bridge.cpp
    test_crown.cpp
    test_dicom.cpp
    test_registration.cpp
    test_isosurface.cpp
    test_workflow.cpp
    test_db.cpp
)
target_link_libraries(occlusa_tests PRIVATE occlusa_core occlusa_db occlusa_doctest)
occlusa_set_compile_options(occlusa_tests)
add_test(NAME occlusa_tests COMMAND occlusa_tests)

# End-to-end checks on Linux with EGL: generate the phantom, run OcclusaCAD headless and
#  - align the scan automatically; fail if the error against ground truth exceeds 0.25 mm,
#  - design a crown automatically; fail if the margin deviates more than 0.3 mm from the
#    analytic margin or the crown is not watertight and saved to the case.
option(OCCLUSACAD_HEADLESS_TESTS "Add headless end-to-end tests (Linux, EGL/Mesa required)" OFF)
if(OCCLUSACAD_HEADLESS_TESTS AND OCCLUSACAD_BUILD_TOOLS)
    add_test(NAME designer_alignment_e2e
        COMMAND ${CMAKE_COMMAND}
            -DPHANTOM=$<TARGET_FILE:occlusa_phantom>
            -DDESIGNER=$<TARGET_FILE:OcclusaCAD>
            -DWORK=${CMAKE_CURRENT_BINARY_DIR}/e2e
            -P ${CMAKE_CURRENT_SOURCE_DIR}/DesignerE2E.cmake)
    add_test(NAME designer_crown_e2e
        COMMAND ${CMAKE_COMMAND}
            -DPHANTOM=$<TARGET_FILE:occlusa_phantom>
            -DDESIGNER=$<TARGET_FILE:OcclusaCAD>
            -DTOOTHLIB=$<TARGET_FILE:occlusa_toothlib>
            -DWORK=${CMAKE_CURRENT_BINARY_DIR}/e2e_crown
            -P ${CMAKE_CURRENT_SOURCE_DIR}/DesignerCrownE2E.cmake)
endif()
