# Invoked by CTest: phantom crown and bridge cases -> OcclusaCAD (headless) -> margin detection,
# insertion axis, automatic design, save to the case -> checks against the analytic margins and the saved files.
file(REMOVE_RECURSE ${WORK})
file(MAKE_DIRECTORY ${WORK})
execute_process(COMMAND ${PHANTOM} --out ${WORK}/phantom --create-case ${WORK}/data --extra-cases 0
                RESULT_VARIABLE rc OUTPUT_VARIABLE phantom_out)
if(NOT rc EQUAL 0)
    message(FATAL_ERROR "occlusa_phantom failed (${rc})")
endif()
string(REGEX MATCH "Created crown demo case [^ ]+ \\(([0-9a-f-]+)\\)" match "${phantom_out}")
if(NOT CMAKE_MATCH_1)
    message(FATAL_ERROR "The phantom did not create the crown case:\n${phantom_out}")
endif()
set(ENV{OCCLUSACAD_CONFIG} ${WORK}/config.json)
execute_process(COMMAND ${DESIGNER} --data-root ${WORK}/data --case ${CMAKE_MATCH_1}
                        --demo-crown ${WORK}/phantom/crown/crown_truth.json --demo-max-margin-error 0.3 --demo-save
                        --frames 20 --screenshot ${WORK}/crown.png
                RESULT_VARIABLE rc OUTPUT_VARIABLE out ERROR_VARIABLE err)
message("${err}")
if(NOT rc EQUAL 0)
    message(FATAL_ERROR "OcclusaCAD end-to-end crown design failed (${rc})")
endif()
if(NOT err MATCHES "DEMO crown 46: .* watertight")
    message(FATAL_ERROR "The crown was not designed")
endif()
file(GLOB crowns ${WORK}/data/cases/*/design/crown_46.stl)
if(NOT crowns)
    message(FATAL_ERROR "The crown STL was not saved to the case")
endif()
if(NOT EXISTS ${WORK}/crown.png)
    message(FATAL_ERROR "No screenshot produced")
endif()

# Three-unit bridge 35-36-37 on the same phantom: both margins, common axis, pontic, connectors
# and the merged bridge saved as one STL.
string(REGEX MATCH "Created bridge demo case [^ ]+ \\(([0-9a-f-]+)\\)" match "${phantom_out}")
if(NOT CMAKE_MATCH_1)
    message(FATAL_ERROR "The phantom did not create the bridge case")
endif()
execute_process(COMMAND ${DESIGNER} --data-root ${WORK}/data --case ${CMAKE_MATCH_1}
                        --demo-crown ${WORK}/phantom/crown/bridge_truth.json --demo-max-margin-error 0.3 --demo-save
                        --frames 20 --screenshot ${WORK}/bridge.png
                RESULT_VARIABLE rc OUTPUT_VARIABLE out ERROR_VARIABLE err)
message("${err}")
if(NOT rc EQUAL 0)
    message(FATAL_ERROR "OcclusaCAD end-to-end bridge design failed (${rc})")
endif()
if(NOT err MATCHES "DEMO bridge 35-36-37: .* watertight")
    message(FATAL_ERROR "The bridge was not designed and merged")
endif()
file(GLOB bridges ${WORK}/data/cases/*/design/bridge_35-36-37.stl)
if(NOT bridges)
    message(FATAL_ERROR "The bridge STL was not saved to the case")
endif()
