# Invoked by CTest: phantom -> OcclusaCAD (headless) -> automatic alignment -> accuracy check.
file(REMOVE_RECURSE ${WORK})
file(MAKE_DIRECTORY ${WORK})
execute_process(COMMAND ${PHANTOM} --out ${WORK}/phantom --create-case ${WORK}/data --extra-cases 0
                RESULT_VARIABLE rc)
if(NOT rc EQUAL 0)
    message(FATAL_ERROR "occlusa_phantom failed (${rc})")
endif()
set(ENV{OCCLUSACAD_CONFIG} ${WORK}/config.json)
execute_process(COMMAND ${DESIGNER} --dicom ${WORK}/phantom/dicom --scan ${WORK}/phantom/lower_scan.stl
                        --demo-align ${WORK}/phantom/ground_truth.json --demo-max-error 0.25
                        --frames 20 --screenshot ${WORK}/designer.png
                RESULT_VARIABLE rc OUTPUT_VARIABLE out ERROR_VARIABLE err)
message("${err}")
if(NOT rc EQUAL 0)
    message(FATAL_ERROR "OcclusaCAD end-to-end alignment failed (${rc})")
endif()
if(NOT EXISTS ${WORK}/designer.png)
    message(FATAL_ERROR "No screenshot produced")
endif()
