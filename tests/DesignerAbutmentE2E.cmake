# Invoked by CTest: phantom scan body case -> OcclusaCAD (headless) -> implant library connection,
# scan body matching from a click, default abutment united with the library interface, save to the
# case -> checks against the true implant position and the saved files. Then the same with the
# generic library exported to a library folder in the data folder (OcclusaCAD format).
file(REMOVE_RECURSE ${WORK})
file(MAKE_DIRECTORY ${WORK})
execute_process(COMMAND ${PHANTOM} --out ${WORK}/phantom --create-case ${WORK}/data --extra-cases 0
                RESULT_VARIABLE rc OUTPUT_VARIABLE phantom_out)
if(NOT rc EQUAL 0)
    message(FATAL_ERROR "occlusa_phantom failed (${rc})")
endif()
string(REGEX MATCH "Created abutment demo case [^ ]+ \\(([0-9a-f-]+)\\)" match "${phantom_out}")
if(NOT CMAKE_MATCH_1)
    message(FATAL_ERROR "The phantom did not create the abutment case:\n${phantom_out}")
endif()
set(CASE ${CMAKE_MATCH_1})
set(ENV{OCCLUSACAD_CONFIG} ${WORK}/config.json)
execute_process(COMMAND ${DESIGNER} --data-root ${WORK}/data --case ${CASE}
                        --demo-abutment ${WORK}/phantom/abutment/abutment_truth.json --demo-max-implant-error 0.05 --demo-save
                        --frames 20 --screenshot ${WORK}/abutment.png
                RESULT_VARIABLE rc OUTPUT_VARIABLE out ERROR_VARIABLE err)
message("${err}")
if(NOT rc EQUAL 0)
    message(FATAL_ERROR "OcclusaCAD end-to-end abutment design failed (${rc})")
endif()
if(NOT err MATCHES "DEMO abutment 36: .* united with the interface")
    message(FATAL_ERROR "The abutment was not designed")
endif()
file(GLOB abutments ${WORK}/data/cases/*/design/abutment_36.stl)
if(NOT abutments)
    message(FATAL_ERROR "The abutment STL was not saved to the case")
endif()

# The generic library as a lab library folder (STL files + library.json), used from disk.
execute_process(COMMAND ${IMPLANTLIB} export --library occlusacad-generic --out ${WORK}/data/implant-libraries/lab-generic
                        --id lab-generic --name "Lab Generic"
                RESULT_VARIABLE rc)
if(NOT rc EQUAL 0)
    message(FATAL_ERROR "occlusa_implantlib export failed (${rc})")
endif()
file(READ ${WORK}/phantom/abutment/abutment_truth.json truth)
string(REPLACE "\"occlusacad-generic\"" "\"lab-generic\"" truth "${truth}")
file(WRITE ${WORK}/lab_truth.json "${truth}")
execute_process(COMMAND ${DESIGNER} --data-root ${WORK}/data --case ${CASE}
                        --demo-abutment ${WORK}/lab_truth.json --demo-max-implant-error 0.05
                        --frames 10 --screenshot ${WORK}/lab.png
                RESULT_VARIABLE rc OUTPUT_VARIABLE out ERROR_VARIABLE err)
message("${err}")
if(NOT rc EQUAL 0 OR NOT err MATCHES "DEMO abutment 36: .* united with the interface")
    message(FATAL_ERROR "The abutment was not designed from the library folder (${rc})")
endif()
