# Compiler settings applied to first-party targets via occlusa_set_compile_options().

find_package(Threads REQUIRED)

function(occlusa_set_compile_options target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /permissive- /utf-8 /Zc:__cplusplus /bigobj)
        target_compile_definitions(${target} PRIVATE _CRT_SECURE_NO_WARNINGS NOMINMAX WIN32_LEAN_AND_MEAN UNICODE _UNICODE)
        if(OCCLUSACAD_WARNINGS_AS_ERRORS)
            target_compile_options(${target} PRIVATE /WX)
        endif()
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Wno-unused-parameter)
        if(OCCLUSACAD_WARNINGS_AS_ERRORS)
            target_compile_options(${target} PRIVATE -Werror)
        endif()
    endif()
    if(WIN32 AND NOT MSVC)
        target_compile_definitions(${target} PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN UNICODE _UNICODE)
    endif()
endfunction()

# Configure a GUI executable: no console window on Windows, plain binary elsewhere.
function(occlusa_gui_executable target)
    if(WIN32)
        set_target_properties(${target} PROPERTIES WIN32_EXECUTABLE ON)
        if(MSVC)
            # Keep a standard main() entry point for the WIN32 subsystem.
            target_link_options(${target} PRIVATE /ENTRY:mainCRTStartup)
        endif()
    endif()
    # TODO(packaging): build macOS .app bundles (MACOSX_BUNDLE) with Info.plist and icons.
endfunction()
