
cmake_path(GET CMAKE_CURRENT_SOURCE_DIR FILENAME curr_dir)   

SET(APP_NAME ${curr_dir})
file( GLOB SRC_LIST *.c )

message("Dir: ${curr_dir}(${CMAKE_CURRENT_SOURCE_DIR})  Source list: ${SRC_LIST}")

add_executable(${APP_NAME} ${SRC_LIST})
add_map_output(${APP_NAME})
add_extra_outputs(${APP_NAME})

target_include_directories(${APP_NAME} PRIVATE 
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(${APP_NAME} "TOOLCHAIN_gcc_${TARGET_PROCESSOR}" WCH_Lib)
target_linker_file(${APP_NAME} ${CMAKE_SOURCE_DIR}/cmake/ld/chf2xx.ld)


# target_wrap_functions(${APP_NAME} printf)
# target_wrap_functions(${APP_NAME} vsprintf)
# target_link_options(${APP_NAME} PRIVATE "-Wl,--wrap=vsprintf2")
# add_c_compile_options()



