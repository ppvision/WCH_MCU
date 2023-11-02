
# cmake_path(GET CMAKE_CURRENT_SOURCE_DIR FILENAME curr_dir)   
#  ${curr_dir}

SET(APP_NAME riscv_gpio)
add_executable(${APP_NAME})

file( GLOB SRC_LIST *.c )
set(src_list main.c system_ch32v20x ch32v20x_it.c)
# message("Dir: ${curr_dir}(${CMAKE_CURRENT_SOURCE_DIR})  Source list: ${SRC_LIST}")

add_map_output(${APP_NAME})
add_extra_outputs(${APP_NAME})


target_sources(${APP_NAME} PRIVATE
    ${SRC_LIST}
)

target_include_directories(${APP_NAME} PRIVATE 
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(${APP_NAME} "TOOLCHAIN_gcc_${TARGET_PROCESSOR}" WCH_Lib)
target_linker_file(${APP_NAME} ${CMAKE_SOURCE_DIR}/cmake/ld/riscv/ch32v2xx.ld)


# target_wrap_functions(${app_name} printf)
# target_wrap_functions(${app_name} vsprintf)
# target_link_options(${app_name} PRIVATE "-Wl,--wrap=vsprintf2")
# add_c_compile_options()



