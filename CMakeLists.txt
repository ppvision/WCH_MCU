cmake_minimum_required(VERSION 3.5)

SET(TARGET_PROCESSOR m3)

SET(APP_NAME chf208_soc)



include(cmake/util.cmake)
include(cmake/toolchain/toolchain-arm.cmake)

project(${APP_NAME} C ASM)


option(USB_CONSOLE "build for USB console, otherwise UART" OFF)
option(BUILD_TYPE  "build Debug version" ON)



if(BUILD_TYPE)
	SET(CMAKE_BUILD_TYPE Debug)
endif()



# import some lib
# include(lib/cmake_libs.cmake)

add_subdirectory(CH32F2xxSDK)
add_subdirectory(App)
# add_subdirectory(EXAM)


# add_executable(${APP_NAME} ${SRC_LIST})
# add_map_output(${APP_NAME})
# add_extra_outputs(${APP_NAME})

# target_include_directories(${APP_NAME} PRIVATE 
# 	App
# )


# # target_wrap_functions(${APP_NAME} printf)
# # target_wrap_functions(${APP_NAME} vsprintf)
# # target_link_options(${APP_NAME} PRIVATE "-Wl,--wrap=vsprintf2")
# # add_c_compile_options()


# target_link_libraries(${APP_NAME} "TOOLCHAIN_gcc_${TARGET_PROCESSOR}" WCH_Lib)

# target_linker_file(${APP_NAME} ${CMAKE_CURRENT_LIST_DIR}/cmake/ld/chf2xx.ld)
