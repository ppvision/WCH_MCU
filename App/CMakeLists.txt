

get_sub_dirs(sub_dirs ${CMAKE_CURRENT_LIST_DIR})

foreach(subdir ${sub_dirs})
    message("subdir :  ${subdir}/CMakeLists.txt")

    if (EXISTS "${CMAKE_CURRENT_LIST_DIR}/${subdir}/CMakeLists.txt")
        add_subdirectory(${subdir})
    endif()
    
    # file( GLOB SRC_LIST "${subdir}/*.c" )
    # list(LENGTH SRC_LIST  file_num)

    # if( file_num GREATER 0)

    #     string(REPLACE "/" ";" APP_DIR "${subdir}")
        


    #     list(GET APP_DIR -1 APP_NAME_)

    #     message("Dir: ${subdir} (${APP_NAME_})  Source list: ${file_num}")

    #     set(EXE_NAME "${APP_NAME_}")
    #     add_executable   ("${EXE_NAME}" ${SRC_LIST})
    #     add_map_output   ("${EXE_NAME}" )
    #     add_extra_outputs("${EXE_NAME}" )
    #     target_include_directories(${EXE_NAME} PRIVATE 
    #         "${CMAKE_CURRENT_LIST_DIR}/${subdir}"
    #     )
    #     target_link_libraries(${EXE_NAME} "TOOLCHAIN_gcc_${TARGET_PROCESSOR}" WCH_Lib)
    # endif()
endforeach()

# 