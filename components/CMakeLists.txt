add_library(FreeRTOS INTERFACE)
target_include_directories(FreeRTOS INTERFACE 
    ${CMAKE_SOURCE_DIR}/config
    ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS
    ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS/include
    ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS/portable/MemMang;
    ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS/portable/Common;
    ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS/portable/gcc/ARM_CM3

)

target_sources(FreeRTOS INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS/portable/gcc/ARM_CM3/port.c
    ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS/portable/MemMang/heap_4.c
    ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS/timers.c
    ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS/tasks.c
    ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS/queue.c
    ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS/list.c
    ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS/event_groups.c
    ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS/stream_buffer.c
    ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS/croutine.c
)

