cmake_minimum_required(VERSION 3.5)

# TARGET_PROCESSOR >m0,m0plus,m4,m4f,m33,m7,m0plus,riscv32

SET(TARGET_PROCESSOR riscv32)



include(cmake/util.cmake)
include(cmake/toolchain/toolchain.cmake)


project(WCH32 C ASM)


if(TARGET_PROCESSOR STREQUAL "riscv32")
	add_subdirectory(CH32V2xxSDK)
	add_subdirectory(App/GPIO_Toggle_riscv)

else()
	add_subdirectory(CH32F2xxSDK)
	add_subdirectory(App/GPIO_Toggle)

endif()


option(USB_CONSOLE "build for USB console, otherwise UART" OFF)
option(BUILD_TYPE  "build Debug version" ON)


if(BUILD_TYPE)
	SET(CMAKE_BUILD_TYPE ${BUILD_TYPE})
endif()


