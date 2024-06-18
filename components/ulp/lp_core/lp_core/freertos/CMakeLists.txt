# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD

set(FREERTOS_PORT A_CUSTOM_PORT)
# Let FREERTOS_HEAP undefined. Do not use the heap

add_library(freertos_config INTERFACE)
target_include_directories(freertos_config SYSTEM
    INTERFACE
        "${sdkconfig_dir}"
        "${ULP_COMPONENT_INCLUDES}"
        "${IDF_PATH}/components/ulp/lp_core/lp_core/freertos/config")
target_compile_definitions(freertos_config INTERFACE projCOVERAGE_TEST=0)

add_library(freertos_kernel_port OBJECT)
target_sources(freertos_kernel_port
    PRIVATE
        "${IDF_PATH}/components/ulp/lp_core/lp_core/freertos/port/port.c"
        "${IDF_PATH}/components/ulp/lp_core/lp_core/freertos/port/portASM.S"
        "${IDF_PATH}/components/ulp/lp_core/lp_core/freertos/port/portmacro.h")
target_include_directories(freertos_kernel_port
    PRIVATE
        "${sdkconfig_dir}"
        "${ULP_COMPONENT_INCLUDES}"
        "${IDF_PATH}/components/ulp/lp_core/lp_core/include"
        "${IDF_PATH}/components/ulp/lp_core/shared/include"
        "${IDF_PATH}/components/riscv/include")

add_library(freertos_kernel_port_headers INTERFACE)
target_include_directories(freertos_kernel_port_headers
    INTERFACE "${IDF_PATH}/components/ulp/lp_core/lp_core/freertos/port")

target_link_libraries(freertos_kernel_port
    PRIVATE
        freertos_kernel_port_headers
        freertos_kernel_include)

add_library(freertos_kernel STATIC)

add_subdirectory(FreeRTOS-Kernel/include)
add_subdirectory(FreeRTOS-Kernel/portable)

target_sources(freertos_kernel PRIVATE
    FreeRTOS-Kernel/croutine.c
    FreeRTOS-Kernel/event_groups.c
    FreeRTOS-Kernel/list.c
    FreeRTOS-Kernel/queue.c
    FreeRTOS-Kernel/stream_buffer.c
    FreeRTOS-Kernel/tasks.c
    FreeRTOS-Kernel/timers.c
)

if(DEFINED FREERTOS_HEAP)
    # User specified a heap implementation add heap implementation to freertos_kernel.
    target_sources(freertos_kernel PRIVATE
        # If FREERTOS_HEAP is digit between 1 .. 5 - it is heap number,
        # otherwise - it is path to custom heap source file
        $<IF:$<BOOL:$<FILTER:${FREERTOS_HEAP},EXCLUDE,^[1-5]$>>,
            ${FREERTOS_HEAP},FreeRTOS-Kernel/portable/MemMang/heap_${FREERTOS_HEAP}.c>)
endif()


target_link_libraries(freertos_kernel
    PUBLIC
        freertos_kernel_include
        freertos_kernel_port_headers
    PRIVATE
        freertos_kernel_port
)
