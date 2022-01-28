if(NOT DRIVER_LPUART_MIMX8QM6_cm4_core0_INCLUDED)
    
    set(DRIVER_LPUART_MIMX8QM6_cm4_core0_INCLUDED true CACHE BOOL "driver_lpuart component is included.")

    target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/fsl_lpuart.c
    )


    target_include_directories(${MCUX_SDK_PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/.
    )


    include(driver_common_MIMX8QM6_cm4_core0)

endif()
