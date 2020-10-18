project(sensor_ade9153a)

add_definitions(
        ${SENSORS_DEFINITIONS}
)

list(APPEND ${PROJECT_NAME}_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/ade9153a.c
        )

add_library(${PROJECT_NAME} OBJECT EXCLUDE_FROM_ALL ${${PROJECT_NAME}_SOURCES})

if (DEFINED BOARD_DEFINITIONS)
    target_compile_definitions(${PROJECT_NAME} PUBLIC ${BOARD_DEFINITIONS})
endif ()

target_include_directories(${PROJECT_NAME} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/
        )

set_property(TARGET ${PROJECT_NAME} PROPERTY C_STANDARD 11)
