add_library(ili9486_16_pararrel STATIC
    ili9486_16_pararrel.cc
)

target_link_libraries(ili9486_16_pararrel
        pico_stdlib
)

target_include_directories(ili9486_16_pararrel PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})