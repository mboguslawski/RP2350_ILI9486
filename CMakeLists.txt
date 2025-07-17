add_library(ili9486_16_pararrel STATIC
    ili9486_16_pararrel.cc
)

target_link_libraries(ili9486_16_pararrel
        pico_stdlib
        hardware_pio
        hardware_dma
)

pico_generate_pio_header(ili9486_16_pararrel ${CMAKE_CURRENT_LIST_DIR}/ili9486_gram_write.pio)

target_include_directories(ili9486_16_pararrel PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})