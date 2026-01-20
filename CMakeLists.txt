add_library(rp2350_ili9486 STATIC
    rp2350_ili9486.cc
)

target_link_libraries(rp2350_ili9486
        pico_stdlib
        hardware_pio
        hardware_dma
        hardware_irq
)

pico_generate_pio_header(rp2350_ili9486 ${CMAKE_CURRENT_LIST_DIR}/rp2350_ili9486_gram_write.pio)

target_include_directories(rp2350_ili9486 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})