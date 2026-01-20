#include "rp2350_ili9486.h"

// [us] Return busy state after DMA transfer completion (sattle ili9486 internal states)
static const uint64_t BUSY_DURATION_AFTER_DMA_COMPLETION = (uint64_t)100;

static const uint16_t SHORT_SIDE = (uint16_t)320; // px
static const uint16_t LONG_SIDE = (uint16_t)480; // px
static const uint64_t PIXEL_NUM = (uint64_t)153600; // 480px x 320px 

static const uint8_t SM_DATA_LINES = 0;
static const uint8_t SM_WRX = 1;

static struct {
    volatile uint64_t dma_completed_time;
    int dma_channel;
    int8_t csx;
    int8_t dcx;
    int8_t resx;
    int8_t wrx;
    int8_t d0;
    int8_t horizontal_mode;
    volatile int8_t dma_busy;
    PIO pio;

} ili9486_ctx;

static void ili9486_dma_transfer_complete_ISR();
static void ili9486_setup();
static void ili9486_send_cmd(const uint8_t);
static void ili9486_send_data(const uint8_t);

void ili9486_init(const int8_t csx, const int8_t dcx, const int8_t resx, const int8_t wrx, const int8_t d0, const PIO pio) {
    ili9486_ctx.csx = csx;
    ili9486_ctx.dcx = dcx;
    ili9486_ctx.resx = resx;
    ili9486_ctx.wrx = wrx;
    ili9486_ctx.d0 = d0;
    ili9486_ctx.pio = pio;
    ili9486_ctx.horizontal_mode = false;
    ili9486_ctx.dma_completed_time = 0;
    ili9486_ctx.dma_busy = false;
    
    // DMA setup
    ili9486_ctx.dma_channel = dma_claim_unused_channel(true);
    irq_set_exclusive_handler(DMA_IRQ_0, ili9486_dma_transfer_complete_ISR);
    irq_set_enabled(DMA_IRQ_0, true);
    dma_channel_set_irq0_enabled(ili9486_ctx.dma_channel, true);

    // PIO setup
    int offsetProg1 = pio_add_program(ili9486_ctx.pio, &data_push_16_program);
    int offsetProg2 = pio_add_program(ili9486_ctx.pio, &wrx_management_program);

    data_push_16_program_init(ili9486_ctx.pio, SM_DATA_LINES, offsetProg1, ili9486_ctx.d0);
    wrx_management_program_init(ili9486_ctx.pio, SM_WRX, offsetProg2, ili9486_ctx.wrx);

    pio_sm_set_enabled(ili9486_ctx.pio, SM_DATA_LINES, true);
    pio_sm_set_enabled(ili9486_ctx.pio, SM_WRX, true);

    // GPIO setup (D0-D15 and wrx are handled via PIO)
    if (csx != -1) {
        gpio_init(ili9486_ctx.csx);
        gpio_set_dir(ili9486_ctx.csx, GPIO_OUT);
    }

    gpio_init(ili9486_ctx.dcx);
    gpio_set_dir(ili9486_ctx.dcx, GPIO_OUT);
    
    if (ili9486_ctx.resx != -1) {
        gpio_init(ili9486_ctx.resx);
        gpio_set_dir(ili9486_ctx.resx, GPIO_OUT);
    }

    ili9486_setup();
}

uint16_t ili9486_to_rgb565(const uint8_t red, const uint8_t green, const uint8_t blue) {
 	return ((blue & 0xF8) << 8) | ((green & 0xFC) << 3) | ((red & 0xF8) >> 3);
}

uint16_t ili9486_to_bgr565(const uint8_t red, const uint8_t green, const uint8_t blue) {
 	return ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | ((blue & 0xF8) >> 3);
}

void ili9486_set_orientation(const uint8_t orientation) {
    // Memory Access Control
    ili9486_send_cmd(0x36);
    ili9486_send_data(orientation);

    ili9486_ctx.horizontal_mode = orientation & (1 << 5);
}

static void ili9486_init_GRAM_write() {
    ili9486_send_cmd(0x2C);
    gpio_put(ili9486_ctx.dcx, 1);
}

bool ili9486_is_busy() {
        return (ili9486_ctx.dma_busy) ||
            (pio_sm_get_tx_fifo_level(ili9486_ctx.pio, SM_DATA_LINES) != 0) ||
        ((time_us_64() - ili9486_ctx.dma_completed_time) <= BUSY_DURATION_AFTER_DMA_COMPLETION);

}

static void ili9486_write_16_blocking(const uint16_t data) {
    pio_sm_put_blocking(ili9486_ctx.pio, 0, (uint32_t)data);
    

    while( pio_sm_get_tx_fifo_level(ili9486_ctx.pio, SM_DATA_LINES) != 0 && pio_interrupt_get(ili9486_ctx.pio, 0) ) {
        sleep_us(1);
    }
}

static void ili9486_dma_transfer_complete_ISR() {
    ili9486_ctx.dma_completed_time = time_us_64();
    ili9486_ctx.dma_busy = false;
    dma_irqn_acknowledge_channel(0, ili9486_ctx.dma_channel);
}

static void ili9486_write_buffer_DMA(const uint16_t *const buffer, const uint64_t bufferSize, const uint64_t repeatBits) {
    ili9486_ctx.dma_busy = true;
    
    // Configure dma channel
    dma_channel_config c = dma_channel_get_default_config(ili9486_ctx.dma_channel);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(ili9486_ctx.pio, SM_DATA_LINES, true));
    channel_config_set_ring(&c, false, repeatBits);

    // Start transfer
    dma_channel_configure(
        ili9486_ctx.dma_channel,
        &c,
        &ili9486_ctx.pio->txf[SM_DATA_LINES],        // Destination: PIO TX FIFO
        buffer,              // Source
        bufferSize,          // Total pixels to write
        true                 // Start transfer
    );
}

static void ili9486_send_cmd(const uint8_t cmd) {
    gpio_put(ili9486_ctx.dcx, 0);
    ili9486_write_16_blocking((uint16_t)cmd);
}

static void ili9486_send_data(const uint8_t data) {
    gpio_put(ili9486_ctx.dcx, 1);
    ili9486_write_16_blocking((uint16_t)data);
}

void ili9486_set_window(const uint16_t x0, const uint16_t y0, const uint16_t x1, const uint16_t y1) {
    ili9486_send_cmd(0x2A);
    ili9486_send_data( (ili9486_ctx.horizontal_mode ? y0 : x0) >> 8 ); 
    ili9486_send_data( (ili9486_ctx.horizontal_mode ? y0 : x0) & 0xFF) ;
    ili9486_send_data( (ili9486_ctx.horizontal_mode ? y1 : x1) >> 8); 
    ili9486_send_data( (ili9486_ctx.horizontal_mode ? y1 : x1) & 0xFF);

    ili9486_send_cmd(0x2B);
    ili9486_send_data( (ili9486_ctx.horizontal_mode ? x0 : y0) >> 8 );
    ili9486_send_data( (ili9486_ctx.horizontal_mode ? x0 : y0) & 0xFF );
    ili9486_send_data( (ili9486_ctx.horizontal_mode ? x1 : y1) >> 8 );
    ili9486_send_data( (ili9486_ctx.horizontal_mode ? x1 : y1) & 0xFF );
}

void ili9486_print_pixels(const uint16_t const *buffer, const uint64_t size) {
    ili9486_init_GRAM_write();
    ili9486_write_buffer_DMA(buffer, size, 0);
}

void ili9486_fill_screen(const uint16_t color) {
    ili9486_set_window(0, 0, SHORT_SIDE - 1, LONG_SIDE -1);
    ili9486_init_GRAM_write();

    ili9486_write_buffer_DMA(&color, PIXEL_NUM, 1);
}

void ili9486_print_frame(const uint16_t const *buffer) {
    ili9486_set_window(0, 0, SHORT_SIDE - 1, LONG_SIDE -1);
    ili9486_init_GRAM_write();

    ili9486_write_buffer_DMA(buffer, PIXEL_NUM, 0);
}

void ili9486_set_adaptive_brightness(const uint8_t mode) {
    if (mode > 0x03) { return; } // Only 0x00-0x03 are valid parameters

    ili9486_send_cmd(0x55);
    ili9486_send_data(mode);
}

static void ili9486_setup() {
    if (ili9486_ctx.csx != -1) {
        gpio_put(ili9486_ctx.csx, 0);
    }

    // Reset
    if (ili9486_ctx.resx != -1) {
        gpio_put(ili9486_ctx.resx, 1);
        sleep_ms(100);
        gpio_put(ili9486_ctx.resx, 0);
        sleep_ms(100);
        gpio_put(ili9486_ctx.resx, 1);
        sleep_ms(100);
    }

    // Power control 1
    ili9486_send_cmd(0xC0);
    ili9486_send_data(0x19);
    ili9486_send_data(0x1A);

    // Power control 2
    ili9486_send_cmd(0xC1);
    ili9486_send_data(0x45);
    ili9486_send_data(0x00);

    // Power Control 3 (For Normal Mode)
    ili9486_send_cmd(0xC2);
    ili9486_send_data(0x33);

    // VCOM Control
    ili9486_send_cmd(0xC5);
    ili9486_send_data(0x00);
    ili9486_send_data(0x28);

    // Frame Rate Control (In Normal Mode/Full Colors)
    ili9486_send_cmd(0xB1);
    ili9486_send_data(0xB0); // Modifying this can resolve flicering strips, see ILI9486 datasheet before edit
    ili9486_send_data(0x11);

    // Display Inversion Control
    ili9486_send_cmd(0xB4);
    ili9486_send_data(0x02);

    // Display Function Control
    ili9486_send_cmd(0xB6);
    ili9486_send_data(0x00);
    ili9486_send_data(0x42);
    ili9486_send_data(0x3B);

    // Entry Mode Set
    ili9486_send_cmd(0xB7);
    ili9486_send_data(0x07);

    // Positive gamma correction
    ili9486_send_cmd(0xE0);
    ili9486_send_data(0x1F);
    ili9486_send_data(0x25);
    ili9486_send_data(0x22);
    ili9486_send_data(0x0B);
    ili9486_send_data(0x06);
    ili9486_send_data(0x0A);
    ili9486_send_data(0x4E);
    ili9486_send_data(0xC6);
    ili9486_send_data(0x39);
    ili9486_send_data(0x00);
    ili9486_send_data(0x00);
    ili9486_send_data(0x00);
    ili9486_send_data(0x00);
    ili9486_send_data(0x00);
    ili9486_send_data(0x00);

    // Negative gamma correction
    ili9486_send_cmd(0xE1);
    ili9486_send_data(0x1F);
    ili9486_send_data(0x3F);
    ili9486_send_data(0x3F);
    ili9486_send_data(0x0F);
    ili9486_send_data(0x1F);
    ili9486_send_data(0x0F);
    ili9486_send_data(0x46);
    ili9486_send_data(0x49);
    ili9486_send_data(0x31);
    ili9486_send_data(0x05);
    ili9486_send_data(0x09);
    ili9486_send_data(0x03);
    ili9486_send_data(0x1C);
    ili9486_send_data(0x1A);
    ili9486_send_data(0x00);

    // Pixel format. 0x66 RGB666, 0x55 RGB565
    ili9486_send_cmd(0x3A);
    ili9486_send_data(0x55);

    // Adaptive brightness
    ili9486_send_cmd(0x55);
    ili9486_send_data(0x02); // 0x02 - still image mode

    // Not sure what it does, found this init sequence and seems to work well
    ili9486_send_cmd(0xF1);
    ili9486_send_data(0x36);
    ili9486_send_data(0x04);
    ili9486_send_data(0x00);
    ili9486_send_data(0x3C);
    ili9486_send_data(0x0F);
    ili9486_send_data(0x0F);
    ili9486_send_data(0xA4);
    ili9486_send_data(0x02);
    ili9486_send_cmd(0xF2);
    ili9486_send_data(0x18);
    ili9486_send_data(0xA3);
    ili9486_send_data(0x12);
    ili9486_send_data(0x02);
    ili9486_send_data(0x32);
    ili9486_send_data(0x12);
    ili9486_send_data(0xFF);
    ili9486_send_data(0x32);
    ili9486_send_data(0x00);
    ili9486_send_cmd(0xF4);
    ili9486_send_data(0x40);
    ili9486_send_data(0x00);
    ili9486_send_data(0x08);
    ili9486_send_data(0x91);
    ili9486_send_data(0x04);
    ili9486_send_cmd(0xF8);
    ili9486_send_data(0x21);
    ili9486_send_data(0x04);
    ili9486_send_cmd(0xF9);
    ili9486_send_data(0x00);
    ili9486_send_data(0x08);

    // Sleep OUT
    ili9486_send_cmd(0x11);
    sleep_ms(120);

    // Display ON
    ili9486_send_cmd(0x29);
    sleep_ms(20);
}
