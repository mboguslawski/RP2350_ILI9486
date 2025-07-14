#include "ili9486_16_pararrel.hpp"



ili9486_16_pararrel::ili9486_16_pararrel(const uint8_t csx, const uint8_t dcx, const uint8_t resx, const uint8_t wrx, const uint8_t d0):
    csx(csx),
    dcx(dcx),
    resx(resx),
    wrx(wrx),
    d0(d0)
{}

void ili9486_16_pararrel::init(const ColorMode mode) {
    // GPIO setup
    gpio_init(csx);
    gpio_set_dir(csx, GPIO_OUT);
    gpio_init(dcx);
    gpio_set_dir(dcx, GPIO_OUT);
    gpio_init(resx);
    gpio_set_dir(resx, GPIO_OUT);
    gpio_init(wrx);
    gpio_set_dir(wrx, GPIO_OUT);

    for (uint8_t i = 0; i < DATA_PINS_NUM; i++) {
        gpio_init(d0 + i);
        gpio_set_dir(d0 + i, GPIO_OUT); 
    }

    // Initialization sequence //

    gpio_put(csx, 1);
    gpio_put(dcx, 1);

    // Reset
    gpio_put(resx, 1);
    sleep_ms(100);
    gpio_put(resx, 0);
    sleep_ms(100);
    gpio_put(resx, 1);
    sleep_ms(100);

    // Power control 1
    sendCommand(0xC0);
    sendData(0x19);
    sendData(0x1A);

    // Power control 2
    sendCommand(0xC1);
    sendData(0x45);
    sendData(0x00);

    // Power Control 3 (For Normal Mode)
    sendCommand(0xC2);
    sendData(0x33);

    // VCOM Control
    sendCommand(0xC5);
    sendData(0x00);
    sendData(0x28);

    // Frame Rate Control (In Normal Mode/Full Colors)
    sendCommand(0xB1);
    sendData(0xA0); // Modifying this can resolve flicering strips, see ILI9486 datasheet before edit
    sendData(0x11);

    // Display Inversion Control
    sendCommand(0xB4);
    sendData(0x02);

    // Display Function Control
    sendCommand(0xB6);
    sendData(0x00);
    sendData(0x42);
    sendData(0x3B);

    // Entry Mode Set
    sendCommand(0xB7);
    sendData(0x07);

    // Positive gamma correction
    sendCommand(0xE0);
    sendData(0x1F);
    sendData(0x25);
    sendData(0x22);
    sendData(0x0B);
    sendData(0x06);
    sendData(0x0A);
    sendData(0x4E);
    sendData(0xC6);
    sendData(0x39);
    sendData(0x00);
    sendData(0x00);
    sendData(0x00);
    sendData(0x00);
    sendData(0x00);
    sendData(0x00);

    // Negative gamma correction
    sendCommand(0xE1);
    sendData(0x1F);
    sendData(0x3F);
    sendData(0x3F);
    sendData(0x0F);
    sendData(0x1F);
    sendData(0x0F);
    sendData(0x46);
    sendData(0x49);
    sendData(0x31);
    sendData(0x05);
    sendData(0x09);
    sendData(0x03);
    sendData(0x1C);
    sendData(0x1A);
    sendData(0x00);

    // Pixel format. 0x66 RGB666, 0x55 RGB565
    sendCommand(0x3A);
    const uint8_t format = (mode == ColorMode::RGB656) ? 0x55 : 0x55; // TODO RGB666 format
    sendData(format);

    // Unknown init sequence
    sendCommand(0xF1);
    sendData(0x36);
    sendData(0x04);
    sendData(0x00);
    sendData(0x3C);
    sendData(0x0F);
    sendData(0x0F);
    sendData(0xA4);
    sendData(0x02);

    sendCommand(0xF2);
    sendData(0x18);
    sendData(0xA3);
    sendData(0x12);
    sendData(0x02);
    sendData(0x32);
    sendData(0x12);
    sendData(0xFF);
    sendData(0x32);
    sendData(0x00);

    sendCommand(0xF4);
    sendData(0x40);
    sendData(0x00);
    sendData(0x08);
    sendData(0x91);
    sendData(0x04);

    sendCommand(0xF8);
    sendData(0x21);
    sendData(0x04);

    sendCommand(0xF9);
    sendData(0x00);
    sendData(0x08);

    // Sleep OUT
    sendCommand(0x11);
    sleep_ms(120);

    // Display ON
    sendCommand(0x29);
    sleep_ms(20);
}

void ili9486_16_pararrel::dataOut(uint16_t data) {
    for (uint8_t i = 0; i < DATA_PINS_NUM; i++) {
        gpio_put(d0+i, data & (1<<i));
    }
}

void ili9486_16_pararrel::sendCommand(uint8_t cmd) {
    gpio_put(dcx, 0);
    gpio_put(csx, 0);
    gpio_put(wrx, 0);
    uint16_t c = cmd;
    dataOut(c);
    gpio_put(wrx, 1);
    gpio_put(csx, 1);
}

void ili9486_16_pararrel::sendData(uint8_t data) {
    gpio_put(dcx, 1);
    gpio_put(csx, 0);
    gpio_put(wrx, 0);
    uint16_t d = data;
    dataOut(d);
    gpio_put(wrx, 1);
    gpio_put(csx, 1);
}

void ili9486_16_pararrel::setAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    sendCommand(0x2A);
    sendData(x0 >> 8); 
    sendData(x0 & 0xFF);
    sendData(x1 >> 8); 
    sendData(x1 & 0xFF);

    sendCommand(0x2B);
    sendData(y0 >> 8);
    sendData(y0 & 0xFF);
    sendData(y1 >> 8);
    sendData(y1 & 0xFF);
}

void ili9486_16_pararrel::fillScreen(uint8_t red, uint8_t green, uint8_t blue) {
    setAddressWindow(0, 0, ili9486_16_pararrel::SHORT_SIDE - 1, ili9486_16_pararrel::LONG_SIDE -1);
    initGRAMWrite();

    uint16_t color = rgb888_to_bgr565(red, green, blue);

    for (uint64_t i = 0; i < (uint64_t)ili9486_16_pararrel::LONG_SIDE * (uint64_t)ili9486_16_pararrel::SHORT_SIDE; i++) {
        // Without nops transfer is too fast (ili9486 limitations)
        gpio_put(wrx, 0);
        __asm volatile("nop");
        __asm volatile("nop");
        __asm volatile("nop");
        gpio_clr_mask(0xFFFF);
        gpio_set_mask(color);
        __asm volatile("nop");
        __asm volatile("nop");
        gpio_put(wrx, 1);
        __asm volatile("nop");
        __asm volatile("nop");
    }

    gpio_put(csx, 1);
    sendCommand(0x29); // Seems to discrad tearing effect while changing frame
}

void ili9486_16_pararrel::printFrame(uint8_t *buffer) {
    sendCommand(0x29); // Seems to discrad tearing effect while changing frame
    
    setAddressWindow(0, 0, ili9486_16_pararrel::SHORT_SIDE - 1, ili9486_16_pararrel::LONG_SIDE -1);
    initGRAMWrite();

    uint32_t c = (uint32_t)0;

    for (uint64_t i = 0; i < (uint64_t)ili9486_16_pararrel::LONG_SIDE * (uint64_t)ili9486_16_pararrel::SHORT_SIDE; i++) {
        gpio_put(wrx, 0);
        uint32_t c = (uint32_t)0 + rgb888_to_bgr565(buffer[i*3+0], buffer[i*3+1], buffer[i*3+2]);
        gpio_put_masked(0x0000FFFF, c);
        gpio_put(wrx, 1);
    }

    gpio_put(csx, 1);
    sendCommand(0x29); // Seems to discrad tearing effect while changing frame
}
