#include "ili9486_16_pararrel.hpp"



ili9486_16_pararrel::ili9486_16_pararrel(const uint8_t csx, const uint8_t dcx, const uint8_t resx, const uint8_t wrx, const uint8_t d0):
    csx(csx),
    dcx(dcx),
    resx(resx),
    wrx(wrx),
    d0(d0)
{}

void ili9486_16_pararrel::init() {
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
    sendData(0x19); // VRH1[4:0] - Sets VREG1OUT voltage for positive gamma
    sendData(0x1A); // VRH2[4:0] - Sets the VREG2OUT voltage for negative gamma

    // Power control 2
    sendCommand(0xC1);
    sendData(0x44); // BT[2:0] ([6] = 1) - Sets the factor used in the step-up circuits
    sendData(0x00); // VC[2:0] - Sets VCI1 regulator output voltage.

    // Power Control 3 (For Normal Mode)
    sendCommand(0xC2);
    /*
    DCA0[2:0] DCA1[4:3]
    
    DCA0 - Selects the operating frequency of the step-up circuit 1/4/5 for Normal mode. The higher step-up operating
    frequency enhances the drivability of the step-up circuit and the quality of display but increases the current consumption.
    Adjust the frequency taking the trade-off between the display quality and the current consumption into account.
    
    DCA1 [2:0]: Selects the operating frequency of the step-up circuit 2/3 for Normal mode. The higher step-up operating
    frequency enhances the drivability of the step-up circuit and the quality of display but increases the current consumption.
    Adjust the frequency taking the trade-off between the display quality and the current consumption into account.
    */    
    sendData(0x33);

    // VCOM Control
    sendCommand(0xC5);
    sendData(0x00); // nVM[0] - When the NV memory is programmed, the nVM will be set as ‘1’ automatically. 0 -> NV memory is not programmed. 1- > NV memory is programmed
    sendData(0x38); // VCM_REG[7:0] - Is used to set factor to generate VCOM voltage from the reference voltage VREG2OUT.

    // Frame Rate Control (In Normal Mode/Full Colors)
    sendCommand(0xB1);
    sendData(0xA0); // FRS[7:4] DIVA[1:0] - FRS sets the frame frequency of full color normal mode. DIVA is division ratio for internal clocks when Normal mode.
    sendData(0x11); // RTNA[4:0] - Is used to set 1H (line) period of Normal mode at CPU interface.


    // Display Inversion Control
    sendCommand(0xB4);
    sendData(0x02); // ZINV[4] DINV[1:0] - ZINV sets Z-inversion mode (0-disable, 1-enable). DINV sets the inversion mode

    // Display Function Control - for more info see datasheet
    sendCommand(0xB6);
    sendData(0x00);
    sendData(0x42);
    sendData(0x3B);

    // Entry Mode Set
    sendCommand(0xB7);
    sendData(0x07);

    // Positive gamma correction
    sendCommand(0xE0);
    sendData(0x1F); // VP0[4:0]
    sendData(0x25); // VP1[5:0]
    sendData(0x22); // VP2[5:0]
    sendData(0x0B); // VP4[3:0]
    sendData(0x06); // VP6[4:0]
    sendData(0x0A); // VP13[3:0]
    sendData(0x4E); // VP20[6:0]
    sendData(0xC6); // VP36[7:4] VP27[3:0]
    sendData(0x39); // VP43[6:0]
    sendData(0x00); // VP50[3:0]
    sendData(0x00); // VP57[4:0]
    sendData(0x00); // VP59[3:0]
    sendData(0x00); // VP61[5:0]
    sendData(0x00); // VP62[5:0]
    sendData(0x00); // VP63[4:0]

    // Negative gamma correction
    sendCommand(0xE1);
    sendData(0x1F); // VN0[4:0]
    sendData(0x3F); // VN1[5:0]
    sendData(0x3F); // VN2[5:0]
    sendData(0x0F); // VN4[3:0]
    sendData(0x1F); // RVN6[4:0]
    sendData(0x0F); // VN13[3:0]
    sendData(0x46); // VN20[6:0]
    sendData(0x49); // VN36[7:4] VN27[3:0]
    sendData(0x31); // VN43[6:0]
    sendData(0x05); // VN50[3:0]
    sendData(0x09); // VN57[4:0]
    sendData(0x03); // VN59[3:0]
    sendData(0x1C); // VN61[5:0]
    sendData(0x1A); // VN62[5:0]
    sendData(0x00); // VN63[4:0]
    
    // Pixel format 
    sendCommand(0x3A);
    sendData(0x55);    // 18-bit/pixel

    // Not sure what it does, found in docs as initialize sequence
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

    sendCommand(0x11); // Sleep OUT
    sleep_ms(120);

    sendCommand(0x29); // Display ON
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
    sendCommand(0x2A); // CASET
    sendData(x0 >> 8); sendData(x0 & 0xFF);
    sendData(x1 >> 8); sendData(x1 & 0xFF);

    sendCommand(0x2B); // PASET
    sendData(y0 >> 8); sendData(y0 & 0xFF);
    sendData(y1 >> 8); sendData(y1 & 0xFF);

    sendCommand(0x2C); // RAMWR
}

#define RGB888_TO_RGB565(r, g, b)  ( ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3) )

void ili9486_16_pararrel::fillScreen(uint8_t red, uint8_t green, uint8_t blue) {
    setAddressWindow(0, 0, ili9486_16_pararrel::SHORT_SIDE - 1, ili9486_16_pararrel::LONG_SIDE -1);

    red = red << 2;
    green = green << 2;
    blue = blue << 2;
    gpio_put(dcx, 1);
    gpio_put(csx, 0);

    uint32_t color = 0;
    color += RGB888_TO_RGB565(red, green, blue);

    for (uint64_t i = 0; i < (uint64_t)ili9486_16_pararrel::LONG_SIDE * (uint64_t)ili9486_16_pararrel::SHORT_SIDE; i++) {
        gpio_put(wrx, 0);
        __asm volatile("nop");
        __asm volatile("nop");
        __asm volatile("nop");
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
