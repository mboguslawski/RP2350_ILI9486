#include "ili9486_16_pararrel.hpp"

ili9486_16_pararrel& ili9486_16_pararrel::getInstance() {
    static ili9486_16_pararrel driver;
    return driver;
}

void ili9486_16_pararrel::init(const uint8_t csx, const uint8_t dcx, const uint8_t resx, const uint8_t wrx, const uint8_t d0, const PIO pio, const ColorMode mode) {
    // Assign values
    this->csx = csx;
    this->dcx = dcx;
    this->resx = resx;
    this->wrx = wrx;
    this->d0 = d0;
    this->pio = pio;
    dmaCompletedTime = 0;
    dmaBusy = false;
    
    // DMA setup
    this->dmaChannel = dma_claim_unused_channel(true);
    irq_set_exclusive_handler(DMA_IRQ_0, dmaTransferCompleteISR);
    irq_set_enabled(DMA_IRQ_0, true);
    dma_channel_set_irq0_enabled(dmaChannel, true);

    // PIO setup
    int offsetProg1 = pio_add_program(pio, &data_push_program);
    int offsetProg2 = pio_add_program(pio, &wrx_management_program);

    data_push_program_init(pio, SM0, offsetProg1, d0);
    wrx_management_program_init(pio, SM1, offsetProg2, wrx);

    pio_sm_set_enabled(pio, SM0, true);
    pio_sm_set_enabled(pio, SM1, true);

    // GPIO setup (D0-D15 and wrx are handled via PIO)
    gpio_init(csx);
    gpio_set_dir(csx, GPIO_OUT);
    gpio_init(dcx);
    gpio_set_dir(dcx, GPIO_OUT);
    gpio_init(resx);
    gpio_set_dir(resx, GPIO_OUT);

    setupILI9486(mode);
}

void ili9486_16_pararrel::setOrientation(const bool flipRowAddr, const bool flipColAddr, const bool makeHLonger, const bool flipLRefresh, const bool flipSRefresh, const bool BGR) {
    uint8_t parameters = 0; 
    parameters |= 
        (flipRowAddr << 7)
        | (flipColAddr << 6)
        | (makeHLonger << 5)
        | (flipLRefresh << 4)
        | (BGR << 3)
        | (flipSRefresh << 2);

    // Memory Access Control
    sendCommand(0x36);
    sendData(parameters);
}

void ili9486_16_pararrel::write16blocking(uint16_t data, bool pioWait) {
    pio_sm_put_blocking(pio, 0, (uint32_t)data);
    

    while( pio_sm_get_tx_fifo_level(pio, SM0) != 0 && pio_interrupt_get(pio, 0) ) {
        sleep_us(1);
    }
}

extern "C" void ili9486_16_pararrel::dmaTransferCompleteISR() {
    ili9486_16_pararrel &instance =  ili9486_16_pararrel::getInstance();
    instance.dmaCompletedTime = time_us_64();
    instance.dmaBusy = false;
    dma_irqn_acknowledge_channel(0, instance.dmaChannel);
}

void ili9486_16_pararrel::writeBufferDMA(uint16_t *buffer, uint64_t bufferSize, uint64_t repeatBits) {
    dmaBusy = true;
    
    // Configure dma channel
    dma_channel_config c = dma_channel_get_default_config(dmaChannel);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(pio, SM0, true));
    channel_config_set_ring(&c, false, repeatBits);

    // Start transfer
    dma_channel_configure(
        dmaChannel,
        &c,
        &pio->txf[0],        // Destination: PIO TX FIFO
        buffer,              // Source
        bufferSize,          // Total pixels to write
        true                 // Start transfer
    );
}

void ili9486_16_pararrel::sendCommand(uint8_t cmd) {
    gpio_put(dcx, 0);
    write16blocking((uint16_t)cmd);
}

void ili9486_16_pararrel::sendData(uint8_t data) {
    gpio_put(dcx, 1);
    write16blocking((uint16_t)data);
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

void ili9486_16_pararrel::fillScreen(uint16_t *color) {
    setAddressWindow(0, 0, ili9486_16_pararrel::SHORT_SIDE - 1, ili9486_16_pararrel::LONG_SIDE -1);
    initGRAMWrite();

    writeBufferDMA(color, (uint64_t)LONG_SIDE * (uint64_t)SHORT_SIDE, 1);
}

void ili9486_16_pararrel::printFrame(uint16_t *buffer) {
    setAddressWindow(0, 0, ili9486_16_pararrel::SHORT_SIDE - 1, ili9486_16_pararrel::LONG_SIDE -1);
    initGRAMWrite();

    writeBufferDMA(buffer, LONG_SIDE * SHORT_SIDE);
}

void ili9486_16_pararrel::setAdaptiveBrightnessMode(const uint8_t mode) {
    if (mode > 0x03) { return; } // Only 0x00-0x03 are valid parameters

    sendCommand(0x55);
    sendData(mode);
}

void ili9486_16_pararrel::setupILI9486(const ColorMode mode) {
    gpio_put(csx, 0);

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
    sendData(0xB0); // Modifying this can resolve flicering strips, see ILI9486 datasheet before edit
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

    // Adaptive brightness
    sendCommand(0x55);
    sendData(0x02); // 0x02 - still image mode

    // Not sure what it does, found this init sequence and seems to work well
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
