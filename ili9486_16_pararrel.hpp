/*
ili9486 driver for 16 data lines communication with rp2350
optimazed for sending whole frame at once, as rp2350 has enough RAM 

This driver does not implement reads from ILI9486, so it does not
cover every ILI9486 feature.

!!! ONLY ONE CLASS INSTANCE ALLOWED !!!
To drive more ili9486 based displays with one rp2350 reconfigure the same driver instance.
There is not enough pins on rp2350 to run to more then one driver at the same time,
with only one instance allowed some logic (eg. interrupt handling) is easier to manage.
TODO: easy reconfiguration
*/

#pragma once

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"

#include "ili9486_gram_write.pio.h"

class ili9486_16_pararrel {
public:
	// Screen dimensions
	static constexpr uint16_t LONG_SIDE = (uint16_t)480;
	static constexpr uint16_t SHORT_SIDE = (uint16_t)320;
	static constexpr uint32_t PIXEL_NUM = (uint32_t)153600; // 480*320=153600

	static ili9486_16_pararrel& getInstance(); // Return only possible instance of this class

	enum ColorMode {RGB656};

	// Claim one dma channel, and start PIO state machines 
	void init(const uint8_t csx, const uint8_t dcx, const uint8_t resx, const uint8_t wrx, const uint8_t d0, const PIO pio, const ColorMode mode);
	// Fill entire screen with one color
	void fillScreen(uint16_t *color);
	// Print frame from buffer, BUFFER LENGTH MUST MATCH NUMBER OF PIXELS (153600) 
	void printFrame(uint16_t *buffer);
	// Set rectangle into which next print will be loaded (can be smaller then entire screen)
	void setAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
	// Convert RGB888 format to BGR565, and pack it to 16-bit value
	static inline uint16_t rgb888_to_bgr565(const uint8_t red, const uint8_t green, const uint8_t blue);
private:
	// Only one class instance allowed
	ili9486_16_pararrel() = default;
	~ili9486_16_pararrel() = default;
    // Prevent duplicates
    ili9486_16_pararrel(const ili9486_16_pararrel&) = delete;
    ili9486_16_pararrel& operator=(const ili9486_16_pararrel&) = delete;
    ili9486_16_pararrel(ili9486_16_pararrel&&) = delete;
    ili9486_16_pararrel& operator=(ili9486_16_pararrel&&) = delete;
	
	// Which state machines to use
	static constexpr uint8_t SM0 = (uint8_t)0;
	static constexpr uint8_t SM1 = (uint8_t)1;

	void sendCommand(uint8_t command); // Handles dcx pin and send data to PIO (via CPU)
	void sendData(uint8_t data); // Handles dcx pin and send data to PIO (via CPU) 
	void write16blocking(uint16_t data, bool pioWait = true); // Send 16 bits to pio FIFO (via CPU)
	void writeBufferDMA(uint16_t *buffer, uint64_t bufferSize, uint64_t repeatBits = 0); // Sends buffer to PIO (via DMA)

	inline void waitForPio(); // Wait till pio finished data transfer
	inline void initGRAMWrite();

	// Initial commands to setup ILI9486
	void setupILI9486(const ColorMode mode);

	PIO pio;
	uint8_t csx;
	uint8_t dcx;
	uint8_t resx;
	uint8_t wrx;
	uint8_t d0; // D0-D15 are consecutive pins
	int dmaChannel; // Currently used DMA channel, -1 if no dma channel in use
};

uint16_t ili9486_16_pararrel::rgb888_to_bgr565(const uint8_t red, const uint8_t green, const uint8_t blue) {
 	return ((blue & 0xF8) << 8) | ((green & 0xFC) << 3) | ((red & 0xF8) >> 3);
}

void ili9486_16_pararrel::initGRAMWrite() {
	sendCommand(0x2C);
	gpio_put(dcx, 1);
}

void ili9486_16_pararrel::waitForPio() {
	while(pio_interrupt_get(pio, 0) || !pio_sm_is_tx_fifo_empty(pio, 0)) {
        sleep_us(1);
    }
}
