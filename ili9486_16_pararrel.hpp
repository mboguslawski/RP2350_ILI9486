/*
ili9486 driver for 16 data lines communication with rp2350
optimazed for sending whole frame at once, as rp2350 has enough RAM 
*/

#pragma once

#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "ili9486_gram_write.pio.h"


class ili9486_16_pararrel {
public:
	// DO NOT CHANGE
	static constexpr uint16_t LONG_SIDE = (uint16_t)480;
	static constexpr uint16_t SHORT_SIDE = (uint16_t)320;

	enum ColorMode {RGB656};

	// Assign pin to functionalities, data line pins must be consecutive, d0 is pin with lowest number
	ili9486_16_pararrel(const uint8_t csx, const uint8_t dcx, const uint8_t resx, const uint8_t wrx, const uint8_t d0, const PIO pio);

	void init(const ColorMode mode);
	void fillScreen(uint8_t red, uint8_t green, uint8_t blue);
	void printFrame(uint8_t *buffer);
	void setAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

private:
	// Which state machines to use
	static constexpr uint8_t SM0 = (uint8_t)0;
	static constexpr uint8_t SM1 = (uint8_t)1;

	void sendCommand(uint8_t command); // Send command byte
	void sendData(uint8_t data); // Send data byte
	void write16blocking(uint16_t data, bool pioWait = true); // Send 16 bits to pio FIFO

	inline void waitForPio(); // Wait till pio finished data transfer

	inline void initGRAMWrite();
	inline uint16_t rgb888_to_bgr565(const uint8_t red, const uint8_t green, const uint8_t blue);
	
	const PIO pio;
	const uint8_t csx;
	const uint8_t dcx;
	const uint8_t resx;
	const uint8_t wrx;
	const uint8_t d0; // D0-D15 are consecutive pins
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
