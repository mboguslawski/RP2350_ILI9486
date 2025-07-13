/*
ili9486 driver for 16 data lines communication with rp2350
optimazed for sending whole frame at once, as rp2350 has enough RAM 
*/

#pragma once

#include "pico/stdlib.h"
#include "hardware/gpio.h"


class ili9486_16_pararrel {
public:
	// DO NOT CHANGE
	static constexpr uint16_t LONG_SIDE = (uint16_t)480;
	static constexpr uint16_t SHORT_SIDE = (uint16_t)320;
	static constexpr uint8_t DATA_PINS_NUM = (uint8_t)16;
	
	enum ColorMode {RGB656};

	// Assign pin to functionalities, data line pins must be consecutive, d0 is pin with lowest number
	ili9486_16_pararrel(const uint8_t csx, const uint8_t dcx, const uint8_t resx, const uint8_t wrx, const uint8_t d0);

	void init(const ColorMode mode);
	void fillScreen(uint8_t red, uint8_t green, uint8_t blue);
	void printFrame(uint8_t *buffer);
	void setAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
private:
	void sendCommand(uint8_t command); // Send command byte
	void sendData(uint8_t data); // Send data byte
	void dataOut(uint16_t data); // Slower then setting mask on gpio, to double ensure proper transfer (eg. during initialization)

	inline uint16_t rgb888_to_bgr565(const uint8_t red, const uint8_t green, const uint8_t blue);

	const uint8_t csx;
	const uint8_t dcx;
	const uint8_t resx;
	const uint8_t wrx;
	const uint8_t d0; // D0-D15 are consecutive pins
};

uint16_t ili9486_16_pararrel::rgb888_to_bgr565(const uint8_t red, const uint8_t green, const uint8_t blue) {
 	return ((blue & 0xF8) << 8) | ((green & 0xFC) << 3) | ((red & 0xF8) >> 3);
}
