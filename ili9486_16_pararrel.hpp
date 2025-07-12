#pragma once

#include "pico/stdlib.h"
#include "hardware/gpio.h"


class ili9486_16_pararrel {
public:
	static constexpr uint16_t LONG_SIDE = (uint16_t)480;
	static constexpr uint16_t SHORT_SIDE = (uint16_t)320;
	static constexpr uint8_t DATA_PINS_NUM = (uint8_t)16;

	ili9486_16_pararrel(const uint8_t csx, const uint8_t dcx, const uint8_t resx, const uint8_t wrx, const uint8_t d0);

	void init();
	void fillScreen(uint8_t red, uint8_t green, uint8_t blue);
	void setAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

private:
	void sendCommand(uint8_t command);
	void sendData(uint8_t data);
	void wrxPulse();
	void dataOut(uint16_t data);

	const uint8_t csx;
	const uint8_t dcx;
	const uint8_t resx;
	const uint8_t wrx;
	const uint8_t d0; // D0-D15 are consecutive pins
};
