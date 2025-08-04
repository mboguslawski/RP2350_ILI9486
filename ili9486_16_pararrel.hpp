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
#include "hardware/irq.h"

#include "ili9486_gram_write.pio.h"

class ili9486_16_pararrel {
public:
	// Screen dimensions
	static constexpr uint16_t LONG_SIDE = (uint16_t)480;
	static constexpr uint16_t SHORT_SIDE = (uint16_t)320;
	static constexpr uint32_t PIXEL_NUM = (uint32_t)153600; // 480*320=153600

	static ili9486_16_pararrel& getInstance(); // Return only possible instance of this class

	// Claim one dma channel, and start PIO state machines
	// csx can be hardwired to gnd and can be disabled in driver by passing -1
	// resx can be hardwired to +3.3V and can be disabled in driver by passing -1 
	void init(const int8_t csx, const int8_t dcx, const int8_t resx, const int8_t wrx, const int8_t d0, const PIO pio);
	
	// Set lcd refresh direction, memory write order ans switch between BGR and RGB modes (see ILI9486 0x36 command in datasheet)
	// flipRowAddr - change order in which rows are written to ili9486 memory (change from default order)
	// flipColAddr - change order in which columns are written to ili9486 memory (change from default order)
	// makeHLonger - display's longer side in horizontal. shorter is vertical
	// flipHRefresh - change order in which ili9486 refreshes display along longer side (change from default order)
	// flipSRefresh - change order in which ili9486 refreshes display along shorter sied (change from default order)
	// Refresh order can be used to reduce unpleased effect while changing frame
	void setOrientation(const bool flipRowAddr, const bool flipColAddr, const bool makeHLonger, const bool flipLRefresh, const bool flipSRefresh, const bool BGR);

	// Fill entire screen with one color
	void fillScreen(uint16_t *color);
	
	// Print frame from buffer, BUFFER LENGTH MUST MATCH NUMBER OF PIXELS (153600) 
	void printFrame(uint16_t *buffer);
	
	// Set rectangle into which next print will be loaded (can be smaller then entire screen)
	void setAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
	
	// Sets adaptive brightness mode, for more information see 0x55 command in ILI9486 datasheet
	// Possible modes:
	// 0x00 - off
	// 0x01 - user interface image
	// 0x02 - still picture (during initialization this mode is chosen)
	// 0x03 - moving picture
	// Passing other parameter then above will have no effect on ILI9486
	void setAdaptiveBrightnessMode(const uint8_t mode);

	// Return true if dma/pio is still sending data, any interaction with ili9486 if this method returns true might couze unexpected behavior  
	inline bool isBusy();
	
	// Convert RGB888 format to BGR565, and pack it to 16-bit value (blue is on least significant bits)
	static inline uint16_t rgb888_to_bgr565(const uint8_t red, const uint8_t green, const uint8_t blue);
	
	// Convert RGB888 format to rgb565, and pack it to 16-bit value (red is on least significant bits)
	static inline uint16_t rgb888_to_rgb565(const uint8_t red, const uint8_t green, const uint8_t blue);

private:
	// Only one class instance allowed
	ili9486_16_pararrel() = default;
	~ili9486_16_pararrel() = default;
    // Prevent duplicates
    ili9486_16_pararrel(const ili9486_16_pararrel&) = delete;
    ili9486_16_pararrel& operator=(const ili9486_16_pararrel&) = delete;
    ili9486_16_pararrel(ili9486_16_pararrel&&) = delete;
    ili9486_16_pararrel& operator=(ili9486_16_pararrel&&) = delete;

	// Send one byte as command to ILI9486
	void sendCommand(uint8_t command);

	// Send one byte as data to ILI9486 (optimal for command parameters)
	void sendData(uint8_t data);
	
	// Send 16-bit data block to ILI9486 (via pio)
	void write16blocking(uint16_t data, bool pioWait = true);

	// Sends buffer to PIO (via DMA)
	void writeBufferDMA(uint16_t *buffer, uint64_t bufferSize, uint64_t repeatBits = 0);

	// Sets dmaCompletedTime
	static void dmaTransferCompleteISR(); 
	
	inline void initGRAMWrite();

	// Initial commands to setup ILI9486
	void setupILI9486();

	// Which state machines to use
	static constexpr uint8_t SM_DATA_LINES = (uint8_t)0;
	static constexpr uint8_t SM_WRX = (uint8_t)1;

	PIO pio;
	int8_t csx;
	int8_t dcx;
	int8_t resx;
	int8_t wrx;
	int8_t d0; // D0-D15 are consecutive pins
	int dmaChannel; // Currently used DMA channel, -1 if no dma channel in use
	volatile uint64_t dmaCompletedTime; // Time in us of last completed dma transfer
	volatile bool dmaBusy; // Will be set to false after setting dmaCompletedTime to avoid races
	static constexpr uint64_t BUSY_DURATION_AFTER_COMPLETION = (uint64_t)100; // [us] Return busy state after DMA transfer completion (sattle ili9486 internal states)
};

uint16_t ili9486_16_pararrel::rgb888_to_rgb565(const uint8_t red, const uint8_t green, const uint8_t blue) {
 	return ((blue & 0xF8) << 8) | ((green & 0xFC) << 3) | ((red & 0xF8) >> 3);
}

uint16_t ili9486_16_pararrel::rgb888_to_bgr565(const uint8_t red, const uint8_t green, const uint8_t blue) {
 	return ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | ((blue & 0xF8) >> 3);
}

void ili9486_16_pararrel::initGRAMWrite() {
	sendCommand(0x2C);
	gpio_put(dcx, 1);
}

bool ili9486_16_pararrel::isBusy() {
    return dmaBusy ||
        (pio_sm_get_tx_fifo_level(pio, SM_DATA_LINES) != 0) ||
        ((time_us_64() - dmaCompletedTime) <= BUSY_DURATION_AFTER_COMPLETION);
}
