/*
ili9486 driver for 16 data lines communication with rp2350
optimased for sending whole frame at once, as rp2350 has enough RAM 

This driver does not implement reads from ILI9486, so it does not
cover every ILI9486 feature.

!!! ONLY ONE CLASS INSTANCE ALLOWED !!!
To drive more ili9486 based displays with one rp2350 reconfigure the same driver instance.
There is not enough pins on rp2350 to run to more then one driver at the same time,
with only one instance allowed some logic (eg. interrupt handling) is easier to manage.
TODO: easy reconfiguration
*/

#ifndef RP2350_ILI9486_H
#define RP2350_ILI9486_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"

#include "rp2350_ili9486_gram_write.pio.h"

// Claim one dma channel, and start PIO state machines
// csx can be hardwired to gnd and can be disabled in driver by passing -1
// resx can be hardwired to +3.3V and can be disabled in driver by passing -1 
void ili9486_init(const int8_t csx, const int8_t dcx, const int8_t resx, const int8_t wrx, const int8_t d0, const PIO pio);

// Set lcd refresh direction, memory write order ans switch between BGR and RGB modes (see ILI9486 0x36 command in datasheet)
// bit 7 - flipRowAddr - change order in which rows are written to ili9486 memory (change from default order)
// bit 6 - flipColAddr - change order in which columns are written to ili9486 memory (change from default order)
// bit 5 - makeHLonger - display's longer side in horizontal. shorter is vertical
// bit 4 - flipLRefresh - change order in which ili9486 refreshes display along longer side (change from default order)
// bit 3 - flipSRefresh - change order in which ili9486 refreshes display along shorter sied (change from default order)
// Refresh order can be used to reduce unpleased effect while changing frame
void ili9486_set_orientation(const uint8_t orientation);

// Prints pixels from buffer to the screen, can be used after ili9486_set_window() call to print only to chosen screen part.
void ili9486_print_pixels(const uint16_t *const buffer, const uint64_t size);

// Fill entire screen with one color
void ili9486_fill_screen(const uint16_t color);

// Print frame from buffer, buffer length must be 153600 (number of pixels) 
void ili9486_print_frame(const uint16_t *buffer);

// Set rectangle into which next print will be loaded (can be smaller then entire screen)
void ili9486_set_window(const uint16_t x0, const uint16_t y0, const uint16_t x1, const uint16_t y1);

// Sets adaptive brightness mode, for more information see 0x55 command in ILI9486 datasheet
// Possible modes:
// 0x00 - off
// 0x01 - user interface image
// 0x02 - still picture (during initialization this mode is chosen)
// 0x03 - moving picture
void ili9486_set_adaptive_brightness(const uint8_t mode);

// Return true if dma/pio is still sending data, any interaction with ili9486 if this method returns true might couze unexpected behavior  
bool ili9486_is_busy();

// Convert RGB888 format to BGR565, and pack it to 16-bit value (blue is on least significant bits)
uint16_t ili9486_to_bgr565(const uint8_t red, const uint8_t green, const uint8_t blue);
	
// Convert RGB888 format to rgb565, and pack it to 16-bit value (red is on least significant bits)
uint16_t ili9486_to_rgb565(const uint8_t red, const uint8_t green, const uint8_t blue);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // RP2350_ILI9486_H