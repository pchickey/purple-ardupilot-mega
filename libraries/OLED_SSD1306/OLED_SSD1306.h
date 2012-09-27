
#ifndef __OLED_SSD1306_H__
#define __OLED_SSD1306_H__

#include <AP_HAL.h>

class OLED_SSD1306 {
public:
  OLED_SSD1306(AP_HAL::DigitalSource *cs,
               AP_HAL::DigitalSource *dc,
               AP_HAL::DigitalSource *reset,
               AP_HAL::SPIDeviceDriver *spi);
  void test();

private:
  void _display_init();
  void _invert();
  void _cmd(uint8_t);
  void _write_buf();

  AP_HAL::DigitalSource *_cs;
  AP_HAL::DigitalSource *_dc;
  AP_HAL::DigitalSource *_reset;
  AP_HAL::SPIDeviceDriver *_spi;

  uint8_t *_buf;
};

#endif // __OLED_SSD1306_H__

