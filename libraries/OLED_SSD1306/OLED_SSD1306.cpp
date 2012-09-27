
#include <stdlib.h>

#include "OLED_SSD1306.h"
#include "ssd1306_defs.h"

extern const AP_HAL::HAL& hal;

#define SCREENBUFSIZE (SSD1306_LCDWIDTH*SSD1306_LCDHEIGHT/8)

OLED_SSD1306::OLED_SSD1306(
    AP_HAL::DigitalSource *cs,
    AP_HAL::DigitalSource *dc,
    AP_HAL::DigitalSource *reset,
    AP_HAL::SPIDeviceDriver *spi) :
      _cs(cs),
      _dc(dc),
      _reset(reset),
      _spi(spi) 
{
  _buf = (uint8_t *) malloc(SCREENBUFSIZE);
  if (_buf != 0) {
    memset(_buf, 0, SCREENBUFSIZE);
  }
}

void OLED_SSD1306::test() {
  /* Set chip select high */
  _cs->mode(GPIO_OUTPUT);
  _cs->write(1);

  /* Set D/C line low */
  _dc->mode(GPIO_OUTPUT);
  _dc->write(1);
 
  /* Reset the chip: Hold low for 10ms, then bring high */
  _reset->mode(GPIO_OUTPUT);
  _reset->write(0);
  hal.scheduler->delay(10);
  _reset->write(1);

  _display_init();

  hal.scheduler->delay(10);

  _write_buf();

  for (int i = 0; i < SCREENBUFSIZE; i++) {
    _buf[i] = (uint8_t) (i % 256);
  }
  _write_buf();

  hal.scheduler->delay(100);
  _invert();
  hal.scheduler->delay(1000);
  _invert();
}

void OLED_SSD1306::_display_init() {
  // Init sequence for 128x32 OLED module
  _cmd(SSD1306_DISPLAYOFF);                    // 0xAE
  _cmd(SSD1306_SETDISPLAYCLOCKDIV);            // 0xD5
  _cmd(0x80);                                  // the suggested ratio 0x80
  _cmd(SSD1306_SETMULTIPLEX);                  // 0xA8
  _cmd(0x1F);
  _cmd(SSD1306_SETDISPLAYOFFSET);              // 0xD3
  _cmd(0x0);                                   // no offset
  _cmd(SSD1306_SETSTARTLINE | 0x0);            // line #0
  _cmd(SSD1306_CHARGEPUMP);                    // 0x8D
  _cmd(0x14);
  _cmd(SSD1306_MEMORYMODE);                    // 0x20
  _cmd(0x00);                                  // 0x0 act like ks0108
  _cmd(SSD1306_SEGREMAP | 0x1);
  _cmd(SSD1306_COMSCANDEC);
  _cmd(SSD1306_SETCOMPINS);                    // 0xDA
  _cmd(0x02);
  _cmd(SSD1306_SETCONTRAST);                   // 0x81
  _cmd(0x8F);
  _cmd(SSD1306_SETPRECHARGE);                  // 0xd9
  _cmd(0xF1);
  _cmd(SSD1306_SETVCOMDETECT);                 // 0xDB
  _cmd(0x40);
  _cmd(SSD1306_DISPLAYALLON_RESUME);           // 0xA4
  _cmd(SSD1306_NORMALDISPLAY);                 // 0xA6
  _cmd(SSD1306_INVERTDISPLAY);
}

void OLED_SSD1306::_cmd(uint8_t d) {
  _dc->write(0);
  _cs->write(0);
  _spi->transfer(d);
  _cs->write(1);
}

void OLED_SSD1306::_invert() {
  _cmd(SSD1306_INVERTDISPLAY);
}

void OLED_SSD1306::_write_buf() {
  _cmd(SSD1306_SETLOWCOLUMN);
  _cmd(SSD1306_SETHIGHCOLUMN);
  _cmd(SSD1306_SETSTARTLINE);

  _dc->write(1);
  _cs->write(0);

  for (int i = 0; i < SCREENBUFSIZE; i++) {
    _spi->transfer(_buf[i]);
  }
  
  for (int i = 0; i < SCREENBUFSIZE; i++) {
    _spi->transfer(0);
  }

  _cs->write(1);
}
