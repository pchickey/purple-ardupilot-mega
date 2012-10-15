
#include <AP_Common.h>
#include <AP_HAL.h>
#include <OLED_SSD1306.h>

#include <AP_HAL_AVR.h>
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;

OLED_SSD1306* oled;

const int dc_pin = 13;  /* APM2 OUT10 */
const int rst_pin = 45; /* APM2 OUT11 */
const int cs_pin = 44;  /* APM2 OUT12 */

void setup() {

  AP_HAL::DigitalSource *cs = hal.gpio->channel(cs_pin);
  AP_HAL::DigitalSource *dc = hal.gpio->channel(dc_pin);
  AP_HAL::DigitalSource *rst = hal.gpio->channel(rst_pin);
  
  hal.console->println_P(PSTR("Setting up GPIO"));

  cs->mode(GPIO_OUTPUT);
  cs->write(1);
  dc->mode(GPIO_OUTPUT);
  dc->write(0);
  rst->mode(GPIO_OUTPUT);
  rst->write(0);

  hal.console->println_P(PSTR("Creating OLED_SSD1306 object"));
  oled = new OLED_SSD1306(cs, dc, rst, hal.spi);

  hal.console->println_P(PSTR("OLED_SSD1306 test"));
  oled->test();

  hal.console->println_P(PSTR("Complete"));

}

void loop() {}

extern "C" {
int main (void) {
  hal.init(NULL);
  setup();
  for(;;) loop();
  return 0;
}
}
