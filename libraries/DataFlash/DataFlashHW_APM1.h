
#ifndef __DATAFLASH_HW_APM1_H__
#define __DATAFLASH_HW_APM1_H__

#include "DataFlashHW.h"

class DataFlashHW_APM1 : public DataFlashHW
{
  public:
  void     init();
  void     wait_ready();

  void     buffer_write(uint8_t buffer_num, uint16_t page_addr, uint8_t data);
  uint8_t  buffer_read(uint8_t buffer_num, uint16_t page_addr);

  void     page_to_buffer(uint8_t buffer_num, uint16_t page_addr);
  void     buffer_to_page(uint8_t buffer_num, uint16_t page_addr, uint8_t wait);

  void     page_erase(uint16_t page_addr);
  void     chip_erase();

  uint16_t last_page();

  bool     card_inserted() { return true; } /* Always inserted on APM1 */

  private:
  uint16_t _page_size;

  uint8_t  _read_status_reg();
  uint8_t  _read_status_busy();

  void     _read_manufacturer_id();
  uint16_t _read_page_size();

  void     _cs_active();
  void     _cs_inactive();

  uint8_t  _manufacturer;
  uint8_t  _device_0;
  uint8_t  _device_1;
};

#endif // __DATAFLASH_HW_APM1_H__

