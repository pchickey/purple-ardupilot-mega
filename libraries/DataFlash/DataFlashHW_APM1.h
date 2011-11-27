
#ifndef __DATAFLASH_HW_APM1_H__
#define __DATAFLASH_HW_APM1_H__

#include "DataFlashHW.h"

class DataFlashHW_APM1 : public DataFlashHW
{
  public:
  void     init();
  void     read_manufacturer_id();
  int16_t  get_page();
  int16_t  get_write_page();
  void     page_erase(uint16_t page_addr);
  void     chip_erase();
  uint8_t  read_status_reg();
  uint8_t  read_status_busy();

  void page_to_buffer(uint8_t buffer_num, uint16_t page_addr);
  void buffer_to_page(uint8_t buffer_num, uint16_t page_addr, uint8_t wait);

  void    buffer_write(uint8_t buffer_num, uint16_t page_addr, uint8_t data);
  uint8_t buffer_read(uint8_t buffer_num, uint16_t page_addr);

  uint16_t last_page();

  private:
  uint16_t _read_page_size();
  uint16_t _page_size;

  void _cs_active();
  void _cs_inactive();

  uint8_t _manufacturer;
  uint8_t _device_0;
  uint8_t _device_1;
};

#endif // __DATAFLASH_HW_APM1_H__

