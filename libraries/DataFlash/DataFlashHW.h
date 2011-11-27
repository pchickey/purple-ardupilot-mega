
#ifndef __DATAFLASH_HW_H__
#define __DATAFLASH_HW_H__

#include <stdint.h>

class DataFlashHW
{
  public:
  virtual void    init() = 0;
  virtual void    read_manufacturer_id() = 0;
  virtual int16_t get_page() = 0;
  virtual int16_t get_write_page() = 0;
  virtual void    page_erase(uint16_t page_addr) = 0;
  virtual void    chip_erase() = 0;
  virtual uint8_t read_status_reg() = 0;
  virtual uint8_t read_status_busy() = 0;
  virtual void page_to_buffer(uint8_t buffer_num, uint16_t page_addr) = 0;
  virtual void buffer_to_page(uint8_t buffer_num, uint16_t page_addr, uint8_t wait) = 0;
};

#endif // __DATAFLASH_HW_H__

