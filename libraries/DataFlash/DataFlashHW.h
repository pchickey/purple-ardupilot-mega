
#ifndef __DATAFLASH_HW_H__
#define __DATAFLASH_HW_H__

#include <stdint.h>

class DataFlashHW
{
  public:
  virtual void    init() = 0;
  virtual void    wait_ready() = 0;

  virtual void    buffer_write(uint8_t buffer_num, uint16_t page_addr, uint8_t data) = 0;
  virtual uint8_t buffer_read(uint8_t buffer_num, uint16_t page_addr) = 0;

  virtual void    page_to_buffer(uint8_t buffer_num, uint16_t page_addr) = 0;
  virtual void    buffer_to_page(uint8_t buffer_num, uint16_t page_addr, uint8_t wait) = 0;

  virtual void    page_erase(uint16_t page_addr) = 0;
  virtual void    chip_erase() = 0;

  virtual uint16_t last_page() = 0;
  virtual uint16_t get_page_size() = 0;

  virtual bool    card_inserted() = 0;

};

#endif // __DATAFLASH_HW_H__

