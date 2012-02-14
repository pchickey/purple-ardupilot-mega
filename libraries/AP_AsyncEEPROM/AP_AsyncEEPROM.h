// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#ifndef __AP_ASYNC_EEPROM_H__
#define __AP_ASYNC_EEPROM_H__

#include <stdint.h>

/// @class	AP_AsyncEEPROM
/// @brief	Buffered reading and writing to the EEPROM
class AP_AsyncEEPROM {
  public:
  AP_AsyncEEPROM() {}

  void init();

	uint8_t  read_uint8  (uint16_t address);
	uint16_t read_uint16 (uint16_t address);
	uint32_t read_uint32 (uint16_t address);
	float	   read_float  (uint16_t address);
	
	void	write_uint8  (uint16_t address, uint8_t  value);
	void	write_uint16 (uint16_t address, uint16_t value);
	void	write_uint32 (uint16_t address, uint32_t value);
	void	write_float  (uint16_t address, float   value);

};

#endif // __AP_ASYNC_EEPROM_H__
