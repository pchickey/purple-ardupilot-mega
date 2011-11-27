/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
	DataFlashHW_APM2.cpp - DataFlash log library for AT45DB321D
	Code by Jordi Muñoz and Jose Julio. DIYDrones.com
	This code works only on ATMega2560. It uses Serial port 3 in SPI MSPI mdoe.

	This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

	Dataflash library for AT45DB321D flash memory
	Memory organization : 8192 pages of 512 bytes or 528 bytes

	Maximun write bandwidth : 512 bytes in 14ms
	This code is written so the master never has to wait to write the data on the eeprom

	Methods:
		Init() : Library initialization (SPI initialization)
		StartWrite(page) : Start a write session. page=start page.
		WriteByte(data) : Write a byte
		WriteInt(data) :  Write an integer (2 bytes)
		WriteLong(data) : Write a long (4 bytes)
		StartRead(page) : Start a read on (page)
		GetWritePage() : Returns the last page written to
		GetPage() : Returns the last page read
		ReadByte()
		ReadInt()
		ReadLong()

	Properties:

*/

// AVR LibC Includes
#include <inttypes.h>
#include <avr/interrupt.h>
#include "WConstants.h"

#include "DataFlashHW_APM2.h"

// DataFlash is connected to Serial Port 3 (we will use SPI mode)
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
	#define DF_DATAOUT 14        // MOSI
	#define DF_DATAIN  15        // MISO
	#define DF_SPICLOCK  PJ2     // SCK
	#define DF_SLAVESELECT 28    // SS     (PA6)
    #define DF_RESET 41          // RESET  (PG0)
    #define DF_CARDDETECT 33     // PC4
#else
	# error Please check the Tools/Board menu to ensure you have selected Arduino Mega as your target.
#endif

#define DF_MAX_PAGE 8192

// AT45DB321D Commands (from Datasheet)
#define DF_TRANSFER_PAGE_TO_BUFFER_1   0x53
#define DF_TRANSFER_PAGE_TO_BUFFER_2   0x55
#define DF_STATUS_REGISTER_READ   0xD7
#define DF_READ_MANUFACTURER_AND_DEVICE_ID   0x9F
#define DF_PAGE_READ   0xD2
#define DF_BUFFER_1_READ   0xD4
#define DF_BUFFER_2_READ   0xD6
#define DF_BUFFER_1_WRITE   0x84
#define DF_BUFFER_2_WRITE   0x87
#define DF_BUFFER_1_TO_PAGE_WITH_ERASE   0x83
#define DF_BUFFER_2_TO_PAGE_WITH_ERASE   0x86
#define DF_PAGE_ERASE   0x81
#define DF_BLOCK_ERASE   0x50
#define DF_SECTOR_ERASE   0x7C
#define DF_CHIP_ERASE_0   0xC7
#define DF_CHIP_ERASE_1   0x94
#define DF_CHIP_ERASE_2   0x80
#define DF_CHIP_ERASE_3   0x9A


#define OVERWRITE_DATA 0 // 0: When reach the end page stop, 1: Start overwritten from page 1

// Private Methods //////////////////////////////////////////////////////////////
uint8_t DataFlashHW_APM2::_spi_transfer(uint8_t data)
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR3A & (1<<UDRE3)) );
	/* Put data into buffer, sends the data */
	UDR3 = data;
	/* Wait for data to be received */
	while ( !(UCSR3A & (1<<RXC3)) );
	/* Get and return received data from buffer */
	return UDR3;
}

void DataFlashHW_APM2::_cs_inactive()
{
  digitalWrite(DF_SLAVESELECT,HIGH); //disable device
}

void DataFlashHW_APM2::_cs_active()
{
  digitalWrite(DF_SLAVESELECT,LOW); //enable device
}

// Public Methods //////////////////////////////////////////////////////////////
void DataFlashHW_APM2::init(void)
{
  pinMode(DF_DATAOUT, OUTPUT);
  pinMode(DF_DATAIN, INPUT);
  pinMode(DF_SLAVESELECT,OUTPUT);
  pinMode(DF_RESET,OUTPUT);
  pinMode(DF_CARDDETECT, INPUT);

  // Reset the chip
  digitalWrite(DF_RESET,LOW);
  delay(1);
  digitalWrite(DF_RESET,HIGH);

  _cs_inactive();     //disable device

  // Setup Serial Port3 in SPI mode (MSPI), Mode 0, Clock: 8Mhz
  UBRR3 = 0;
  DDRJ |= (1<<PJ2);                                   // SPI clock XCK3 (PJ2) as output. This enable SPI Master mode
  // Set MSPI mode of operation and SPI data mode 0.
  UCSR3C = (1<<UMSEL31)|(1<<UMSEL30);       //|(1<<1)|(1<<UCPOL3);
  // Enable receiver and transmitter.
  UCSR3B = (1<<RXEN3)|(1<<TXEN3);
  // Set Baud rate
  UBRR3 = 0;         // SPI running at 8Mhz

  // get page size: 512 or 528  (by default: 528)
  _page_size = _read_page_size();
}

// This function is mainly to test the device
void DataFlashHW_APM2::_read_manufacturer_id()
{
  _cs_inactive();    // Reset dataflash command decoder
  _cs_active();

  // Read manufacturer and ID command...
  _spi_transfer(DF_READ_MANUFACTURER_AND_DEVICE_ID);

  _manufacturer = _spi_transfer(0xff);
  _device_0 = _spi_transfer(0xff);
  _device_1 = _spi_transfer(0xff);
  _spi_transfer(0xff);
}

// This function return 1 if Card is inserted on SD slot
bool DataFlashHW_APM2::card_inserted()
{
    return (digitalRead(DF_CARDDETECT) != 0);
}

// Read the status register
uint8_t DataFlashHW_APM2::_read_status_reg()
{
  _cs_inactive();    // Reset dataflash command decoder
  _cs_active();

  // Read status command
  _spi_transfer(DF_STATUS_REGISTER_READ);
  return _spi_transfer(0x00);  // We only want to extract the READY/BUSY bit
}

// Read the status of the DataFlash
uint8_t DataFlashHW_APM2::_read_status_busy()
{
  return(_read_status_reg()&0x80);  // We only want to extract the READY/BUSY bit
}


uint16_t DataFlashHW_APM2::_read_page_size()
{
  return(528-((_read_status_reg()&0x01)<<4));  // if first bit 1 trhen 512 else 528 bytes
}


// Wait until DataFlash is in ready state...
void DataFlashHW_APM2::wait_ready()
{
  while(!_read_status_busy());
}

void DataFlashHW_APM2::page_to_buffer(uint8_t buffer_num, uint16_t page_addr)
{
  _cs_inactive();
  _cs_active();
  if (buffer_num==1)
    _spi_transfer(DF_TRANSFER_PAGE_TO_BUFFER_1);
  else
    _spi_transfer(DF_TRANSFER_PAGE_TO_BUFFER_2);

  if(_page_size == 512){
    _spi_transfer((unsigned char)(page_addr >> 7));
    _spi_transfer((unsigned char)(page_addr << 1));
  }else{
    _spi_transfer((unsigned char)(page_addr >> 6));
    _spi_transfer((unsigned char)(page_addr << 2));
  }
  _spi_transfer(0x00);	// don´t care bytes

  _cs_inactive();	//initiate the transfer
  _cs_active();

  wait_ready();
}

void DataFlashHW_APM2::buffer_to_page(unsigned char buffer_num, uint16_t page_addr,
                                      unsigned char wait)
{
  _cs_inactive();     // Reset dataflash command decoder
  _cs_active();

  if (buffer_num==1)
    _spi_transfer(DF_BUFFER_1_TO_PAGE_WITH_ERASE);
  else
    _spi_transfer(DF_BUFFER_2_TO_PAGE_WITH_ERASE);

  if(_page_size == 512){
    _spi_transfer((unsigned char)(page_addr >> 7));
    _spi_transfer((unsigned char)(page_addr << 1));
  }else{
    _spi_transfer((unsigned char)(page_addr >> 6));
    _spi_transfer((unsigned char)(page_addr << 2));
  }
  _spi_transfer(0x00);	// don´t care bytes

  _cs_inactive();	//initiate the transfer
  _cs_active();

  // Check if we need to wait to write the buffer to memory or we can continue...
  if (wait)
    wait_ready();
}

void DataFlashHW_APM2::buffer_write(uint8_t buffer_num, uint16_t page_addr, unsigned char data)
{
  _cs_inactive();   // Reset dataflash command decoder
  _cs_active();

  if (buffer_num == 1)
    _spi_transfer(DF_BUFFER_1_WRITE);
  else
    _spi_transfer(DF_BUFFER_2_WRITE);
  _spi_transfer(0x00);				 //don't cares
  _spi_transfer((unsigned char)(page_addr >> 8));  //upper part of internal buffer address
  _spi_transfer((unsigned char)(page_addr));	 //lower part of internal buffer address
  _spi_transfer(data);				 //write data byte
}

uint8_t DataFlashHW_APM2::buffer_read(unsigned char buffer_num, uint16_t page_addr)
{
  uint8_t tmp;

  _cs_inactive();   // Reset dataflash command decoder
  _cs_active();

  if (buffer_num == 1)
    _spi_transfer(DF_BUFFER_1_READ);
  else
    _spi_transfer(DF_BUFFER_2_READ);
  _spi_transfer(0x00);				 //don't cares
  _spi_transfer((unsigned char)(page_addr >> 8));  //upper part of internal buffer address
  _spi_transfer((unsigned char)(page_addr));	 //lower part of internal buffer address
  _spi_transfer(0x00);                            //don't cares
  tmp = _spi_transfer(0x00);		         //read data byte

  return (tmp);
}

void DataFlashHW_APM2::page_erase(uint16_t page_addr)
{
  _cs_inactive();																//make sure to toggle CS signal in order
  _cs_active();																//to reset Dataflash command decoder
  _spi_transfer(DF_PAGE_ERASE);   // Command

  if(_page_size==512){
    _spi_transfer((unsigned char)(page_addr >> 7));
    _spi_transfer((unsigned char)(page_addr << 1));
  }else{
    _spi_transfer((unsigned char)(page_addr >> 6));
    _spi_transfer((unsigned char)(page_addr << 2));
  }

  _spi_transfer(0x00);	           // "dont cares"
  _cs_inactive();               //initiate flash page erase
  _cs_active();
  wait_ready();
}


void DataFlashHW_APM2::chip_erase()
{
  _cs_inactive();																//make sure to toggle CS signal in order
  _cs_active();																//to reset Dataflash command decoder
  // opcodes for chip erase
  _spi_transfer(DF_CHIP_ERASE_0);
  _spi_transfer(DF_CHIP_ERASE_1);
  _spi_transfer(DF_CHIP_ERASE_2);
  _spi_transfer(DF_CHIP_ERASE_3);

  _cs_inactive();               //initiate flash page erase
  _cs_active();
  wait_ready();
}

uint16_t DataFlashHW_APM2::last_page()
{
  return 4096;
}

