
#include "DataFlashHW_APM1.h"
#include <SPI.h>

// arduino mega SPI pins
#define DF_DATAOUT 51        // MOSI
#define DF_DATAIN  50        // MISO
#define DF_SPICLOCK  52      // SCK
#define DF_SLAVESELECT 53    // SS     (PB0)
#define DF_RESET 31          // RESET  (PC6)

// AT45DB161D Commands (from Datasheet)
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

void DataFlashHW_APM1::init()
{
  pinMode(DF_DATAOUT, OUTPUT);
  pinMode(DF_DATAIN, INPUT);
  pinMode(DF_SPICLOCK,OUTPUT);
  pinMode(DF_SLAVESELECT,OUTPUT);
	pinMode(DF_RESET,OUTPUT);
	// Reset the chip
	digitalWrite(DF_RESET,LOW);
	delay(1);
	digitalWrite(DF_RESET,HIGH);

  _cs_inactive();     //disable device

  // Setup SPI  Master, Mode 3, fosc/4 = 4MHz
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE3);
  SPI.setClockDivider(SPI_CLOCK_DIV2);

  _page_size = _read_page_size();
}

uint16_t DataFlashHW_APM1::last_page()
{
  return 4096;
}

void DataFlashHW_APM1::read_manufacturer_id()
{
  _cs_active();     // activate dataflash command decoder

  // Read manufacturer and ID command...
  SPI.transfer(DF_READ_MANUFACTURER_AND_DEVICE_ID);

  _manufacturer = SPI.transfer(0xff);
  _device_0 = SPI.transfer(0xff);
  _device_1 = SPI.transfer(0xff);
  SPI.transfer(0xff);

  _cs_inactive();    // Reset dataflash command decoder

}

int16_t DataFlashHW_APM1::get_page()
{
  return 0;
}

int16_t DataFlashHW_APM1::get_write_page()
{ 
  return 0;
}

void DataFlashHW_APM1::page_erase(uint16_t page_addr)
{
  _cs_active();     // activate dataflash command decoder
  SPI.transfer(DF_PAGE_ERASE);   // Command

  if(_page_size==512){
    SPI.transfer((unsigned char)(page_addr >> 7));
    SPI.transfer((unsigned char)(page_addr << 1));
  }else{
    SPI.transfer((unsigned char)(page_addr >> 6));
    SPI.transfer((unsigned char)(page_addr << 2));
  }

  SPI.transfer(0x00);	           // "dont cares"
  _cs_inactive();               //initiate flash page erase
  _cs_active();
  while(!read_status_busy());

  _cs_inactive();   // deactivate dataflash command decoder
}

void DataFlashHW_APM1::chip_erase()
{

  _cs_active();     // activate dataflash command decoder
  // opcodes for chip erase
  SPI.transfer(DF_CHIP_ERASE_0);
  SPI.transfer(DF_CHIP_ERASE_1);
  SPI.transfer(DF_CHIP_ERASE_2);
  SPI.transfer(DF_CHIP_ERASE_3);

  _cs_inactive();               //initiate flash page erase
  _cs_active();
  while(!read_status_busy());

  _cs_inactive();   // deactivate dataflash command decoder
}

uint8_t DataFlashHW_APM1::read_status_reg()
{
  uint8_t tmp;

  _cs_active();     // activate dataflash command decoder

  // Read status command
  SPI.transfer(DF_STATUS_REGISTER_READ);
  tmp = SPI.transfer(0x00);  // We only want to extract the READY/BUSY bit

  _cs_inactive();    // Reset dataflash command decoder
  return tmp;
}

uint8_t DataFlashHW_APM1::read_status_busy()
{
  return (read_status_reg() & 0x80); // Extract READ/BUSY bit
}

void DataFlashHW_APM1::page_to_buffer(uint8_t buffer_num, uint16_t page_addr)
{
  _cs_active();     // activate dataflash command decoder

  if (buffer_num==1)
    SPI.transfer(DF_TRANSFER_PAGE_TO_BUFFER_1);
  else
    SPI.transfer(DF_TRANSFER_PAGE_TO_BUFFER_2);

  if(_page_size==512){
    SPI.transfer((unsigned char)(page_addr >> 7));
    SPI.transfer((unsigned char)(page_addr << 1));
  }else{
    SPI.transfer((unsigned char)(page_addr >> 6));
    SPI.transfer((unsigned char)(page_addr << 2));
  }
  SPI.transfer(0x00);	// don´t care bytes

  _cs_inactive();	//initiate the transfer
  _cs_active();

  while(!read_status_busy());  //monitor the status register, wait until busy-flag is high

  _cs_inactive();
}

void DataFlashHW_APM1::buffer_to_page(uint8_t buffer_num, uint16_t page_addr, uint8_t wait)
{
  _cs_active();     // activate dataflash command decoder

  if (buffer_num==1)
    SPI.transfer(DF_BUFFER_1_TO_PAGE_WITH_ERASE);
  else
    SPI.transfer(DF_BUFFER_2_TO_PAGE_WITH_ERASE);

  if(_page_size==512){
    SPI.transfer((unsigned char)(page_addr >> 7));
    SPI.transfer((unsigned char)(page_addr << 1));
  }else{
    SPI.transfer((unsigned char)(page_addr >> 6));
    SPI.transfer((unsigned char)(page_addr << 2));
  }
  SPI.transfer(0x00);	// don´t care bytes

  _cs_inactive();	//initiate the transfer
  _cs_active();

  // Check if we need to wait to write the buffer to memory or we can continue...
  if (wait)
	while(!read_status_busy());  //monitor the status register, wait until busy-flag is high

  _cs_inactive();	//deactivate dataflash command decoder
}

void DataFlashHW_APM1::buffer_write(uint8_t buffer_num, uint16_t page_addr, uint8_t data)
{
  _cs_active();     // activate dataflash command decoder

  if (buffer_num==1)
    SPI.transfer(DF_BUFFER_1_WRITE);
  else
    SPI.transfer(DF_BUFFER_2_WRITE);
  SPI.transfer(0x00);				 //don't cares
  SPI.transfer((unsigned char)(page_addr>>8));  //upper part of internal buffer address
  SPI.transfer((unsigned char)(page_addr));	 //lower part of internal buffer address
  SPI.transfer(data);				 //write data byte

  _cs_inactive();   // disable dataflash command decoder
}

uint8_t DataFlashHW_APM1::buffer_read(uint8_t buffer_num, uint16_t page_addr)
{
  uint8_t tmp;

  _cs_active();     // activate dataflash command decoder

  if (buffer_num ==1)
    SPI.transfer(DF_BUFFER_1_READ);
  else
    SPI.transfer(DF_BUFFER_2_READ);
  SPI.transfer(0x00);				 //don't cares
  SPI.transfer((unsigned char)(page_addr>>8));  //upper part of internal buffer address
  SPI.transfer((unsigned char)(page_addr));	 //lower part of internal buffer address
  SPI.transfer(0x00);                            //don't cares
  tmp = SPI.transfer(0x00);		         //read data byte

  _cs_inactive();   // deactivate dataflash command decoder

  return (tmp);
}

/* ---------------------------- PRIVATE FUNCTIONS ----------------------------- */

uint16_t DataFlashHW_APM1::_read_page_size()
{
  return(528-((read_status_reg()&0x01)<<4));  // if first bit 1 trhen 512 else 528 bytes
}

void DataFlashHW_APM1::_cs_inactive()
{
  digitalWrite(DF_SLAVESELECT,HIGH); //disable device
}

void DataFlashHW_APM1::_cs_active()
{
  digitalWrite(DF_SLAVESELECT,LOW); //enable device
}

