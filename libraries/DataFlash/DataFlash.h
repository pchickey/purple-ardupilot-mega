/* ************************************************************ */
/* Test for DataFlash Log library                               */
/* ************************************************************ */
#ifndef DataFlash_h
#define DataFlash_h

class DataFlash_Class
{
  public:
	DataFlash_Class() {} // Constructor
	
  virtual void Init() = 0;
	virtual void ReadManufacturerID() = 0;
	virtual int GetPage() = 0;
	virtual int GetWritePage() = 0;
	virtual void PageErase (unsigned int PageAdr) = 0;
	virtual void ChipErase () = 0;
	
  // Write methods
	virtual void StartWrite(int PageAdr) = 0;
	virtual void FinishWrite() = 0;
	virtual void WriteByte(unsigned char data) = 0;
	virtual void WriteInt(int data) = 0;
	virtual void WriteLong(long data) = 0;

	// Read methods
	virtual void StartRead(int PageAdr) = 0;
	virtual unsigned char ReadByte() = 0;
	virtual int ReadInt() = 0;
	virtual long ReadLong() = 0;
};

#include "DataFlash_APM1.h"

#endif
