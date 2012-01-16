// -*-  tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
//
// RTOS concurrent serial transmit/recieve library, for use with
// FreeRTOS kernel. Built and tested with the ArduinoFreeRTOS library.
// Pat Hickey, Dec 2011.
//
// Interrupt-driven serial transmit/receive library.
//
//      Copyright (c) 2010 Michael Smith. All rights reserved.
//
// Receive and baudrate calculations derived from the Arduino
// HardwareSerial driver:
//
//      Copyright (c) 2006 Nicholas Zambetti.  All right reserved.
//
// Transmit algorithm inspired by work:
//
//      Code Jose Julio and Jordi Munoz. DIYDrones.com
//
//      This library is free software; you can redistribute it and/or
//      modify it under the terms of the GNU Lesser General Public
//      License as published by the Free Software Foundation; either
//      version 2.1 of the License, or (at your option) any later version.
//
//      This library is distributed in the hope that it will be useful,
//      but WITHOUT ANY WARRANTY; without even the implied warranty of
//      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//      Lesser General Public License for more details.
//
//      You should have received a copy of the GNU Lesser General Public
//      License along with this library; if not, write to the Free Software
//      Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
//


//#include "../AP_Common/AP_Common.h"
#include "RTOSSerial.h"
#include "WProgram.h"

#if   defined(UDR3)
# define FS_MAX_PORTS   4
#elif defined(UDR2)
# define FS_MAX_PORTS   3
#elif defined(UDR1)
# define FS_MAX_PORTS   2
#else
# define FS_MAX_PORTS   1
#endif

#if 0
RTOSSerial::Buffer __RTOSSerial__rxBuffer[FS_MAX_PORTS];
RTOSSerial::Buffer __RTOSSerial__txBuffer[FS_MAX_PORTS];
#else
xQueueHandle __RTOSSerial__rxQueue[FS_MAX_PORTS];
xQueueHandle __RTOSSerial__txQueue[FS_MAX_PORTS];
#endif
uint8_t RTOSSerial::_serialInitialized = 0;

// Constructor /////////////////////////////////////////////////////////////////

RTOSSerial::RTOSSerial(const uint8_t portNumber, volatile uint8_t *ubrrh, volatile uint8_t *ubrrl,
					   volatile uint8_t *ucsra, volatile uint8_t *ucsrb, const uint8_t u2x,
					   const uint8_t portEnableBits, const uint8_t portTxBits) :
					   _ubrrh(ubrrh),
					   _ubrrl(ubrrl),
					   _ucsra(ucsra),
					   _ucsrb(ucsrb),
					   _u2x(u2x),
					   _portEnableBits(portEnableBits),
					   _portTxBits(portTxBits),
					   _rxQueue(&__RTOSSerial__rxQueue[portNumber]),
					   _txQueue(&__RTOSSerial__txQueue[portNumber])
{
	setInitialized(portNumber);
	begin(57600);
}

// Public Methods //////////////////////////////////////////////////////////////

void RTOSSerial::begin(long baud)
{
	begin(baud, 0, 0);
}

void RTOSSerial::begin(long baud, unsigned int rxSpace, unsigned int txSpace)
{
	uint16_t ubrr;
	bool use_u2x = true;

	// if we are currently open...
	if (_open) {
		// close the port in its current configuration, clears _open
		end();
	}

	// allocate buffers
#if 0
	if (!_allocBuffer(_rxBuffer, rxSpace ? : _default_rx_buffer_size) || !_allocBuffer(_txBuffer, txSpace ?	: _default_tx_buffer_size)) {
		end();
		return; // couldn't allocate buffers - fatal
	}
#else
  if (_rxQueue == 0 || _txQueue == 0) {
  *_rxQueue = xQueueCreate( _rxQueueSpace, sizeof( uint8_t ));
  *_txQueue = xQueueCreate( _txQueueSpace, sizeof( uint8_t ));
  if (*_rxQueue == NULL || *_txQueue == NULL ) {
    end();
    return; // couldn't allocate queues - fatal
  }
  _txsize = txSpace;
#endif

#if 0
	// reset buffer pointers
	_txBuffer->head = _txBuffer->tail = 0;
	_rxBuffer->head = _rxBuffer->tail = 0;
#endif

	// mark the port as open
	_open = true;

	// If the user has supplied a new baud rate, compute the new UBRR value.
	if (baud > 0) {
#if F_CPU == 16000000UL
		// hardcoded exception for compatibility with the bootloader shipped
		// with the Duemilanove and previous boards and the firmware on the 8U2
		// on the Uno and Mega 2560.
		if (baud == 57600)
			use_u2x = false;
#endif

		if (use_u2x) {
			*_ucsra = 1 << _u2x;
			ubrr = (F_CPU / 4 / baud - 1) / 2;
		} else {
			*_ucsra = 0;
			ubrr = (F_CPU / 8 / baud - 1) / 2;
		}

		*_ubrrh = ubrr >> 8;
		*_ubrrl = ubrr;
	}

	*_ucsrb |= _portEnableBits;
}

void RTOSSerial::end()
{
	*_ucsrb &= ~(_portEnableBits | _portTxBits);

  vQueueDelete(*_rxQueue);
  vQueueDelete(*_txQueue);
#if 0
	_freeBuffer(_rxBuffer);
	_freeBuffer(_txBuffer);
#else
#endif
	_open = false;
}

int RTOSSerial::available(void)
{
	if (!_open)
		return (-1);
  return uxQueueMessagesWaiting(*_rxQueue );
#if 0
	return ((_rxBuffer->head - _rxBuffer->tail) & _rxBuffer->mask);
#else
#endif
}

int RTOSSerial::txspace(void)
{
  unsigned portBASE_TYPE avail;
	if (!_open)
		return (-1);
  avail = uxQueueMessagesWaiting(*_rxQueue );
  return (int) (_txQueueSpace - avail);
#if 0
	return ((_txBuffer->mask+1) - ((_txBuffer->head - _txBuffer->tail) & _txBuffer->mask));
#else
#endif
}

int RTOSSerial::read(void)
{
	uint8_t c;
#if 0
	// if the head and tail are equal, the buffer is empty
	if (!_open || (_rxBuffer->head == _rxBuffer->tail))
		return (-1);

	// pull character from tail
	c = _rxBuffer->bytes[_rxBuffer->tail];
	_rxBuffer->tail = (_rxBuffer->tail + 1) & _rxBuffer->mask;
	return (c);
#else
  if (!_open) return (-1);
  if ( xQueueReceive(*_rxQueue, &c, ( portTickType ) 0 ) != errQUEUE_EMPTY) {
    return (int) c;
  }
  return (-1);
  // TODO
#endif
}

int RTOSSerial::peek(void)
{
#if 0
	// if the head and tail are equal, the buffer is empty
	if (!_open || (_rxBuffer->head == _rxBuffer->tail))
		return (-1);

	// pull character from tail
	return (_rxBuffer->bytes[_rxBuffer->tail]);
#else
  uint8_t c;
  if (!_open) return (-1);
  if ( xQueuePeek (*_rxQueue, &c, ( portTickType ) 0 ) != errQUEUE_EMPTY) {
    return (int) c;
  }
  return (-1);
#endif
}

void RTOSSerial::flush(void)
{
#if 0
	// don't reverse this or there may be problems if the RX interrupt
	// occurs after reading the value of _rxBuffer->head but before writing
	// the value to _rxBuffer->tail; the previous value of head
	// may be written to tail, making it appear as if the buffer
	// don't reverse this or there may be problems if the RX interrupt
	// occurs after reading the value of head but before writing
	// the value to tail; the previous value of rx_buffer_head
	// may be written to tail, making it appear as if the buffer
	// were full, not empty.
	_rxBuffer->head = _rxBuffer->tail;

	// don't reverse this or there may be problems if the TX interrupt
	// occurs after reading the value of _txBuffer->tail but before writing
	// the value to _txBuffer->head.
	_txBuffer->tail = _txBuffer->head;
#else
  // TODO
#endif
}

void RTOSSerial::write(uint8_t c)
{
	uint16_t i;

	if (!_open) // drop bytes if not open
		return;
#if 0
	// wait for room in the tx buffer
	i = (_txBuffer->head + 1) & _txBuffer->mask;
	while (i == _txBuffer->tail)
		;

  xQueueSendToBack(*_txQueue, (void *) &c, portMAX_DELAY );
	// add byte to the buffer
	_txBuffer->bytes[_txBuffer->head] = c;
	_txBuffer->head = i;
#else
#endif
	// enable the data-ready interrupt, as it may be off if the buffer is empty
	*_ucsrb |= _portTxBits;
}

// Buffer management ///////////////////////////////////////////////////////////
#if 0
bool RTOSSerial::_allocBuffer(Buffer *buffer, unsigned int size)
{
	uint16_t	mask;
	uint8_t		shift;

	// init buffer state
	buffer->head = buffer->tail = 0;

	// Compute the power of 2 greater or equal to the requested buffer size
	// and then a mask to simplify wrapping operations.  Using __builtin_clz
	// would seem to make sense, but it uses a 256(!) byte table.
	// Note that we ignore requests for more than BUFFER_MAX space.
	for (shift = 1; (1U << shift) < min(_max_buffer_size, size); shift++)
		;
	mask = (1 << shift) - 1;

	// If the descriptor already has a buffer allocated we need to take
	// care of it.
	if (buffer->bytes) {

		// If the allocated buffer is already the correct size then
		// we have nothing to do
		if (buffer->mask == mask)
			return true;

		// Dispose of the old buffer.
		free(buffer->bytes);
	}
	buffer->mask = mask;

	// allocate memory for the buffer - if this fails, we fail.
	buffer->bytes = (uint8_t *) malloc(buffer->mask + 1);

	return (buffer->bytes != NULL);
}

void RTOSSerial::_freeBuffer(Buffer *buffer)
{
	buffer->head = buffer->tail = 0;
	buffer->mask = 0;
	if (NULL != buffer->bytes) {
		free(buffer->bytes);
		buffer->bytes = NULL;
	}
}
#endif

