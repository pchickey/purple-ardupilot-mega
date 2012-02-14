
#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include <avr/io.h>
#include <avr/eeprom.h>

#include "AP_AsyncEEPROM.h"

struct write_byte_t {
  uint16_t addr;
  uint8_t data;
};

#define BUFFER_SIZE 32
#define BUFFER_MASK 0x20

struct write_buffer_t {
  write_byte_t a[32];
  uint8_t head;
  uint8_t tail;
};

static volatile write_buffer_t write_buf;

static void write_buf_push (uint16_t a, uint8_t d) {
  const uint8_t i = (write_buf.head + 1) & BUFFER_MASK;
  // Wait for room in the write buffer only if eeprom interrupt is enabled
  while (i == write_buf.tail)
    if (!(EECR | (1 << EERIE))) return;
    ;
  // add addr and data to buffer
  const uint8_t c_head = write_buf.head;
  write_buf.a[c_head].addr = a;
  write_buf.a[c_head].data = d;
  write_buf.head = i;
  
  // Enable the EEPROM ready interrupt
  EECR |= (1 << EERIE);
}


void AP_AsyncEEPROM::init() {
  write_buf.head = 0;
  write_buf.tail = 0;
  for (int i = 0; i < 32; i++) {
    write_buf.a[i].addr = 0;
    write_buf.a[i].data = 0;
  }
}

extern "C" ISR(EE_READY_vect) {
//  if (SPMCR & (1 << SPMEN)) // Return if self-programming is active.
    return;
}

static inline void pause_writing(void) {
  cli(); // Atomic section
  EECR &= ~(1 << EERIE); // Disable EE_READY interrupt
  sei();
}

static inline void wait_to_read(void) {
  while (!eeprom_is_ready()) {}
}

static inline void resume_writing(void) {
  cli(); // Atomic section
  EECR |= (1 << EERIE); // Enable EE_READY interrupt
  sei();
}

uint8_t AP_AsyncEEPROM::read_uint8(uint16_t addr) {
  uint8_t v;
  pause_writing();
  wait_to_read();
  v = eeprom_read_byte( (const uint8_t*) addr ); 
  resume_writing();
  return v;
}

uint16_t AP_AsyncEEPROM::read_uint16(uint16_t addr) {
  uint16_t v;
  pause_writing();
  wait_to_read();
  v = eeprom_read_word( (const uint16_t *) addr );
  resume_writing();
  return v;
}

uint32_t AP_AsyncEEPROM::read_uint32(uint16_t addr) {
  uint32_t v;
  pause_writing();
  wait_to_read();
  v = eeprom_read_dword( (const uint32_t *) addr );
  resume_writing();
  return v;
}

float AP_AsyncEEPROM::read_float(uint16_t addr) {
  union { float f; uint32_t i; } v;
  pause_writing();
  wait_to_read();
  v.i = eeprom_read_dword( (const uint32_t *) addr );
  resume_writing();
  return v.f;
}

void AP_AsyncEEPROM::write_uint8(uint16_t addr, uint8_t v) {
  write_buf_push(addr, v);

}

void AP_AsyncEEPROM::write_uint16(uint16_t addr, uint16_t v) {
  write_buf_push(addr,   (uint8_t)((v >> 0) & 0x00FF));
  write_buf_push(addr+1, (uint8_t)((v >> 8) & 0x00FF)); 
}

void AP_AsyncEEPROM::write_uint32(uint16_t addr, uint32_t v) {
  write_buf_push(addr,  (uint8_t)((v >> 0) & 0x000000FF));
  write_buf_push(addr+1,(uint8_t)((v >> 8) & 0x000000FF));
  write_buf_push(addr+2,(uint8_t)((v >> 16) & 0x000000FF));
  write_buf_push(addr+3,(uint8_t)((v >> 24) & 0x000000FF));
}

void AP_AsyncEEPROM::write_float(uint16_t addr, float v) {
  uint32_t i = *reinterpret_cast<uint32_t *>(&v);
  write_uint32(addr, i);
}

