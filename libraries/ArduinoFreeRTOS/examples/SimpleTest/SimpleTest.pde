#include <avr/io.h>
#include <ArduinoFreeRTOS.h>


void *task1_handle;
void *task2_handle;

void setup () {
  Serial.begin(115200);

  /* PB5, 6, 7 are outputs, set low */
  DDRB |= _BV(PORTB4) | _BV(PORTB5) | _BV(PORTB6) | _BV(PORTB7);
  PORTB &= ~ (_BV(PORTB4) | _BV(PORTB5) | _BV(PORTB6) | _BV(PORTB7));

  xTaskCreate(task1_func, (signed portCHAR *)"task1", 200,
              NULL, 1, &task1_handle);
  xTaskCreate(task2_func, (signed portCHAR *)"task2", 200,
              NULL, 1, &task2_handle);
  vTaskStartScheduler();
  /* code after vTaskStartScheduler, and code in loop(), is never reached. */
}

void loop () {
  Serial.println("Never reached");
}

void task1_func(void *params)
{
  Serial.println("1: Entering Task");
  for(;;)
      task1_loop(); 
}

void task1_loop() {
   PORTB |= _BV(PORTB5);
   Serial.println("1: Task Loop");
   PORTB &= ~_BV(PORTB5);
   portYIELD();
}


void task2_func(void *params)
{
  Serial.println("2: Entering Task");
  for(;;)
      task2_loop(); 
}

void task2_loop() {
   PORTB |= _BV(PORTB6);
   Serial.println("2: Task Loop");
   PORTB &= ~_BV(PORTB6);
   portYIELD();
}
