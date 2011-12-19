
#include <DuinOS.h>


void *task1_handle;
void *task2_handle;

void setup () {
  Serial.begin(115200);
  xTaskCreate(task1_func, (signed portCHAR *)"task1", configMINIMAL_STACK_SIZE,
              NULL, NORMAL_PRIORITY, &task1_handle);
  xTaskCreate(task2_func, (signed portCHAR *)"task2", configMINIMAL_STACK_SIZE,
              NULL, NORMAL_PRIORITY, &task2_handle);
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
   Serial.println("1: Task Loop");
   portYIELD();
}


void task2_func(void *params)
{
  Serial.println("2: Entering Task");
  for(;;)
      task2_loop(); 
}

void task2_loop() {
   Serial.println("2: Task Loop");
   portYIELD();
}
