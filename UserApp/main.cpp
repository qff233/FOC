#include "cmsis_os2.h"
#include "common.h"
#include "projdefs.h"

osThreadId_t ctrl_loop_task_handle;
void thread_ctrl_loop(void *argument) {
  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  }
}

osThreadId_t oled_task_handle;
void thread_oled_update(void *argument) {
  while (true) {
  }
}

void on_timer_callback() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(TaskHandle_t(ctrl_loop_task_handle),
                         &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void Main() {
  const osThreadAttr_t control_loop_task_attributes = {
      .name = "ControlLoopTask",
      .stack_size = 1024,
      .priority = osPriorityRealtime};
  ctrl_loop_task_handle =
      osThreadNew(thread_ctrl_loop, nullptr, &control_loop_task_attributes);

  const osThreadAttr_t oled_task_attributes = {
      .name = "OledTask", .stack_size = 1024, .priority = osPriorityNormal};
  oled_task_handle =
      osThreadNew(thread_oled_update, nullptr, &oled_task_attributes);
}
