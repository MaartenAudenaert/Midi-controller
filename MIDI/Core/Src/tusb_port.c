/*
 * tusb_port.c
 * TinyUSB board support functions for STM32
 */

#include "tusb.h"
#include "main.h"

// Board porting API: Get current millisecond for TinyUSB timeout
uint32_t board_millis(void)
{
  return HAL_GetTick();
}

// Implement weak symbol for TinyUSB
TU_ATTR_WEAK uint32_t tusb_time_millis_api(void)
{
  return board_millis();
}
