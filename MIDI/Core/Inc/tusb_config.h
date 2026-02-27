/*
 * tusb_config.h
 * TinyUSB Configuration for STM32H533RE MIDI Device
 */

#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

// Include STM32H5 HAL for system functions
#include "stm32h5xx.h"
#include "stm32h5xx_hal.h"

//--------------------------------------------------------------------
// COMMON CONFIGURATION
//--------------------------------------------------------------------

// RHPort number used for device
#define BOARD_TUD_RHPORT      0

// RHPort max operational speed
#define BOARD_TUD_MAX_SPEED   OPT_MODE_FULL_SPEED

// RHPort mode - Device mode for MIDI controller
#define CFG_TUSB_RHPORT0_MODE    (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)

// STM32H533 uses USB DRD FS peripheral (FSDEV-based controller)
// Use OPT_MCU_STM32H5 for proper STM32H5 series support
#ifndef CFG_TUSB_MCU
  #define CFG_TUSB_MCU          OPT_MCU_STM32H5
#endif

#ifndef CFG_TUSB_OS
  #define CFG_TUSB_OS           OPT_OS_NONE
#endif

// Enable Device stack
#define CFG_TUD_ENABLED       1

// Debug level: 0=None, 1=Error, 2=Warning, 3=Info
#define CFG_TUSB_DEBUG        0

// USB DMA on some MCUs can only access a specific SRAM region with restriction on alignment.
#ifndef CFG_TUSB_MEM_SECTION
  #define CFG_TUSB_MEM_SECTION
#endif

#ifndef CFG_TUSB_MEM_ALIGN
  #define CFG_TUSB_MEM_ALIGN        __attribute__ ((aligned(4)))
#endif

//--------------------------------------------------------------------
// DEVICE CONFIGURATION
//--------------------------------------------------------------------

#ifndef CFG_TUD_ENDPOINT0_SIZE
  #define CFG_TUD_ENDPOINT0_SIZE    64
#endif

// Device mode with FullSpeed
#define CFG_TUD_MAX_SPEED         OPT_MODE_FULL_SPEED

//------------- CLASS -------------//
#define CFG_TUD_CDC               0
#define CFG_TUD_MSC               0
#define CFG_TUD_HID               0
#define CFG_TUD_MIDI              1  // MIDI device enabled
#define CFG_TUD_VENDOR            0
#define CFG_TUD_AUDIO             0
#define CFG_TUD_VIDEO             0
#define CFG_TUD_ECM_RNDIS         0
#define CFG_TUD_BTH               0

// MIDI FIFO size of TX and RX
#define CFG_TUD_MIDI_RX_BUFSIZE   64
#define CFG_TUD_MIDI_TX_BUFSIZE   64

#ifdef __cplusplus
}
#endif

#endif /* _TUSB_CONFIG_H_ */
