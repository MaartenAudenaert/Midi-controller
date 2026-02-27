/**
  ******************************************************************************
  * @file    usbd_desc.c
  * @brief   USB Device descriptors
  ******************************************************************************
  */

#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_conf.h"

#define USBD_VID                      0x0483
#define USBD_LANGID_STRING            0x409
#define USBD_MANUFACTURER_STRING      "STMicroelectronics"
#define USBD_PID_FS                   0x5740
#define USBD_PRODUCT_STRING_FS        "STM32 MIDI Device"
#define USBD_CONFIGURATION_STRING_FS  "MIDI Config"
#define USBD_INTERFACE_STRING_FS      "MIDI Interface"

static uint8_t *USBD_MIDI_FS_DeviceDescriptor(USBD_HandleTypeDef *pdev, uint16_t *length);
static uint8_t *USBD_MIDI_FS_LangIDStrDescriptor(USBD_HandleTypeDef *pdev, uint16_t *length);
static uint8_t *USBD_MIDI_FS_ManufacturerStrDescriptor(USBD_HandleTypeDef *pdev, uint16_t *length);
static uint8_t *USBD_MIDI_FS_ProductStrDescriptor(USBD_HandleTypeDef *pdev, uint16_t *length);
static uint8_t *USBD_MIDI_FS_SerialStrDescriptor(USBD_HandleTypeDef *pdev, uint16_t *length);
static uint8_t *USBD_MIDI_FS_ConfigStrDescriptor(USBD_HandleTypeDef *pdev, uint16_t *length);
static uint8_t *USBD_MIDI_FS_InterfaceStrDescriptor(USBD_HandleTypeDef *pdev, uint16_t *length);
static void IntToUnicode(uint32_t value, uint8_t *pbuf, uint8_t len);

USBD_DescriptorsTypeDef MIDI_Desc =
{
  USBD_MIDI_FS_DeviceDescriptor,
  USBD_MIDI_FS_LangIDStrDescriptor,
  USBD_MIDI_FS_ManufacturerStrDescriptor,
  USBD_MIDI_FS_ProductStrDescriptor,
  USBD_MIDI_FS_SerialStrDescriptor,
  USBD_MIDI_FS_ConfigStrDescriptor,
  USBD_MIDI_FS_InterfaceStrDescriptor
};

/* USB Standard Device Descriptor */
__ALIGN_BEGIN uint8_t USBD_FS_DeviceDesc[USB_LEN_DEV_DESC] __ALIGN_END =
{
  0x12,                       /* bLength */
  USB_DESC_TYPE_DEVICE,       /* bDescriptorType */
  0x00,                       /* bcdUSB */
  0x02,
  0x00,                       /* bDeviceClass */
  0x00,                       /* bDeviceSubClass */
  0x00,                       /* bDeviceProtocol */
  USB_MAX_EP0_SIZE,           /* bMaxPacketSize */
  LOBYTE(USBD_VID),           /* idVendor */
  HIBYTE(USBD_VID),           /* idVendor */
  LOBYTE(USBD_PID_FS),        /* idProduct */
  HIBYTE(USBD_PID_FS),        /* idProduct */
  0x00,                       /* bcdDevice rel. 2.00 */
  0x02,
  USBD_IDX_MFC_STR,           /* Index of manufacturer string */
  USBD_IDX_PRODUCT_STR,       /* Index of product string */
  USBD_IDX_SERIAL_STR,        /* Index of serial number string */
  USBD_MAX_NUM_CONFIGURATION  /* bNumConfigurations */
};

/* USB lang identifier descriptor */
__ALIGN_BEGIN uint8_t USBD_LangIDDesc[USB_LEN_LANGID_STR_DESC] __ALIGN_END =
{
  USB_LEN_LANGID_STR_DESC,
  USB_DESC_TYPE_STRING,
  LOBYTE(USBD_LANGID_STRING),
  HIBYTE(USBD_LANGID_STRING)
};

/* Internal string descriptor */
__ALIGN_BEGIN uint8_t USBD_StrDesc[USBD_MAX_STR_DESC_SIZ] __ALIGN_END;

static uint8_t *USBD_MIDI_FS_DeviceDescriptor(USBD_HandleTypeDef *pdev, uint16_t *length)
{
  UNUSED(pdev);
  *length = sizeof(USBD_FS_DeviceDesc);
  return USBD_FS_DeviceDesc;
}

static uint8_t *USBD_MIDI_FS_LangIDStrDescriptor(USBD_HandleTypeDef *pdev, uint16_t *length)
{
  UNUSED(pdev);
  *length = sizeof(USBD_LangIDDesc);
  return USBD_LangIDDesc;
}

static uint8_t *USBD_MIDI_FS_ProductStrDescriptor(USBD_HandleTypeDef *pdev, uint16_t *length)
{
  UNUSED(pdev);
  USBD_GetString((uint8_t *)USBD_PRODUCT_STRING_FS, USBD_StrDesc, length);
  return USBD_StrDesc;
}

static uint8_t *USBD_MIDI_FS_ManufacturerStrDescriptor(USBD_HandleTypeDef *pdev, uint16_t *length)
{
  UNUSED(pdev);
  USBD_GetString((uint8_t *)USBD_MANUFACTURER_STRING, USBD_StrDesc, length);
  return USBD_StrDesc;
}

static uint8_t *USBD_MIDI_FS_SerialStrDescriptor(USBD_HandleTypeDef *pdev, uint16_t *length)
{
  UNUSED(pdev);
  *length = USB_SIZ_STRING_SERIAL;

  /* Use chip unique ID */
  uint32_t deviceserial0 = *(uint32_t *) DEVICE_ID1;
  uint32_t deviceserial1 = *(uint32_t *) DEVICE_ID2;
  uint32_t deviceserial2 = *(uint32_t *) DEVICE_ID3;

  deviceserial0 += deviceserial2;

  if (deviceserial0 != 0)
  {
    IntToUnicode(deviceserial0, &USBD_StrDesc[2], 8);
    IntToUnicode(deviceserial1, &USBD_StrDesc[18], 4);
  }

  return USBD_StrDesc;
}

static uint8_t *USBD_MIDI_FS_ConfigStrDescriptor(USBD_HandleTypeDef *pdev, uint16_t *length)
{
  UNUSED(pdev);
  USBD_GetString((uint8_t *)USBD_CONFIGURATION_STRING_FS, USBD_StrDesc, length);
  return USBD_StrDesc;
}

static uint8_t *USBD_MIDI_FS_InterfaceStrDescriptor(USBD_HandleTypeDef *pdev, uint16_t *length)
{
  UNUSED(pdev);
  USBD_GetString((uint8_t *)USBD_INTERFACE_STRING_FS, USBD_StrDesc, length);
  return USBD_StrDesc;
}

static void IntToUnicode(uint32_t value, uint8_t *pbuf, uint8_t len)
{
  uint8_t idx = 0;

  for (idx = 0; idx < len; idx++)
  {
    if (((value >> 28)) < 0xA)
    {
      pbuf[2 * idx] = (value >> 28) + '0';
    }
    else
    {
      pbuf[2 * idx] = (value >> 28) + 'A' - 10;
    }

    value = value << 4;

    pbuf[2 * idx + 1] = 0;
  }
}
