/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usb_device.c
  * @version        : v1.0_Cube
  * @brief          : This file implements the USB Device
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USB Device Core handle declaration. */
USBD_HandleTypeDef hUsbDeviceFS;

/*
 * -- Insert your variables declaration here --
 */
/* USER CODE BEGIN 0 */

#define CIRCULAR_BUFFER_SIZE 256
uint8_t cdc_rx_buffer[CIRCULAR_BUFFER_SIZE] = { 0 };
uint8_t cdc_rx_head = 0;
uint8_t cdc_rx_tail = 0;

/* USER CODE END 0 */

/*
 * -- Insert your external function declaration here --
 */
/* USER CODE BEGIN 1 */

void usb_write(uint8_t *buffer, size_t len)
{
	//CDC_Transmit_FS(buffer, len);
	while ( CDC_Transmit_FS(buffer, len) != USBD_OK) {
		LL_mDelay(10);
	}
}
uint8_t usb_check_buffer(void)
{
    uint8_t bytes_in_buffer = 0;
    // When the head and tail of the circular buffer are at different point we have data
    if((cdc_rx_head != cdc_rx_tail))
    {
        // Handle data wraps across the buffer end boundary
        if( cdc_rx_head < cdc_rx_tail)
        {
            bytes_in_buffer = cdc_rx_head + (CIRCULAR_BUFFER_SIZE - cdc_rx_tail);
        }
        else
        {
            bytes_in_buffer = cdc_rx_head - cdc_rx_tail;
        }
    }
    return bytes_in_buffer;
}
uint8_t usb_get_byte(void)
{
    if(cdc_rx_tail == CIRCULAR_BUFFER_SIZE)
    {
        cdc_rx_tail = 0;
    }
    return cdc_rx_buffer[cdc_rx_tail++];
}
void usb_receive_data( uint8_t *data_in, size_t len)
{
    for( size_t i = 0; i < len; i++)
    {
        if(cdc_rx_head == CIRCULAR_BUFFER_SIZE)
        {
            cdc_rx_head = 0;
        }
        // copy from the CDC buffer to the circular buffer
        cdc_rx_buffer[cdc_rx_head++] = data_in[i];
    }
}

/* USER CODE END 1 */

/**
  * Init USB device Library, add supported class and start the library
  * @retval None
  */
void MX_USB_DEVICE_Init(void)
{
  /* USER CODE BEGIN USB_DEVICE_Init_PreTreatment */
  
  /* USER CODE END USB_DEVICE_Init_PreTreatment */
  
  /* Init Device Library, add supported class and start the library. */
  if (USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_Start(&hUsbDeviceFS) != USBD_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN USB_DEVICE_Init_PostTreatment */
  HAL_PWREx_EnableUSBVoltageDetector();
  
  /* USER CODE END USB_DEVICE_Init_PostTreatment */
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
