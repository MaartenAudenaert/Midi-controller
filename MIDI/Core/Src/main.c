/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tusb.h"
#include "mcp23s17.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MATRIX_ROWS            4
#define MATRIX_COLS            4
#define MATRIX_KEYS            16
#define MATRIX_SCAN_MS         2   // Scan every 2ms
#define MATRIX_DEBOUNCE_SCANS  8   // 8 scans = 16ms debounce
#define MIDI_BASE_NOTE         60  // Middle C
#define MIDI_NOTE_VELOCITY     127 // Max velocity
#define MIDI_DIAGNOSTICS       0   // Set to 1 for debugging
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

SPI_HandleTypeDef hspi1;

PCD_HandleTypeDef hpcd_USB_DRD_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  Scan 4x4 matrix and return raw state (triple-sampling with majority vote)
  * @retval 16-bit state (1=pressed, 0=released)
  */
static uint16_t Matrix_ScanRaw(void) {
    // Column output patterns (1 bit LOW per column)
    static const uint8_t col_outputs[MATRIX_COLS] = {
        0x0E,  // 1110 - Column 0 LOW
        0x0D,  // 1101 - Column 1 LOW
        0x0B,  // 1011 - Column 2 LOW
        0x07   // 0111 - Column 3 LOW
    };
    
    uint16_t state = 0;
    
    for (uint8_t col = 0; col < MATRIX_COLS; col++) {
        // Activate column
        MCP_Write(MCP_GPIOA, col_outputs[col]);
        
        // Settle delay (120 NOP cycles ~= 2-3 µs)
        for (volatile uint32_t d = 0; d < 120; d++) { 
            __NOP(); 
        }
        
        // Triple-sample with majority vote (glitch rejection)
        uint8_t s1 = MCP_Read(MCP_GPIOB) & 0x0F;
        uint8_t s2 = MCP_Read(MCP_GPIOB) & 0x0F;
        uint8_t s3 = MCP_Read(MCP_GPIOB) & 0x0F;
        uint8_t row_port = (s1 & s2) | (s1 & s3) | (s2 & s3);
        
        // Check each row
        for (uint8_t row = 0; row < MATRIX_ROWS; row++) {
            if ((row_port & (1 << row)) == 0) {  // Pressed = LOW
                state |= (1 << ((row * MATRIX_COLS) + col));
            }
        }
    }
    
    // All columns HIGH (idle)
    MCP_Write(MCP_GPIOA, 0x0F);
    return state;
}

/**
  * @brief  Send MIDI Note ON/OFF message
  * @param  key_index: Key index (0-15)
  * @param  pressed: 1=pressed, 0=released
  * @retval None
  */
static void Matrix_SendMidiKeyEvent(uint8_t key_index, uint8_t pressed) {
    uint8_t note = MIDI_BASE_NOTE + key_index;
    uint8_t msg[3] = {
        pressed ? 0x90 : 0x80,             // Status byte (Note ON/OFF channel 1)
        note,                               // Note number
        pressed ? MIDI_NOTE_VELOCITY : 0   // Velocity
    };
    
    tud_midi_stream_write(0, msg, 3);  // Cable 0, 3 bytes
}

/**
  * @brief  Update debouncing logic and send MIDI events
  * @param  raw_state: Current raw state from Matrix_ScanRaw
  * @param  stable_state: Pointer to stable state
  * @param  debounce_count: Pointer to debounce counters array
  * @retval None
  */
static void Matrix_UpdateDebounce(uint16_t raw_state, uint16_t *stable_state, 
                                   uint8_t *debounce_count) {
    for (uint8_t i = 0; i < MATRIX_KEYS; i++) {
        uint16_t bit = (1 << i);
        uint8_t raw_pressed = ((raw_state & bit) != 0) ? 1 : 0;
        uint8_t stable_pressed = ((*stable_state & bit) != 0) ? 1 : 0;
        
        if (raw_pressed != stable_pressed) {
            // State changing, increment counter
            if (debounce_count[i] < MATRIX_DEBOUNCE_SCANS) {
                debounce_count[i]++;
            }
            
            // If threshold reached, update stable state
            if (debounce_count[i] >= MATRIX_DEBOUNCE_SCANS) {
                if (raw_pressed) {
                    *stable_state |= bit;   // Set bit
                } else {
                    *stable_state &= ~bit;  // Clear bit
                }
                
                // Send MIDI event
                Matrix_SendMidiKeyEvent(i, raw_pressed);
                debounce_count[i] = 0;
            }
        } else {
            // State stable, reset counter
            debounce_count[i] = 0;
        }
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  // MX_USB_PCD_Init(); // TinyUSB initializes the USB peripheral
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* Configure USB GPIOs (PA11=DM, PA12=DP) for USB_DRD_FS */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  
  GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_USB;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Configure USB clock using HSI48 */
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* Enable the USB voltage detector (USB33DEN) so the 3.3 V supply can be monitored */
  SET_BIT(PWR->USBSCR, PWR_USBSCR_USB33DEN);

  /* Enable VDDUSB power supply */
  HAL_PWREx_EnableVddUSB();

  /* Wait until VddUSB is ready before touching the USB peripheral */
  while (!(PWR->VMSR & PWR_VMSR_USB33RDY)) {}

  /* Enable USB DRD FS clock */
  __HAL_RCC_USB_CLK_ENABLE();

  /* Enable CRS (Clock Recovery System) to keep HSI48 accurate for USB */
  __HAL_RCC_CRS_CLK_ENABLE();
  RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};
  RCC_CRSInitStruct.Prescaler             = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source                = RCC_CRS_SYNC_SOURCE_USB;
  RCC_CRSInitStruct.Polarity              = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue           = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000, 1000);
  RCC_CRSInitStruct.ErrorLimitValue       = 34;
  RCC_CRSInitStruct.HSI48CalibrationValue = 32;
  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);

  /* Configure USB DRD FS interrupt priority BEFORE enabling it */
  HAL_NVIC_SetPriority(USB_DRD_FS_IRQn, 5, 0);

  /* Initialize TinyUSB (handles USB peripheral init + enables NVIC interrupt internally) */
  tusb_init();

  /* Enable the NVIC interrupt AFTER TinyUSB has fully initialised the peripheral */
  HAL_NVIC_EnableIRQ(USB_DRD_FS_IRQn);

  /* Wait for USB enumeration */
  HAL_Delay(100);

  /* Initialize MCP23S17 I/O Expander (auto-probe CS pin) */
  MCP_Init();

  /* USER CODE END 2 */

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  // Matrix state variables
  uint16_t raw_keys = 0;
  uint16_t stable_keys = 0;
  uint8_t debounce_count[MATRIX_KEYS] = {0};
  uint8_t mcp_ready = MCP_IsReady();
  
  uint32_t last_scan_time = 0;
  uint32_t last_mcp_retry_time = 0;
  
#if MIDI_DIAGNOSTICS
  uint32_t last_diagnostic_time = 0;
  uint8_t heartbeat = 0;
#endif
  
  while (1)
  {
    tud_task();  // USB task - MUST be called every loop
    uint32_t now = HAL_GetTick();
    
    // Retry MCP detection if not ready
    if (!mcp_ready) {
      if ((now - last_mcp_retry_time) >= 1000) {
        last_mcp_retry_time = now;
        MCP_Init();
        mcp_ready = MCP_IsReady();
      }
      continue;
    }
    
#if MIDI_DIAGNOSTICS
    // Send diagnostic MIDI CC messages (every 500ms)
    if (tud_mounted() && ((now - last_diagnostic_time) >= 500)) {
      last_diagnostic_time = now;
      heartbeat = !heartbeat;
      
      uint8_t cc_msg[3];
      // CC100: Heartbeat (toggles 0/127)
      cc_msg[0] = 0xB0; cc_msg[1] = 100; cc_msg[2] = heartbeat ? 127 : 0;
      tud_midi_stream_write(0, cc_msg, 3);
      
      // CC101: MCP ready status
      cc_msg[1] = 101; cc_msg[2] = mcp_ready ? 127 : 0;
      tud_midi_stream_write(0, cc_msg, 3);
      
      // CC102: IODIRB register value
      cc_msg[1] = 102; cc_msg[2] = MCP_Read(MCP_IODIRB) & 0x7F;
      tud_midi_stream_write(0, cc_msg, 3);
      
      // CC103: GPIOB raw value
      cc_msg[1] = 103; cc_msg[2] = MCP_Read(MCP_GPIOB) & 0x7F;
      tud_midi_stream_write(0, cc_msg, 3);
    }
#endif
    
    // Matrix scanning (every 2ms)
    if (tud_mounted() && ((now - last_scan_time) >= MATRIX_SCAN_MS)) {
      last_scan_time = now;
      raw_keys = Matrix_ScanRaw();
      Matrix_UpdateDebounce(raw_keys, &stable_keys, debounce_count);
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_CSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV2;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.CSIState = RCC_CSI_ON;
  RCC_OscInitStruct.CSICalibrationValue = RCC_CSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_CSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 129;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the programming delay
  */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_0);
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x7;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi1.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi1.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */
  // Niet nodig - TinyUSB regelt USB initialisatie zelf
  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_DRD_FS.Instance = USB_DRD_FS;
  hpcd_USB_DRD_FS.Init.dev_endpoints = 8;
  hpcd_USB_DRD_FS.Init.speed = USBD_FS_SPEED;
  hpcd_USB_DRD_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_DRD_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.bulk_doublebuffer_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.iso_singlebuffer_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_DRD_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); // Default CS HIGH

  /*Configure GPIO pin : PA4 (CS Alternative) */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 (CS Primary / D10) */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Set as Output!
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC4 (Extra Input) */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Implement __io_putchar for printf support (uses ITM for debugging)
int __io_putchar(int ch)
{
  ITM_SendChar(ch);
  return ch;
}

// Voor GCC _write (nodig voor printf in sommige toolchains zoals STM32CubeIDE)
#ifdef __GNUC__
int _write(int file, char *ptr, int len)
{
  int DataIdx;
  for (DataIdx = 0; DataIdx < len; DataIdx++)
  {
    __io_putchar(*ptr++);
  }
  return len;
}
#endif

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
