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
#define HYSTERESIS             2
#define NUM_POTS               8
#define MIDI_CC_START          16
#define POT_FILTER_SAMPLES     4U
#define POT_MIN_INTERVAL_MS    20U
#define ADC_MAX_VALUE          4095U
#define MIDI_MAX_VALUE         127U
#define MIDI_DEADBAND_LOW      2U
#define MIDI_DEADBAND_HIGH     (MIDI_MAX_VALUE - MIDI_DEADBAND_LOW)
#define POTS_INVERT_DIRECTION  1U
#define SK6812_LED_COUNT       16U
#define SK6812_BITS_PER_LED    24U
#define SK6812_RESET_SLOTS     80U
#define SK6812_BUFFER_SIZE     (SK6812_LED_COUNT * SK6812_BITS_PER_LED + SK6812_RESET_SLOTS)
#define SK6812_T0H_PERCENT     24U
#define SK6812_T1H_PERCENT     48U
#define LED_PRESSED_R          0U
#define LED_PRESSED_G          64U
#define LED_PRESSED_B          0U
#define LED_RELEASED_R         0U
#define LED_RELEASED_G         0U
#define LED_RELEASED_B         0U
#define LED_STANDBY_ENABLE     0U
#define LED_STANDBY_R          0U
#define LED_STANDBY_G          0U
#define LED_STANDBY_B          16U
#define LED_STANDBY_TIMEOUT_MS 2000U
#define SK6812_STARTUP_BLINK_ENABLE 1U
#define SK6812_STARTUP_BLINK_MS     120U
#define D8_GPIO_TEST_ENABLE         0U
#define D8_BLINK_MS                 200U
#define SK6812_PC6_TEST_ENABLE      1U
#define SK6812_PC6_BLINK_MS         1000U
#define SK6812_PC6_TOGGLE_KEY_INDEX 0U
#define SK6812_PC6_COLOR_COUNT      4U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;
ADC_HandleTypeDef hadc1;
DMA_NodeTypeDef Node_GPDMA1_Channel0;
DMA_QListTypeDef List_GPDMA1_Channel0;
DMA_HandleTypeDef handle_GPDMA1_Channel0;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
DMA_HandleTypeDef handle_GPDMA1_Channel1;

PCD_HandleTypeDef hpcd_USB_DRD_FS;

/* USER CODE BEGIN PV */
volatile uint16_t adc_buffer[NUM_POTS] = {0};    // Filled continuously by ADC DMA
volatile uint16_t adc_snapshot[NUM_POTS] = {0};  // Stable copy of the latest full scan
volatile uint8_t adc_frame_ready = 0;
uint8_t last_midi_value[NUM_POTS] = {0};
uint16_t pot_history[NUM_POTS][POT_FILTER_SAMPLES] = {{0}};
uint32_t pot_history_sum[NUM_POTS] = {0};
uint8_t pot_history_index = 0;
uint8_t pot_filter_primed = 0;
uint32_t last_pot_send_ms[NUM_POTS] = {0};
static uint8_t sk6812_colors[SK6812_LED_COUNT][3] = {{0}};
static uint16_t sk6812_pwm_buffer[SK6812_BUFFER_SIZE] = {0};
static volatile uint8_t sk6812_dma_busy = 0;
static volatile uint8_t sk6812_needs_update = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_GPDMA1_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM6_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
static void ADC_Start(void);
static void Process_Potentiometers(void);
static uint8_t Pot_ConvertToMidi(uint16_t raw_value);
static void SK6812_Init(void);
static void SK6812_SetColor(uint8_t led_index, uint8_t r, uint8_t g, uint8_t b);
static void SK6812_SetAll(uint8_t r, uint8_t g, uint8_t b);
static void SK6812_ApplyMatrixState(uint16_t stable_state);
static void SK6812_Show(void);
static void SK6812_BlinkStartup(void);

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
                if (raw_pressed) {
                  SK6812_SetColor(i, LED_PRESSED_R, LED_PRESSED_G, LED_PRESSED_B);
                } else {
                  SK6812_SetColor(i, LED_RELEASED_R, LED_RELEASED_G, LED_RELEASED_B);
                }
                debounce_count[i] = 0;
            }
        } else {
            // State stable, reset counter
            debounce_count[i] = 0;
        }
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC1) {
        return;
    }

    // Keep channel values from the same scan together before the DMA writes again.
    for (uint8_t i = 0; i < NUM_POTS; i++) {
        adc_snapshot[i] = adc_buffer[i];
    }

    adc_frame_ready = 1;
}

static uint8_t Pot_ConvertToMidi(uint16_t raw_value)
{
    uint8_t midi_value = (uint8_t)(((uint32_t)raw_value * MIDI_MAX_VALUE + (ADC_MAX_VALUE / 2U)) / ADC_MAX_VALUE);

    if (midi_value <= MIDI_DEADBAND_LOW) {
        midi_value = 0;
    } else if (midi_value >= MIDI_DEADBAND_HIGH) {
        midi_value = MIDI_MAX_VALUE;
    }

#if POTS_INVERT_DIRECTION
    midi_value = (uint8_t)(MIDI_MAX_VALUE - midi_value);
#endif

    return midi_value;
}

static void ADC_Start(void)
{
    if (HAL_TIM_Base_Start(&htim6) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, NUM_POTS) != HAL_OK) {
        Error_Handler();
    }
}

static void Process_Potentiometers(void)
{
    if (!adc_frame_ready) {
        return;
    }

    adc_frame_ready = 0;
  uint32_t now_ms = HAL_GetTick();

    if (!pot_filter_primed) {
        for (uint8_t i = 0; i < NUM_POTS; i++) {
            uint16_t sample = adc_snapshot[i] & ADC_MAX_VALUE;

            for (uint8_t slot = 0; slot < POT_FILTER_SAMPLES; slot++) {
                pot_history[i][slot] = sample;
            }

            pot_history_sum[i] = (uint32_t)sample * POT_FILTER_SAMPLES;
            last_midi_value[i] = Pot_ConvertToMidi(sample);
            last_pot_send_ms[i] = now_ms;
        }

        pot_filter_primed = 1;
        return;
    }

    for (uint8_t i = 0; i < NUM_POTS; i++) {
        uint16_t sample = adc_snapshot[i] & ADC_MAX_VALUE;
        pot_history_sum[i] -= pot_history[i][pot_history_index];
        pot_history[i][pot_history_index] = sample;
        pot_history_sum[i] += sample;

        uint16_t averaged_sample = (uint16_t)(pot_history_sum[i] / POT_FILTER_SAMPLES);
        uint8_t new_value = Pot_ConvertToMidi(averaged_sample);
        int16_t diff = (int16_t)new_value - (int16_t)last_midi_value[i];
        if (diff < 0) {
            diff = -diff;
        }

        if ((diff >= HYSTERESIS) && ((now_ms - last_pot_send_ms[i]) >= POT_MIN_INTERVAL_MS)) {
            if (tud_mounted()) {
                uint8_t msg[3] = {0xB0, (uint8_t)(MIDI_CC_START + i), new_value};
                tud_midi_stream_write(0, msg, 3);
            }

            last_midi_value[i] = new_value;
          last_pot_send_ms[i] = now_ms;
        }
    }

    pot_history_index++;
    if (pot_history_index >= POT_FILTER_SAMPLES) {
        pot_history_index = 0;
    }
}

  static void SK6812_Init(void)
  {
    for (uint8_t i = 0; i < SK6812_LED_COUNT; i++) {
  #if LED_STANDBY_ENABLE
      sk6812_colors[i][0] = LED_STANDBY_G;
      sk6812_colors[i][1] = LED_STANDBY_R;
      sk6812_colors[i][2] = LED_STANDBY_B;
  #else
      sk6812_colors[i][0] = LED_RELEASED_G;
      sk6812_colors[i][1] = LED_RELEASED_R;
      sk6812_colors[i][2] = LED_RELEASED_B;
  #endif
    }
    sk6812_needs_update = 1;
  }

  static void SK6812_BuildBuffer(void)
  {
    uint16_t period = (uint16_t)(htim3.Init.Period + 1U);
    uint16_t t0h = (uint16_t)((period * SK6812_T0H_PERCENT) / 100U);
    uint16_t t1h = (uint16_t)((period * SK6812_T1H_PERCENT) / 100U);
    uint32_t index = 0;

    for (uint8_t led = 0; led < SK6812_LED_COUNT; led++) {
      for (uint8_t byte_index = 0; byte_index < 3U; byte_index++) {
        uint8_t value = sk6812_colors[led][byte_index];
        for (int8_t bit = 7; bit >= 0; bit--) {
          sk6812_pwm_buffer[index++] = (value & (1U << bit)) ? t1h : t0h;
        }
      }
    }

    for (uint32_t i = 0; i < SK6812_RESET_SLOTS; i++) {
      sk6812_pwm_buffer[index++] = 0;
    }
  }

  static void SK6812_SetColor(uint8_t led_index, uint8_t r, uint8_t g, uint8_t b)
  {
    if (led_index >= SK6812_LED_COUNT) {
      return;
    }

    sk6812_colors[led_index][0] = g;
    sk6812_colors[led_index][1] = r;
    sk6812_colors[led_index][2] = b;
    sk6812_needs_update = 1;
  }

  static void SK6812_SetAll(uint8_t r, uint8_t g, uint8_t b)
  {
    for (uint8_t i = 0; i < SK6812_LED_COUNT; i++) {
      sk6812_colors[i][0] = g;
      sk6812_colors[i][1] = r;
      sk6812_colors[i][2] = b;
    }
    sk6812_needs_update = 1;
  }

  static void SK6812_ApplyMatrixState(uint16_t stable_state)
  {
    for (uint8_t i = 0; i < SK6812_LED_COUNT; i++) {
      if (stable_state & (1U << i)) {
        SK6812_SetColor(i, LED_PRESSED_R, LED_PRESSED_G, LED_PRESSED_B);
      } else {
        SK6812_SetColor(i, LED_RELEASED_R, LED_RELEASED_G, LED_RELEASED_B);
      }
    }
  }

  static void SK6812_BlinkStartup(void)
  {
#if SK6812_STARTUP_BLINK_ENABLE
    SK6812_SetAll(0U, 48U, 0U);
    SK6812_Show();
    while (sk6812_dma_busy) {}
    HAL_Delay(SK6812_STARTUP_BLINK_MS);

    SK6812_SetAll(0U, 0U, 0U);
    SK6812_Show();
    while (sk6812_dma_busy) {}
#endif
  }

  static void SK6812_Show(void)
  {
    if (sk6812_dma_busy) {
      return;
    }

    SK6812_BuildBuffer();
    sk6812_dma_busy = 1;

    if (HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_1, (uint32_t *)sk6812_pwm_buffer, SK6812_BUFFER_SIZE) != HAL_OK) {
      sk6812_dma_busy = 0;
      Error_Handler();
    }
  }

  void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
  {
    if (htim->Instance != TIM3) {
      return;
    }

    HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, 0);
    sk6812_dma_busy = 0;
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
  MX_GPDMA1_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  MX_USB_PCD_Init();
  MX_TIM3_Init();
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
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }
  ADC_Start();
  SK6812_Init();
  SK6812_BlinkStartup();

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
#if LED_STANDBY_ENABLE
  uint32_t last_key_activity_ms = 0;
  uint8_t standby_active = 0;
#endif
#if D8_GPIO_TEST_ENABLE
  uint32_t last_d8_toggle_ms = 0;
#endif
#if SK6812_PC6_TEST_ENABLE
  uint32_t last_pc6_toggle_ms = 0;
  uint8_t pc6_color_step = 0;
  uint8_t pc6_test_enabled = 0;
  uint8_t pc6_toggle_stable_pressed = 0;
  uint8_t pc6_toggle_debounce_count = 0;
#endif
  
#if MIDI_DIAGNOSTICS
  uint32_t last_diagnostic_time = 0;
  uint8_t heartbeat = 0;
#endif
  
#if LED_STANDBY_ENABLE
  last_key_activity_ms = HAL_GetTick();
#endif

  while (1)
  {
    tud_task();  // USB task - MUST be called every loop
    Process_Potentiometers();
    uint32_t now = HAL_GetTick();
#if D8_GPIO_TEST_ENABLE
    if ((now - last_d8_toggle_ms) >= D8_BLINK_MS) {
      last_d8_toggle_ms = now;
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_9);
    }
#endif
#if SK6812_PC6_TEST_ENABLE
    static const uint8_t pc6_colors[SK6812_PC6_COLOR_COUNT][3] = {
      {48U, 0U, 0U},    // red
      {0U, 48U, 0U},    // green
      {0U, 0U, 48U},    // blue
      {32U, 32U, 32U}   // white
    };
    uint16_t pc6_toggle_bit = (uint16_t)(1U << SK6812_PC6_TOGGLE_KEY_INDEX);

    if (!mcp_ready && ((now - last_mcp_retry_time) >= 1000U)) {
      last_mcp_retry_time = now;
      MCP_Init();
      mcp_ready = MCP_IsReady();
    }

    if (mcp_ready && ((now - last_scan_time) >= MATRIX_SCAN_MS)) {
      uint8_t raw_toggle_pressed;
      last_scan_time = now;
      raw_keys = Matrix_ScanRaw();
      raw_toggle_pressed = ((raw_keys & pc6_toggle_bit) != 0U) ? 1U : 0U;

      if (raw_toggle_pressed != pc6_toggle_stable_pressed) {
        if (pc6_toggle_debounce_count < MATRIX_DEBOUNCE_SCANS) {
          pc6_toggle_debounce_count++;
        }

        if (pc6_toggle_debounce_count >= MATRIX_DEBOUNCE_SCANS) {
          pc6_toggle_stable_pressed = raw_toggle_pressed;
          pc6_toggle_debounce_count = 0;

          if (pc6_toggle_stable_pressed) {
            pc6_test_enabled ^= 1U;
            last_pc6_toggle_ms = now;
            pc6_color_step = 0U;

            if (pc6_test_enabled) {
              SK6812_SetAll(pc6_colors[pc6_color_step][0], pc6_colors[pc6_color_step][1], pc6_colors[pc6_color_step][2]);
            } else {
              SK6812_SetAll(0U, 0U, 0U);
            }
          }
        }
      } else {
        pc6_toggle_debounce_count = 0;
      }
    }

    if (pc6_test_enabled && ((now - last_pc6_toggle_ms) >= SK6812_PC6_BLINK_MS)) {
      last_pc6_toggle_ms = now;
      pc6_color_step = (uint8_t)((pc6_color_step + 1U) % SK6812_PC6_COLOR_COUNT);
      SK6812_SetAll(pc6_colors[pc6_color_step][0], pc6_colors[pc6_color_step][1], pc6_colors[pc6_color_step][2]);
    }

    if (sk6812_needs_update && !sk6812_dma_busy) {
      sk6812_needs_update = 0;
      SK6812_Show();
    }
    continue;
#endif
    
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
      uint16_t prev_stable_keys = stable_keys;
      raw_keys = Matrix_ScanRaw();
      Matrix_UpdateDebounce(raw_keys, &stable_keys, debounce_count);
#if LED_STANDBY_ENABLE
      if (stable_keys != prev_stable_keys) {
        last_key_activity_ms = now;
        if (standby_active && stable_keys != 0) {
          standby_active = 0;
          SK6812_ApplyMatrixState(stable_keys);
        }
      }
#endif
    }

#if LED_STANDBY_ENABLE
    if (!standby_active && (stable_keys == 0) && ((now - last_key_activity_ms) >= LED_STANDBY_TIMEOUT_MS)) {
      standby_active = 1;
      SK6812_SetAll(LED_STANDBY_R, LED_STANDBY_G, LED_STANDBY_B);
    }
#endif

    if (sk6812_needs_update && !sk6812_dma_busy) {
      sk6812_needs_update = 0;
      SK6812_Show();
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV10;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 8;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.SamplingMode = ADC_SAMPLING_MODE_NORMAL;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief GPDMA1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPDMA1_Init(void)
{

  /* USER CODE BEGIN GPDMA1_Init 0 */

  /* USER CODE END GPDMA1_Init 0 */

  /* Peripheral clock enable */
  __HAL_RCC_GPDMA1_CLK_ENABLE();

  /* GPDMA1 interrupt Init */
    HAL_NVIC_SetPriority(GPDMA1_Channel0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel0_IRQn);
    HAL_NVIC_SetPriority(GPDMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel1_IRQn);

  /* USER CODE BEGIN GPDMA1_Init 1 */

  /* USER CODE END GPDMA1_Init 1 */
  /* USER CODE BEGIN GPDMA1_Init 2 */

  /* USER CODE END GPDMA1_Init 2 */

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
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x7;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
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
  // Ensure MCP23S17-compatible SPI settings even after CubeMX regeneration.
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = (uint32_t)((HAL_RCC_GetPCLK1Freq() / 800000U) - 1U);
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 249;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  // Keep MCP chip-select lines deasserted (active-low CS).
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);

#if D8_GPIO_TEST_ENABLE
  // D8 on NUCLEO-H533RE header: PA9 (GPIO test blink)
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
#endif

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
