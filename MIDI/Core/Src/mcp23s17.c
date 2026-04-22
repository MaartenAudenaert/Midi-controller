/**
  ******************************************************************************
  * @file           : mcp23s17.c
  * @brief          : MCP23S17 SPI I/O Expander Driver with Multi-CS Probe
  ******************************************************************************
  */

#include "mcp23s17.h"

// External SPI handle (defined in main.c)
extern SPI_HandleTypeDef hspi1;

// CS Pin structure
typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} mcp_cs_t;

// Active CS pin (detected during probe)
static mcp_cs_t s_active_cs = {NULL, 0};
static uint8_t s_mcp_ready = 0;

/**
  * @brief  Write to MCP23S17 register using specific CS pin
  * @param  port: GPIO port for CS
  * @param  pin: GPIO pin for CS
  * @param  reg: Register address
  * @param  value: Value to write
  * @retval HAL status
  */
static HAL_StatusTypeDef mcp_write_cs(GPIO_TypeDef *port, uint16_t pin, uint8_t reg, uint8_t value)
{
    uint8_t data[3];
    data[0] = MCP_OPCODE_WRITE;
    data[1] = reg;
    data[2] = value;
    
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);  // CS LOW
    HAL_StatusTypeDef st = HAL_SPI_Transmit(&hspi1, data, 3, 10);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);    // CS HIGH
    return st;
}

/**
  * @brief  Read from MCP23S17 register using specific CS pin
  * @param  port: GPIO port for CS
  * @param  pin: GPIO pin for CS
  * @param  reg: Register address
  * @param  value: Pointer to store read value
  * @retval HAL status
  */
static HAL_StatusTypeDef mcp_read_cs(GPIO_TypeDef *port, uint16_t pin, uint8_t reg, uint8_t *value)
{
    uint8_t tx_data[3] = {MCP_OPCODE_READ, reg, 0x00};
    uint8_t rx_data[3];
    
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);  // CS LOW
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 3, 10);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);    // CS HIGH
    
    if (st == HAL_OK) {
        *value = rx_data[2];
    }
    return st;
}

/**
  * @brief  Probe MCP23S17 on specific CS pin
  * @param  port: GPIO port to test
  * @param  pin: GPIO pin to test
  * @retval 1 if MCP found, 0 otherwise
  */
static uint8_t mcp_probe_cs(GPIO_TypeDef *port, uint16_t pin)
{
    uint8_t read_back = 0;
    
    // Test pattern 1: 0xA5
    if (mcp_write_cs(port, pin, MCP_IODIRA, 0xA5) != HAL_OK)
        return 0;
    if ((mcp_read_cs(port, pin, MCP_IODIRA, &read_back) != HAL_OK) || 
        (read_back != 0xA5))
        return 0;
    
    // Test pattern 2: 0x5A
    if (mcp_write_cs(port, pin, MCP_IODIRA, 0x5A) != HAL_OK)
        return 0;
    if ((mcp_read_cs(port, pin, MCP_IODIRA, &read_back) != HAL_OK) || 
        (read_back != 0x5A))
        return 0;
    
    return 1;  // Success
}

/**
  * @brief  Initialize MCP23S17 with auto-probe for CS pin
  * @retval None
  */
void MCP_Init(void)
{
    // Multi-CS pin candidates for flexible wiring
    const mcp_cs_t candidates[] = {
        {GPIOC, GPIO_PIN_9},  // Arduino D10 (most common)
        {GPIOA, GPIO_PIN_4},  // Alternative
        {GPIOB, GPIO_PIN_6}   // Alternative
    };
    
    s_mcp_ready = 0;
    
    // Probe all candidates
    for (uint8_t i = 0; i < 3; i++) {
        if (mcp_probe_cs(candidates[i].port, candidates[i].pin)) {
            s_active_cs = candidates[i];
            s_mcp_ready = 1;
            break;
        }
    }
    
    if (!s_mcp_ready) {
        return;  // No MCP23S17 found
    }
    
    // Configure MCP23S17 for 4x4 matrix
    // 1. GPA0-3 as outputs (columns), GPA4-7 as inputs
    MCP_Write(MCP_IODIRA, 0xF0);
    
    // 2. GPB0-3 as inputs (rows), GPB4-7 don't care
    MCP_Write(MCP_IODIRB, 0x0F);
    
    // 3. Enable pull-ups on row inputs
    MCP_Write(MCP_GPPUB, 0x0F);
    
    // 4. All columns HIGH (idle state)
    MCP_Write(MCP_GPIOA, 0x0F);
}

/**
  * @brief  Write to MCP23S17 register using detected CS pin
  * @param  reg: Register address
  * @param  value: Value to write
  * @retval None
  */
void MCP_Write(uint8_t reg, uint8_t value)
{
    if (!s_mcp_ready) return;
    mcp_write_cs(s_active_cs.port, s_active_cs.pin, reg, value);
}

/**
  * @brief  Read from MCP23S17 register using detected CS pin
  * @param  reg: Register address
  * @retval Read value (0 if error)
  */
uint8_t MCP_Read(uint8_t reg)
{
    if (!s_mcp_ready) return 0;
    
    uint8_t value = 0;
    mcp_read_cs(s_active_cs.port, s_active_cs.pin, reg, &value);
    return value;
}

/**
  * @brief  Check if MCP23S17 is ready
  * @retval 1 if ready, 0 otherwise
  */
uint8_t MCP_IsReady(void)
{
    return s_mcp_ready;
}

/**
  * @brief  Test MCP23S17 link
  * @retval 1 if link OK, 0 otherwise
  */
uint8_t MCP_TestLink(void)
{
    if (!s_mcp_ready) return 0;
    
    uint8_t test_val = 0;
    MCP_Write(MCP_IODIRA, 0xAA);
    test_val = MCP_Read(MCP_IODIRA);
    
    return (test_val == 0xAA) ? 1 : 0;
}
