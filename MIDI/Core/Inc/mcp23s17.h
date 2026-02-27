/**
  ******************************************************************************
  * @file           : mcp23s17.h
  * @brief          : MCP23S17 SPI I/O Expander Driver with Multi-CS Probe
  ******************************************************************************
  */

#ifndef __MCP23S17_H
#define __MCP23S17_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

// MCP23S17 Registers (IOCON.BANK = 0)
#define MCP_IODIRA   0x00  // I/O Direction Register A
#define MCP_IODIRB   0x01  // I/O Direction Register B
#define MCP_GPPUA    0x0C  // Pull-up Register A
#define MCP_GPPUB    0x0D  // Pull-up Register B
#define MCP_GPIOA    0x12  // GPIO Port A
#define MCP_GPIOB    0x13  // GPIO Port B

// Opcode (Address pins A2=0, A1=0, A0=0)
#define MCP_OPCODE_WRITE 0x40
#define MCP_OPCODE_READ  0x41

// Function Prototypes
void MCP_Init(void);
void MCP_Write(uint8_t reg, uint8_t value);
uint8_t MCP_Read(uint8_t reg);
uint8_t MCP_IsReady(void);
uint8_t MCP_TestLink(void);

#ifdef __cplusplus
}
#endif

#endif /* __MCP23S17_H */
