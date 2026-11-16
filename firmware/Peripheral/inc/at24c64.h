#ifndef __AT24C64_H
#define __AT24C64_H

#include "ch32v20x.h"

/* GPIO Configuration --------------------------------------------------------*/
#define EE_GPIO_PORT    GPIOA
#define EE_SDA_PIN      GPIO_Pin_9   // PA9
#define EE_SCL_PIN      GPIO_Pin_10  // PA10

/* AT24C64 Parameters --------------------------------------------------------*/
#define EE_DEV_ADDR     0xA0         // Base 8-bit Address (A0/A1/A2 = GND)
#define EE_PAGE_SIZE    32           // AT24C64 page size in bytes
#define EE_CAPACITY     8192         // Total capacity in bytes (64Kbits)

/* Function Prototypes -------------------------------------------------------*/
void AT24C64_Init(void);
uint8_t AT24C64_WriteByte(uint16_t mem_addr, uint8_t data);
uint8_t AT24C64_ReadByte(uint16_t mem_addr, uint8_t *pData);
uint8_t AT24C64_WriteBuffer(uint16_t mem_addr, uint8_t *pBuffer, uint16_t numToWrite);
uint8_t AT24C64_ReadBuffer(uint16_t mem_addr, uint8_t *pBuffer, uint16_t numToRead);
uint8_t AT24C64_WaitStandbyState(void);

uint64_t Get_RV_Cycles(void);

#endif /* __AT24C64_H */