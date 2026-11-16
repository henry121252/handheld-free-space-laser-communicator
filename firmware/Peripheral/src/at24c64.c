#include "at24c64.h"

// Pre-calculated cycle delay count for 400kHz Fast-Mode (~1.25us half-period)
static uint32_t g_i2c_delay_ticks = 0;

// ---------------------------------------------------------------------------
// Low-Level Bit-Bang Helpers
// ---------------------------------------------------------------------------

#define SCL_H() GPIO_WriteBit(EE_GPIO_PORT, EE_SCL_PIN, Bit_SET)
#define SCL_L() GPIO_WriteBit(EE_GPIO_PORT, EE_SCL_PIN, Bit_RESET)
#define SDA_H() GPIO_WriteBit(EE_GPIO_PORT, EE_SDA_PIN, Bit_SET)
#define SDA_L() GPIO_WriteBit(EE_GPIO_PORT, EE_SDA_PIN, Bit_RESET)
#define SDA_R() GPIO_ReadInputDataBit(EE_GPIO_PORT, EE_SDA_PIN)

/**
 * @brief High-efficiency delay using cycle counter without runtime division
 */
static inline void i2c_delay(void) {
    uint64_t start = Get_RV_Cycles();
    while ((Get_RV_Cycles() - start) < g_i2c_delay_ticks);
}

static void i2c_Start(void) {
    SDA_H();
    SCL_H();
    i2c_delay();
    SDA_L();
    i2c_delay();
    SCL_L();
    i2c_delay();
}

static void i2c_Stop(void) {
    SDA_L();
    SCL_H();
    i2c_delay();
    SDA_H();
    i2c_delay();
}

// Returns 0 for ACK, 1 for NACK
static uint8_t i2c_SendByte(uint8_t byte) {
    for (uint8_t i = 0; i < 8; i++) {
        if (byte & 0x80) SDA_H();
        else             SDA_L();
        
        i2c_delay();
        SCL_H();
        i2c_delay();
        SCL_L();
        byte <<= 1;
    }
    
    // Read ACK
    SDA_H(); 
    i2c_delay();
    SCL_H();
    i2c_delay();
    uint8_t ack = SDA_R();
    SCL_L();
    i2c_delay();
    
    return ack; // 0 = ACK, 1 = NACK
}

static uint8_t i2c_ReadByte(uint8_t ack) {
    uint8_t byte = 0;
    SDA_H();
    for (uint8_t i = 0; i < 8; i++) {
        byte <<= 1;
        i2c_delay();
        SCL_H();
        i2c_delay();
        if (SDA_R()) byte |= 1;
        SCL_L();
    }
    
    // Send ACK (0) or NACK (1)
    if (ack) SDA_L();
    else     SDA_H();
    
    i2c_delay();
    SCL_H();
    i2c_delay();
    SCL_L();
    SDA_H();
    i2c_delay();
    
    return byte;
}

// ---------------------------------------------------------------------------
// Public API Functions
// ---------------------------------------------------------------------------

void AT24C64_Init(void) {
    // Compute ticks once during Init (~1.25us for ~400kHz I2C speed)
    g_i2c_delay_ticks = (uint32_t)(1.25f * (SystemCoreClock / 1000000.0f));

    GPIO_InitTypeDef GPIO_InitStructure = {0};
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = EE_SCL_PIN | EE_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(EE_GPIO_PORT, &GPIO_InitStructure);

    SDA_H();
    SCL_H();
}

/**
 * @brief  ACK Polling function. Replaces static 5ms delay.
 *         Spins until EEPROM finishes internal write cycle.
 */
uint8_t AT24C64_WaitStandbyState(void) {
    uint32_t timeout = 1000; // Safety limit
    
    while (timeout--) {
        i2c_Start();
        if (i2c_SendByte(EE_DEV_ADDR) == 0) {
            // Received ACK: internal write finished!
            i2c_Stop();
            return 0;
        }
        i2c_Stop();
        Delay_Us(100);
    }
    return 1; // Timed out
}

uint8_t AT24C64_WriteByte(uint16_t mem_addr, uint8_t data) {
    i2c_Start();
    if (i2c_SendByte(EE_DEV_ADDR))             { i2c_Stop(); return 1; }
    if (i2c_SendByte((mem_addr >> 8) & 0xFF))  { i2c_Stop(); return 1; }
    if (i2c_SendByte(mem_addr & 0xFF))         { i2c_Stop(); return 1; }
    if (i2c_SendByte(data))                    { i2c_Stop(); return 1; }
    i2c_Stop();

    // Fast ACK Polling instead of Delay_Ms(5)
    return AT24C64_WaitStandbyState();
}

uint8_t AT24C64_ReadByte(uint16_t mem_addr, uint8_t *pData) {
    i2c_Start();
    if (i2c_SendByte(EE_DEV_ADDR))             { i2c_Stop(); return 1; }
    if (i2c_SendByte((mem_addr >> 8) & 0xFF))  { i2c_Stop(); return 1; }
    if (i2c_SendByte(mem_addr & 0xFF))         { i2c_Stop(); return 1; }

    i2c_Start();
    if (i2c_SendByte(EE_DEV_ADDR | 0x01))      { i2c_Stop(); return 1; }

    *pData = i2c_ReadByte(0); // NACK final byte
    i2c_Stop();

    return 0;
}

/**
 * @brief Low-level single-page write helper (max 32 bytes within 1 page boundary)
 */
static uint8_t AT24C64_WritePageInternal(uint16_t mem_addr, uint8_t* pBuffer, uint16_t numToWrite) {
    i2c_Start();
    if (i2c_SendByte(EE_DEV_ADDR))             { i2c_Stop(); return 1; }
    if (i2c_SendByte((mem_addr >> 8) & 0xFF))  { i2c_Stop(); return 1; }
    if (i2c_SendByte(mem_addr & 0xFF))         { i2c_Stop(); return 1; }

    for (uint16_t i = 0; i < numToWrite; i++) {
        if (i2c_SendByte(pBuffer[i]))          { i2c_Stop(); return 1; }
    }
    i2c_Stop();

    return AT24C64_WaitStandbyState();
}

/**
 * @brief High-level write function that handles arbitrary length & page boundary math
 */
uint8_t AT24C64_WriteBuffer(uint16_t mem_addr, uint8_t* pBuffer, uint16_t numToWrite) {
    uint16_t count = 0;
    uint16_t page_offset = mem_addr % EE_PAGE_SIZE;
    uint16_t bytes_left_in_page = EE_PAGE_SIZE - page_offset;

    if (page_offset != 0) {
        // Write unaligned portion first to reach the next page boundary
        count = (numToWrite < bytes_left_in_page) ? numToWrite : bytes_left_in_page;
        if (AT24C64_WritePageInternal(mem_addr, pBuffer, count)) return 1;
        
        mem_addr += count;
        pBuffer  += count;
        numToWrite -= count;
    }

    // Write full aligned 32-byte pages
    while (numToWrite > 0) {
        count = (numToWrite < EE_PAGE_SIZE) ? numToWrite : EE_PAGE_SIZE;
        if (AT24C64_WritePageInternal(mem_addr, pBuffer, count)) return 1;

        mem_addr += count;
        pBuffer  += count;
        numToWrite -= count;
    }

    return 0;
}

uint8_t AT24C64_ReadBuffer(uint16_t mem_addr, uint8_t* pBuffer, uint16_t numToRead) {
    if (numToRead == 0) return 0;

    i2c_Start();
    if (i2c_SendByte(EE_DEV_ADDR))             { i2c_Stop(); return 1; }
    if (i2c_SendByte((mem_addr >> 8) & 0xFF))  { i2c_Stop(); return 1; }
    if (i2c_SendByte(mem_addr & 0xFF))         { i2c_Stop(); return 1; }

    i2c_Start();
    if (i2c_SendByte(EE_DEV_ADDR | 0x01))      { i2c_Stop(); return 1; }

    for (uint16_t i = 0; i < numToRead - 1; i++) {
        pBuffer[i] = i2c_ReadByte(1); // ACK sequential bytes
    }
    pBuffer[numToRead - 1] = i2c_ReadByte(0); // NACK last byte

    i2c_Stop();
    return 0;
}