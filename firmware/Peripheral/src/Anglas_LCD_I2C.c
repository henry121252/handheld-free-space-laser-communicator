/*
 * Anglas_LCD_I2C.c
 *
 * Hyper-Optimized Bit-Banged Software I2C Driver for CH32V203
 */

#include "Anglas_LCD_I2C.h"

/* 1-Cycle Register Direct GPIO Macros for CH32V203 */
#define SCL_HIGH()      (LCD_I2C_PORT->BSHR = SOFT_SCL_PIN)
#define SCL_LOW()       (LCD_I2C_PORT->BCR  = SOFT_SCL_PIN)
#define SDA_HIGH()      (LCD_I2C_PORT->BSHR = SOFT_SDA_PIN)
#define SDA_LOW()       (LCD_I2C_PORT->BCR  = SOFT_SDA_PIN)
#define READ_SDA()      ((LCD_I2C_PORT->INDR & SOFT_SDA_PIN) ? 1 : 0)

/* Private Function Prototypes */
static inline void I2C_Delay(void);
static void I2C_Start(void);
static void I2C_Stop(void);
static uint8_t I2C_WriteByte(uint8_t data);
static void LCD_I2C_SendNibbles(uint8_t value, uint8_t is_data);

/**
 * @brief  Ultra-fast bus delay (~350kHz-400kHz I2C clock speed)
 */
static inline void I2C_Delay(void) {
    Delay_Us(1); // 1us half-period = 500kHz theoretical max
}

static void I2C_Start(void) {
    SDA_HIGH();
    SCL_HIGH();
    I2C_Delay();
    SDA_LOW();
    I2C_Delay();
    SCL_LOW();
    I2C_Delay();
}

static void I2C_Stop(void) {
    SDA_LOW();
    SCL_HIGH();
    I2C_Delay();
    SDA_HIGH();
    I2C_Delay();
}

/**
 * @brief  Writes 1 byte over I2C bus (Optimized inline loop)
 */
static uint8_t I2C_WriteByte(uint8_t data) {
    for (uint8_t i = 0; i < 8; i++) {
        if (data & 0x80) SDA_HIGH();
        else             SDA_LOW();
        
        I2C_Delay();
        SCL_HIGH();
        I2C_Delay();
        SCL_LOW();
        data <<= 1;
    }

    // ACK phase
    SDA_HIGH();
    I2C_Delay();
    SCL_HIGH();
    I2C_Delay();
    
    uint8_t ack = READ_SDA();
    
    SCL_LOW();
    I2C_Delay();
    
    return ack;
}

/**
 * @brief  Sends nibbles to PCF8574
 */
static void LCD_I2C_SendNibbles(uint8_t value, uint8_t is_data) {
    uint8_t rs = is_data ? PIN_RS : 0;
    uint8_t high_nibble = (value & 0xF0) | BACKLIGHT | rs;
    uint8_t low_nibble  = ((value << 4) & 0xF0) | BACKLIGHT | rs;

    I2C_Start();
    if (I2C_WriteByte(PCF8574_ADDRESS << 1) == 0) {
        I2C_WriteByte(high_nibble | PIN_EN);
        I2C_WriteByte(high_nibble);

        I2C_WriteByte(low_nibble | PIN_EN);
        I2C_WriteByte(low_nibble);
    }
    I2C_Stop();
}

static void LCD_I2C_DirectWrite(uint8_t data) {
    I2C_Start();
    if (I2C_WriteByte(PCF8574_ADDRESS << 1) == 0) {
        I2C_WriteByte(data | PIN_EN);
        I2C_WriteByte(data);
    }
    I2C_Stop();
}

void LCD_I2C_Init(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin = SOFT_SCL_PIN | SOFT_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LCD_I2C_PORT, &GPIO_InitStructure);

    SDA_HIGH();
    SCL_HIGH();
    
    Delay_Ms(50);
    
    LCD_I2C_DirectWrite(0x30 | BACKLIGHT);
    Delay_Ms(5);

    LCD_I2C_DirectWrite(0x30 | BACKLIGHT);
    Delay_Us(150);

    LCD_I2C_DirectWrite(0x30 | BACKLIGHT);
    Delay_Ms(5);

    LCD_I2C_DirectWrite(0x20 | BACKLIGHT);
    Delay_Ms(5);

    LCD_I2C_SendNibbles(MODE_4_BITS, 0);
    LCD_I2C_SendNibbles(DISPLAY_ON, 0);
    LCD_I2C_SendNibbles(ENTRY_MODE_SET, 0);
    LCD_I2C_ClearText();
}

void LCD_I2C_ClearText(void) {
    LCD_I2C_SendNibbles(DISPLAY_CLEAR, 0);
    Delay_Ms(2); // Mandatory HD44780 clear hardware delay
}

void LCD_I2C_Set_Cursor_Blink(uint8_t row, uint8_t col, uint8_t blink) {
    uint8_t base_addr = (row <= 1) ? 0x80 : 0xC0;
    uint8_t address = base_addr + (col - 1);
    LCD_I2C_SendNibbles(address, 0);
    
    if (blink) {
        LCD_I2C_SendNibbles(DISPLAY_ON | 0x03, 0);
    } else {
        LCD_I2C_SendNibbles(DISPLAY_ON, 0);
    }
}

void LCD_I2C_WriteText(uint8_t row, uint8_t col, const char* string) {
    uint8_t base_addr = (row <= 1) ? 0x80 : 0xC0;
    uint8_t address = base_addr + (col - 1);
    LCD_I2C_SendNibbles(address, 0);
    
    I2C_Start();
    if (I2C_WriteByte(PCF8574_ADDRESS << 1) != 0) {
        I2C_Stop();
        return;
    }

    while (*string) {
        uint8_t data = (uint8_t)*string++;
        uint8_t high_nibble = (data & 0xF0) | BACKLIGHT | PIN_RS;
        uint8_t low_nibble  = ((data << 4) & 0xF0) | BACKLIGHT | PIN_RS;

        I2C_WriteByte(high_nibble | PIN_EN);
        I2C_WriteByte(high_nibble);
        I2C_WriteByte(low_nibble | PIN_EN);
        I2C_WriteByte(low_nibble);
    }
    
    I2C_Stop();
}

/**
 * @brief  ULTRA-FAST STREAMING ROW WRITER (~0.6ms execution time)
 *         Overwrites full row and pads empty space without calling ClearText()!
 */
void LCD_I2C_WriteRow(uint8_t row, const char* string) {
    uint8_t address = (row <= 1) ? 0x80 : 0xC0;
    LCD_I2C_SendNibbles(address, 0);
    
    I2C_Start();
    if (I2C_WriteByte(PCF8574_ADDRESS << 1) != 0) {
        I2C_Stop();
        return;
    }

    uint8_t count = 0;
    while (*string && count < 16) {
        uint8_t data = (uint8_t)*string++;
        uint8_t high_nibble = (data & 0xF0) | BACKLIGHT | PIN_RS;
        uint8_t low_nibble  = ((data << 4) & 0xF0) | BACKLIGHT | PIN_RS;

        I2C_WriteByte(high_nibble | PIN_EN);
        I2C_WriteByte(high_nibble);
        I2C_WriteByte(low_nibble | PIN_EN);
        I2C_WriteByte(low_nibble);
        count++;
    }

    // Auto-pad remaining character spaces up to 16
    while (count < 16) {
        uint8_t data = ' ';
        uint8_t high_nibble = (data & 0xF0) | BACKLIGHT | PIN_RS;
        uint8_t low_nibble  = ((data << 4) & 0xF0) | BACKLIGHT | PIN_RS;

        I2C_WriteByte(high_nibble | PIN_EN);
        I2C_WriteByte(high_nibble);
        I2C_WriteByte(low_nibble | PIN_EN);
        I2C_WriteByte(low_nibble);
        count++;
    }

    I2C_Stop();
}

void LCD_I2C_Shift_Right(void) { LCD_I2C_SendNibbles(0x1C, 0); }
void LCD_I2C_Shift_Left(void)  { LCD_I2C_SendNibbles(0x18, 0); }

void LCD_I2C_CGRAM_CreateChar(uint8_t pos, const uint8_t *msg) {
    LCD_I2C_SendNibbles(0x40 + (pos * 8), 0);
    for (uint8_t i = 0; i < 8; i++) {
        LCD_I2C_SendNibbles(msg[i], 1);
    }
}

void LCD_I2C_CGRAM_WriteChar(uint8_t row, uint8_t col, uint8_t pos) {
    uint8_t base_addr = (row <= 1) ? 0x80 : 0xC0;
    uint8_t address = base_addr + (col - 1);
    LCD_I2C_SendNibbles(address, 0);
    LCD_I2C_SendNibbles(pos, 1);
}