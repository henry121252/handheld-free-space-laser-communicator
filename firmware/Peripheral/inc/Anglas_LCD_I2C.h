/*
 * Anglas_LCD_I2C.h
 *
 * Hyper-Optimized Bit-Banged Software I2C Driver for CH32V203
 * Ultra-Low Latency Implementation for Optical Comm Devices
 */

#ifndef INC_ANGLAS_LCD_I2C_H_
#define INC_ANGLAS_LCD_I2C_H_

#include "ch32v20x.h"

/* Master Software I2C Pin Configuration -------------------------------------*/
#define LCD_I2C_PORT      GPIOA
#define SOFT_SCL_PIN      GPIO_Pin_10  // PA10
#define SOFT_SDA_PIN      GPIO_Pin_9   // PA9

/* Device Configuration ------------------------------------------------------*/
#define PCF8574_ADDRESS   0x27         // Default PCF8574 backpack address

#define PIN_RS            0x01
#define PIN_RW            0x02
#define PIN_EN            0x04
#define BACKLIGHT         0x08

/* HD44780 LCD Commands ------------------------------------------------------*/
#define DISPLAY_CLEAR     0x01 // 2ms hardware freeze
#define RETURN_HOME       0x02
#define ENTRY_MODE_SET    0x06
#define DISPLAY_ON        0x0C
#define MODE_4_BITS       0x28

/* Function Prototypes -------------------------------------------------------*/
void LCD_I2C_Init(void);
void LCD_I2C_ClearText(void);
void LCD_I2C_Set_Cursor_Blink(uint8_t row, uint8_t col, uint8_t blink);
void LCD_I2C_WriteText(uint8_t row, uint8_t col, const char* string);

/* Fast Streaming API (Use these to prevent MCU latency) */
void LCD_I2C_WriteRow(uint8_t row, const char* string);

void LCD_I2C_Shift_Right(void);
void LCD_I2C_Shift_Left(void);
void LCD_I2C_CGRAM_CreateChar(uint8_t pos, const uint8_t *msg);
void LCD_I2C_CGRAM_WriteChar(uint8_t row, uint8_t col, uint8_t pos);

#endif /* INC_ANGLAS_LCD_I2C_H_ */