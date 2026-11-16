/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : Henry Kester
 * Version            : V8.7 (Configurable Bit Period & EEPROM Rate Storage)
 * Date               : 2026/08/28
 * Description        : Optical Comm Handheld Device on CH32V203 @ 144MHz
 *********************************************************************************/

#include "debug.h"
#include "Anglas_LCD_I2C.h"
#include <string.h>
#include <stdio.h>
#include "at24c64.h"

/* Pin Definitions -----------------------------------------------------------*/
#define B0_Pin          GPIO_Pin_1   // PA1: '1' Button
#define B1_Pin          GPIO_Pin_0   // PA0: '0' Button
#define B2_Pin          GPIO_Pin_2   // PA2: Backspace Button
#define B4_Pin          GPIO_Pin_3   // PA3: Transmit Button
#define LED_Pin         GPIO_Pin_4   // PA4: Optical TX LED
#define ENCODER_A_PIN   GPIO_Pin_5   // PA5: Rotary Encoder A
#define RX_Pin          GPIO_Pin_7   // PA7: Optical RX Signal
#define ENCODER_B_PIN   GPIO_Pin_8   // PA8: Rotary Encoder B
#define SOFT_SDA_PIN    GPIO_Pin_9   // PA9: Software I2C SDA
#define SOFT_SCL_PIN    GPIO_Pin_10  // PA10: Software I2C SCL
#define ENCODER_BTN_PIN GPIO_Pin_0   // PB0: Rotary Encoder Push Button

#define LED_Port        GPIOA

/* Direct Register Pin Control (Active-Low Circuit) --------------------------*/
// Pin HIGH (BSHR) -> 3.3V -> Physical LED is OFF (Darkness / Bit 0 / Idle)
// Pin LOW  (BCR)  -> 0.0V -> Physical LED is ON  (Light Pulse / Bit 1)
#define LED_OFF()       (LED_Port->BSHR = LED_Pin)
#define LED_ON()        (LED_Port->BCR  = LED_Pin)

/* Dynamic Protocol Timing Configuration -------------------------------------*/
uint16_t g_bit_period_us = 10;                         // Default 10us bit period (100kb/s)

#define BIT_PERIOD_US         (g_bit_period_us)
#define BIT_CENTER_US         (g_bit_period_us / 2)
#define PREAMBLE_PERIOD_US    (g_bit_period_us)
#define START_PULSE_US        (g_bit_period_us * 4)
#define VALID_SYNC_THRESH_US  (g_bit_period_us * 5 / 2)
#define END_MARKER_BYTE       0x04                     // ASCII EOT (End of Transmission)

#define MAX_CHARS             8192

/* EEPROM Storage Configuration ----------------------------------------------*/
#define MAX_SAVED_MSGS        8
#define MSG_NAME_LEN          12

typedef struct {
    char name[MSG_NAME_LEN];
    uint16_t bit_count;
    uint16_t ee_offset;
} MsgEntry;

/* Menu Configuration --------------------------------------------------------*/
#define MENU_ITEMS_COUNT      9
const char* menu_items[MENU_ITEMS_COUNT] = {
    "1. Clear Message",
    "2. Exit Menu",
    "3. Saved Msgs",
    "4. Save Message",
    "5. Delete Msg",
    "6. Erase Msgs",
    "7. Set Bit Period",
    "8. Erase EEPROM",
    "9. Run EEPROM Tst"
};

/* Global Variables ----------------------------------------------------------*/
uint8_t bitHistory[MAX_CHARS / 8] = {0};

int count = 0;        // Absolute bit index
int ascii_count = 0;  // Fully typed bytes
int binary_count = 0; // Current uncommitted bits

uint8_t zeroVal = 1,     lastzeroVal = 1;
uint8_t oneVal = 1,      lastoneVal = 1;
uint8_t displayLast = 1, lastdisplayLast = 1;
uint8_t transmit = 1,    lasttransmit = 1;

int viewer_mode = 0;  // 0 = Typing Mode, 1 = Reading Mode
int viewer_page = 0;  // Active viewed page (0-indexed)
int total_pages = 1;  // Total calculated pages

int menu_mode = 0;
int menu_select = 0;
uint8_t btn_last = 1;

char buff[32];

/* Function Prototypes -------------------------------------------------------*/
void TIM2_Microsecond_Init(void);
static inline void Wait_Until(uint16_t target_time);
uint64_t Get_RV_Cycles(void);
void GPIO_Peripheral_Init(void);
static void Redraw_Current_Page(void);
void Transmit(uint8_t *data_array);
uint16_t Expect_Frame_Header(void);
static uint8_t Sample_Bit_With_Resync(uint16_t *p_sample_time, uint8_t *p_last_raw_pin);
uint8_t Receive_And_Display(void);
void Handle_Rotary_Navigation(void);
void Run_EEPROM_Test(void);

/* EEPROM Storage Manager Prototypes -----------------------------------------*/
static void EEPROM_Write_Buffer(uint16_t addr, const uint8_t *buf, uint16_t len);
static void EEPROM_Read_Buffer(uint16_t addr, uint8_t *buf, uint16_t len);
static void EEPROM_Init_Check(void);
static uint8_t EEPROM_Get_Msg_Count(void);
static void EEPROM_Set_Msg_Count(uint8_t cnt);
static uint16_t EEPROM_Get_Bit_Period(void);
static void EEPROM_Set_Bit_Period(uint16_t period);
static void EEPROM_Save_Message_Slot(uint8_t index, const char* name, uint16_t bit_len, const uint8_t *data);
static void EEPROM_Load_Message_Slot(uint8_t index, MsgEntry *out_entry, uint8_t *out_data);
static void EEPROM_Delete_Message_Slot(uint8_t index);
static void EEPROM_Erase_Messages(void);
static void EEPROM_Full_Erase(void);

/* Interactive UI Subroutines ------------------------------------------------*/
static int Prompt_Yes_No(const char *title);
static void Enter_Message_Name(char *out_name);
static void Save_Current_Message_UI(void);
static void Show_Saved_Messages_Menu(void);
static void Delete_Message_UI(void);
static void Display_Bitrate_Splash(uint16_t period_us);
static void Set_Bit_Period_UI(void);

/* Hardware Microsecond Timer (TIM2 @ 1MHz Continuous) -----------------------*/
void TIM2_Microsecond_Init(void) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = {0};
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF; // 65535us wrap
    TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock / 1000000) - 1; // 1MHz tick (1us per tick)
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    // Force immediate prescaler shadow register update
    TIM_GenerateEvent(TIM2, TIM_EventSource_Update);
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);

    TIM_Cmd(TIM2, ENABLE);
}

/* Precise Rollover-Safe Timestamp Wait --------------------------------------*/
static inline void Wait_Until(uint16_t target_time) {
    while ((int16_t)((uint16_t)TIM2->CNT - target_time) < 0);
}

/* Cycle Counter for I2C Peripheral Driver -----------------------------------*/
uint64_t Get_RV_Cycles(void) {
    return ((uint64_t)TIM2->CNT) * (SystemCoreClock / 1000000);
}

/* EEPROM Low-Level Helpers --------------------------------------------------*/
static void EEPROM_Write_Buffer(uint16_t addr, const uint8_t *buf, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        AT24C64_WriteByte(addr + i, buf[i]);
        Delay_Ms(5);
    }
}

static void EEPROM_Read_Buffer(uint16_t addr, uint8_t *buf, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        AT24C64_ReadByte(addr + i, &buf[i]);
    }
}

static void EEPROM_Init_Check(void) {
    uint8_t magic[4] = {0};
    EEPROM_Read_Buffer(0x0000, magic, 4);
    if (magic[0] != 'M' || magic[1] != 'S' || magic[2] != 'G' || magic[3] != 0x01) {
        // [0..3]=Magic, [4]=Msg Count, [5]=Reserved, [6..7]=Bit Period (10us little-endian)
        uint8_t init_hdr[8] = {'M', 'S', 'G', 0x01, 0, 0, 10, 0};
        EEPROM_Write_Buffer(0x0000, init_hdr, 8);
    }
}

static uint8_t EEPROM_Get_Msg_Count(void) {
    EEPROM_Init_Check();
    uint8_t cnt = 0;
    AT24C64_ReadByte(0x0004, &cnt);
    if (cnt > MAX_SAVED_MSGS) cnt = 0;
    return cnt;
}

static void EEPROM_Set_Msg_Count(uint8_t cnt) {
    AT24C64_WriteByte(0x0004, cnt);
    Delay_Ms(5);
}

static uint16_t EEPROM_Get_Bit_Period(void) {
    EEPROM_Init_Check();
    uint8_t buf[2] = {0};
    EEPROM_Read_Buffer(0x0006, buf, 2);
    uint16_t val = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    if (val < 2 || val > 10000) {
        val = 10;
        EEPROM_Set_Bit_Period(10);
    }
    return val;
}

static void EEPROM_Set_Bit_Period(uint16_t period) {
    uint8_t buf[2];
    buf[0] = (uint8_t)(period & 0xFF);
    buf[1] = (uint8_t)((period >> 8) & 0xFF);
    EEPROM_Write_Buffer(0x0006, buf, 2);
}

static uint16_t Get_Entry_Addr(uint8_t index) {
    return 0x0010 + (index * sizeof(MsgEntry));
}

static uint16_t Get_Payload_Addr(uint8_t index) {
    return 0x0200 + (index * 0x0300);
}

static void EEPROM_Save_Message_Slot(uint8_t index, const char* name, uint16_t bit_len, const uint8_t *data) {
    MsgEntry entry;
    memset(&entry, 0, sizeof(entry));
    strncpy(entry.name, name, MSG_NAME_LEN - 1);
    entry.bit_count = bit_len;
    entry.ee_offset = Get_Payload_Addr(index);

    EEPROM_Write_Buffer(Get_Entry_Addr(index), (uint8_t*)&entry, sizeof(MsgEntry));

    uint16_t byte_len = (bit_len + 7) / 8;
    if (byte_len > 768) byte_len = 768;
    EEPROM_Write_Buffer(entry.ee_offset, data, byte_len);
}

static void EEPROM_Load_Message_Slot(uint8_t index, MsgEntry *out_entry, uint8_t *out_data) {
    EEPROM_Read_Buffer(Get_Entry_Addr(index), (uint8_t*)out_entry, sizeof(MsgEntry));
    uint16_t byte_len = (out_entry->bit_count + 7) / 8;
    if (byte_len > 768) byte_len = 768;
    EEPROM_Read_Buffer(out_entry->ee_offset, out_data, byte_len);
}

static void EEPROM_Delete_Message_Slot(uint8_t index) {
    uint8_t cnt = EEPROM_Get_Msg_Count();
    if (index >= cnt) return;

    uint8_t chunk_buf[64];

    for (uint8_t i = index; i < cnt - 1; i++) {
        MsgEntry next_entry;
        EEPROM_Read_Buffer(Get_Entry_Addr(i + 1), (uint8_t*)&next_entry, sizeof(MsgEntry));

        uint16_t byte_len = (next_entry.bit_count + 7) / 8;
        if (byte_len > 768) byte_len = 768;

        uint16_t src_payload_addr = Get_Payload_Addr(i + 1);
        uint16_t dst_payload_addr = Get_Payload_Addr(i);

        for (uint16_t offset = 0; offset < byte_len; offset += 64) {
            uint16_t chunk_len = (byte_len - offset > 64) ? 64 : (byte_len - offset);
            EEPROM_Read_Buffer(src_payload_addr + offset, chunk_buf, chunk_len);
            EEPROM_Write_Buffer(dst_payload_addr + offset, chunk_buf, chunk_len);
        }

        next_entry.ee_offset = dst_payload_addr;
        EEPROM_Write_Buffer(Get_Entry_Addr(i), (uint8_t*)&next_entry, sizeof(MsgEntry));
    }

    MsgEntry zero_entry;
    memset(&zero_entry, 0, sizeof(zero_entry));
    EEPROM_Write_Buffer(Get_Entry_Addr(cnt - 1), (uint8_t*)&zero_entry, sizeof(zero_entry));

    EEPROM_Set_Msg_Count(cnt - 1);
}

static void EEPROM_Erase_Messages(void) {
    EEPROM_Set_Msg_Count(0);
    uint8_t zero_buf[sizeof(MsgEntry)] = {0};
    for (uint8_t i = 0; i < MAX_SAVED_MSGS; i++) {
        EEPROM_Write_Buffer(Get_Entry_Addr(i), zero_buf, sizeof(MsgEntry));
    }
}

static void EEPROM_Full_Erase(void) {
    uint8_t ff_buf[32];
    memset(ff_buf, 0xFF, sizeof(ff_buf));
    for (uint16_t addr = 0; addr < 8192; addr += 32) {
        EEPROM_Write_Buffer(addr, ff_buf, 32);
    }
    EEPROM_Init_Check();
    g_bit_period_us = 10;
}

/* GPIO Initialization -------------------------------------------------------*/
void GPIO_Peripheral_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    // Inputs: Buttons & Rotary Encoder A/B
    GPIO_InitStructure.GPIO_Pin = B0_Pin | B1_Pin | B2_Pin | B4_Pin | ENCODER_A_PIN | ENCODER_B_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // PA7 Optical RX Signal (Input Pull-Down)
    GPIO_InitStructure.GPIO_Pin = RX_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Rotary Encoder Push Button (PB0)
    GPIO_InitStructure.GPIO_Pin = ENCODER_BTN_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // Open-Drain Outputs for Software I2C
    GPIO_InitStructure.GPIO_Pin = SOFT_SCL_PIN | SOFT_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Push-Pull Output for LED Transmitter
    GPIO_InitStructure.GPIO_Pin = LED_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LED_Port, &GPIO_InitStructure);

    LED_OFF(); // Pin HIGH -> LED is PHYSICALLY OFF on boot
}

/* Main Execution Loop -------------------------------------------------------*/
int main(void) {
    SystemCoreClockUpdate();
    TIM2_Microsecond_Init(); // Start continuous 1MHz hardware microsecond timer
    Delay_Init();            // WCH standard Delay engine
    USART_Printf_Init(115200);

    printf("CH32V203 Running @ %u Hz\r\n", (unsigned int)SystemCoreClock);

    GPIO_Peripheral_Init();
    AT24C64_Init();
    EEPROM_Init_Check();
    g_bit_period_us = EEPROM_Get_Bit_Period(); // Load persisted Bit Period
    LCD_I2C_Init();

    // Startup Banner
    LCD_I2C_WriteText(1, 1, " -INITIALIZED- ");
    Delay_Ms(500);
    LCD_I2C_ClearText();
    
    // Draw initial empty page with active blinking cursor immediately on boot
    Redraw_Current_Page();

    while (1) {
        // Optical Reception Handler
        if (GPIOA->INDR & RX_Pin) {
            if (Receive_And_Display()) {
                Redraw_Current_Page();
            }
        }

        Handle_Rotary_Navigation();

        if (menu_mode == 0) {
            zeroVal     = (GPIOA->INDR & B1_Pin) ? 1 : 0;
            oneVal      = (GPIOA->INDR & B0_Pin) ? 1 : 0;
            displayLast = (GPIOA->INDR & B2_Pin) ? 1 : 0;
            transmit    = (GPIOA->INDR & B4_Pin) ? 1 : 0;

            // --- CHECK '0' BUTTON (B1_Pin) ---
            if ((zeroVal == 0) && (lastzeroVal == 1)) {
                Delay_Ms(25);
                if ((GPIOA->INDR & B1_Pin) == 0) {
                    if (viewer_mode == 1) {
                        viewer_page = (viewer_page == 0) ? (total_pages - 1) : (viewer_page - 1);
                        Redraw_Current_Page();
                    } else if (count < MAX_CHARS) {
                        bitHistory[count / 8] &= ~(1 << (count % 8));
                        count++; binary_count++;
                        if (binary_count == 8) { ascii_count++; binary_count = 0; }
                        Redraw_Current_Page();
                    }
                }
            }

            // --- CHECK '1' BUTTON (B0_Pin) ---
            if ((oneVal == 0) && (lastoneVal == 1)) {
                Delay_Ms(25);
                if ((GPIOA->INDR & B0_Pin) == 0) {
                    if (viewer_mode == 1) {
                        viewer_page = (viewer_page + 1 >= total_pages) ? 0 : (viewer_page + 1);
                        Redraw_Current_Page();
                    } else if (count < MAX_CHARS) {
                        bitHistory[count / 8] |= (1 << (count % 8));
                        count++; binary_count++;
                        if (binary_count == 8) { ascii_count++; binary_count = 0; }
                        Redraw_Current_Page();
                    }
                }
            }

            // --- CHECK BACKSPACE BUTTON (B2_Pin) ---
            if ((displayLast == 0) && (lastdisplayLast == 1)) {
                Delay_Ms(25);
                if ((GPIOA->INDR & B2_Pin) == 0) {
                    if (viewer_mode == 1) {
                        memset(bitHistory, 0, sizeof(bitHistory));
                        count = 0; ascii_count = 0; binary_count = 0; viewer_page = 0; viewer_mode = 0;
                        Redraw_Current_Page();
                    } else if (count > 0) {
                        if (binary_count > 0) {
                            count--; binary_count--;
                            bitHistory[count / 8] &= ~(1 << (count % 8));
                        } else if (ascii_count > 0) {
                            ascii_count--; count -= 8;
                            for (int b = 0; b < 8; b++) bitHistory[(count + b) / 8] &= ~(1 << ((count + b) % 8));
                            binary_count = 0;
                        }
                        Redraw_Current_Page();
                    }
                }
            }

            // --- TRANSMISSION MANAGEMENT (B4_Pin) ---
            if ((transmit == 0) && (lasttransmit == 1)) {
                Delay_Ms(25);
                if ((GPIOA->INDR & B4_Pin) == 0) {
                    if (count >= 8) {
                        LCD_I2C_Set_Cursor_Blink(1, 1, 0);
                        LCD_I2C_ClearText();
                        sprintf(buff, "Transmitting...");
                        LCD_I2C_WriteText(1, 1, buff);

                        do {
                            Transmit(bitHistory);
                            Delay_Ms(2);
                            transmit = (GPIOA->INDR & B4_Pin) ? 1 : 0;
                        } while (transmit == 0);

                        Delay_Ms(30);
                        while ((GPIOA->INDR & B4_Pin) == 0);
                        Delay_Ms(30);

                        Redraw_Current_Page();
                    } else {
                        LCD_I2C_Set_Cursor_Blink(1, 1, 0);
                        LCD_I2C_ClearText();
                        LCD_I2C_WriteRow(1, "No Message Typed");
                        LCD_I2C_WriteRow(2, "Min 1 Char / 8b ");
                        Delay_Ms(1000);
                        Redraw_Current_Page();
                    }
                }
            }

            lastzeroVal = zeroVal;
            lastoneVal = oneVal;
            lastdisplayLast = displayLast;
            lasttransmit = transmit;
        }

        Delay_Ms(1);
    }
}

/* Interactive UI Subroutines ------------------------------------------------*/
static int Prompt_Yes_No(const char *title) {
    int sel = 0;
    uint8_t btn_last_local = (GPIOB->INDR & ENCODER_BTN_PIN) ? 1 : 0;
    
    LCD_I2C_ClearText();
    LCD_I2C_WriteRow(1, (char*)title);
    
    uint8_t enc_prev = 0x03;
    int8_t enc_dir = 0;
    
    while (1) {
        char r2[17];
        sprintf(r2, " %cNo    %cYes", (sel == 0 ? '>' : ' '), (sel == 1 ? '>' : ' '));
        LCD_I2C_WriteRow(2, r2);
        
        uint8_t btn_now = (GPIOB->INDR & ENCODER_BTN_PIN) ? 1 : 0;
        if (btn_last_local == 1 && btn_now == 0) {
            Delay_Ms(30);
            if ((GPIOB->INDR & ENCODER_BTN_PIN) == 0) {
                while ((GPIOB->INDR & ENCODER_BTN_PIN) == 0);
                Delay_Ms(30);
                return sel;
            }
        }
        btn_last_local = btn_now;
        
        uint8_t a = (GPIOA->INDR & ENCODER_A_PIN) ? 1 : 0;
        uint8_t b = (GPIOA->INDR & ENCODER_B_PIN) ? 1 : 0;
        uint8_t curr_state = (a << 1) | b;
        if (curr_state != enc_prev) {
            if (enc_prev == 0x03) {
                if (curr_state == 0x01) enc_dir = 1;
                else if (curr_state == 0x02) enc_dir = -1;
            } else if (curr_state == 0x03) {
                if (enc_dir != 0) {
                    sel = !sel;
                }
                enc_dir = 0;
            }
            enc_prev = curr_state;
        }
        Delay_Ms(2);
    }
}

static void Enter_Message_Name(char *out_name) {
    const char charset[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789 -<*";
    const int charset_len = sizeof(charset) - 1;
    
    char name_buf[MSG_NAME_LEN] = {0};
    int name_idx = 0;
    int char_select = 0;
    
    uint8_t btn_last_local = (GPIOB->INDR & ENCODER_BTN_PIN) ? 1 : 0;
    uint8_t b2_last_local  = (GPIOA->INDR & B2_Pin) ? 1 : 0;
    uint8_t enc_prev = 0x03;
    int8_t enc_dir = 0;

    LCD_I2C_ClearText();

    while (1) {
        char row1[17];
        char row2[17];
        
        sprintf(row1, "Name:%-11s", name_buf);
        
        char c = charset[char_select];
        if (c == '*') {
            sprintf(row2, "Sel: [DONE]");
        } else if (c == '<') {
            sprintf(row2, "Sel: [BACK]");
        } else {
            sprintf(row2, "Sel: [ %c ]", c);
        }
        
        LCD_I2C_WriteRow(1, row1);
        LCD_I2C_WriteRow(2, row2);

        uint8_t btn_now = (GPIOB->INDR & ENCODER_BTN_PIN) ? 1 : 0;
        if (btn_last_local == 1 && btn_now == 0) {
            Delay_Ms(30);
            if ((GPIOB->INDR & ENCODER_BTN_PIN) == 0) {
                while ((GPIOB->INDR & ENCODER_BTN_PIN) == 0);
                Delay_Ms(30);
                
                if (c == '*') {
                    if (name_idx == 0) {
                        sprintf(name_buf, "MSG_%d", EEPROM_Get_Msg_Count() + 1);
                    }
                    break;
                } else if (c == '<') {
                    if (name_idx > 0) {
                        name_idx--;
                        name_buf[name_idx] = '\0';
                    }
                } else {
                    if (name_idx < MSG_NAME_LEN - 1) {
                        name_buf[name_idx++] = c;
                        name_buf[name_idx] = '\0';
                    }
                }
            }
        }
        btn_last_local = btn_now;

        uint8_t b2_now = (GPIOA->INDR & B2_Pin) ? 1 : 0;
        if (b2_last_local == 1 && b2_now == 0) {
            Delay_Ms(25);
            if ((GPIOA->INDR & B2_Pin) == 0) {
                if (name_idx > 0) {
                    name_idx--;
                    name_buf[name_idx] = '\0';
                }
            }
        }
        b2_last_local = b2_now;

        uint8_t a = (GPIOA->INDR & ENCODER_A_PIN) ? 1 : 0;
        uint8_t b = (GPIOA->INDR & ENCODER_B_PIN) ? 1 : 0;
        uint8_t curr_state = (a << 1) | b;

        if (curr_state != enc_prev) {
            if (enc_prev == 0x03) {
                if (curr_state == 0x01) enc_dir = 1;
                else if (curr_state == 0x02) enc_dir = -1;
            } else if (curr_state == 0x03) {
                if (enc_dir == 1) {
                    char_select = (char_select + 1 >= charset_len) ? 0 : (char_select + 1);
                } else if (enc_dir == -1) {
                    char_select = (char_select == 0) ? (charset_len - 1) : (char_select - 1);
                }
                enc_dir = 0;
            }
            enc_prev = curr_state;
        }
        Delay_Ms(2);
    }
    
    strncpy(out_name, name_buf, MSG_NAME_LEN - 1);
    out_name[MSG_NAME_LEN - 1] = '\0';
}

static void Save_Current_Message_UI(void) {
    if (count == 0) {
        LCD_I2C_ClearText();
        LCD_I2C_WriteRow(1, "No Message Typed");
        LCD_I2C_WriteRow(2, " Type one first! ");
        Delay_Ms(2000);
        menu_mode = 0;
        Redraw_Current_Page();
        return;
    }

    uint8_t msg_cnt = EEPROM_Get_Msg_Count();
    if (msg_cnt >= MAX_SAVED_MSGS) {
        LCD_I2C_ClearText();
        LCD_I2C_WriteRow(1, " Storage Full!  ");
        LCD_I2C_WriteRow(2, "  Max 8 Msgs    ");
        Delay_Ms(2000);
        menu_mode = 0;
        Redraw_Current_Page();
        return;
    }

    char msg_name[MSG_NAME_LEN] = {0};
    Enter_Message_Name(msg_name);

    EEPROM_Save_Message_Slot(msg_cnt, msg_name, count, bitHistory);
    EEPROM_Set_Msg_Count(msg_cnt + 1);

    LCD_I2C_ClearText();
    LCD_I2C_WriteRow(1, " Message Saved! ");
    sprintf(buff, "Name: %s", msg_name);
    LCD_I2C_WriteRow(2, buff);
    Delay_Ms(1500);

    menu_mode = 0;
    Redraw_Current_Page();
}

static void Show_Saved_Messages_Menu(void) {
    uint8_t msg_cnt = EEPROM_Get_Msg_Count();
    if (msg_cnt == 0) {
        LCD_I2C_ClearText();
        LCD_I2C_WriteRow(1, " No Saved Msgs! ");
        LCD_I2C_WriteRow(2, "   Press Btn    ");
        Delay_Ms(2000);
        menu_mode = 0;
        Redraw_Current_Page();
        return;
    }

    int sub_select = 0;
    int list_total = msg_cnt + 1;
    uint8_t btn_last_local = (GPIOB->INDR & ENCODER_BTN_PIN) ? 1 : 0;
    uint8_t enc_prev = 0x03;
    int8_t enc_dir = 0;

    LCD_I2C_ClearText();

    while (1) {
        int i1 = sub_select;
        int i2 = sub_select + 1;
        if (i2 >= list_total) {
            i2 = sub_select;
            i1 = sub_select - 1;
        }

        char r1[17] = {0};
        char r2[17] = {0};

        if (i1 < msg_cnt) {
            MsgEntry e1;
            EEPROM_Read_Buffer(Get_Entry_Addr(i1), (uint8_t*)&e1, sizeof(MsgEntry));
            sprintf(r1, "%c%d: %s", (i1 == sub_select ? '>' : ' '), i1 + 1, e1.name);
        } else {
            sprintf(r1, "%c%d: Exit", (i1 == sub_select ? '>' : ' '), i1 + 1);
        }

        if (i2 < msg_cnt) {
            MsgEntry e2;
            EEPROM_Read_Buffer(Get_Entry_Addr(i2), (uint8_t*)&e2, sizeof(MsgEntry));
            sprintf(r2, "%c%d: %s", (i2 == sub_select ? '>' : ' '), i2 + 1, e2.name);
        } else {
            sprintf(r2, "%c%d: Exit", (i2 == sub_select ? '>' : ' '), i2 + 1);
        }

        LCD_I2C_WriteRow(1, r1);
        LCD_I2C_WriteRow(2, r2);

        uint8_t btn_now = (GPIOB->INDR & ENCODER_BTN_PIN) ? 1 : 0;
        if (btn_last_local == 1 && btn_now == 0) {
            Delay_Ms(30);
            if ((GPIOB->INDR & ENCODER_BTN_PIN) == 0) {
                while ((GPIOB->INDR & ENCODER_BTN_PIN) == 0);
                Delay_Ms(30);

                if (sub_select == msg_cnt) {
                    menu_mode = 0;
                    Redraw_Current_Page();
                    return;
                } else {
                    MsgEntry loaded_entry;
                    EEPROM_Load_Message_Slot(sub_select, &loaded_entry, bitHistory);
                    count = loaded_entry.bit_count;
                    ascii_count = count / 8;
                    binary_count = count % 8;
                    viewer_mode = 0;
                    viewer_page = 0;
                    menu_mode = 0;

                    LCD_I2C_ClearText();
                    LCD_I2C_WriteRow(1, " Message Loaded ");
                    sprintf(buff, "Name: %s", loaded_entry.name);
                    LCD_I2C_WriteRow(2, buff);
                    Delay_Ms(1500);

                    Redraw_Current_Page();
                    return;
                }
            }
        }
        btn_last_local = btn_now;

        uint8_t a = (GPIOA->INDR & ENCODER_A_PIN) ? 1 : 0;
        uint8_t b = (GPIOA->INDR & ENCODER_B_PIN) ? 1 : 0;
        uint8_t curr_state = (a << 1) | b;

        if (curr_state != enc_prev) {
            if (enc_prev == 0x03) {
                if (curr_state == 0x01) enc_dir = 1;
                else if (curr_state == 0x02) enc_dir = -1;
            } else if (curr_state == 0x03) {
                if (enc_dir == 1) {
                    sub_select = (sub_select + 1 >= list_total) ? 0 : (sub_select + 1);
                } else if (enc_dir == -1) {
                    sub_select = (sub_select == 0) ? (list_total - 1) : (sub_select - 1);
                }
                enc_dir = 0;
            }
            enc_prev = curr_state;
        }
        Delay_Ms(2);
    }
}

static void Delete_Message_UI(void) {
    uint8_t msg_cnt = EEPROM_Get_Msg_Count();
    if (msg_cnt == 0) {
        LCD_I2C_ClearText();
        LCD_I2C_WriteRow(1, " No Saved Msgs! ");
        LCD_I2C_WriteRow(2, "   Press Btn    ");
        Delay_Ms(2000);
        menu_mode = 0;
        Redraw_Current_Page();
        return;
    }

    int sub_select = 0;
    int list_total = msg_cnt + 1;
    uint8_t btn_last_local = (GPIOB->INDR & ENCODER_BTN_PIN) ? 1 : 0;
    uint8_t enc_prev = 0x03;
    int8_t enc_dir = 0;

    LCD_I2C_ClearText();

    while (1) {
        int i1 = sub_select;
        int i2 = sub_select + 1;
        if (i2 >= list_total) {
            i2 = sub_select;
            i1 = sub_select - 1;
        }

        char r1[17] = {0};
        char r2[17] = {0};

        if (i1 < msg_cnt) {
            MsgEntry e1;
            EEPROM_Read_Buffer(Get_Entry_Addr(i1), (uint8_t*)&e1, sizeof(MsgEntry));
            sprintf(r1, "%c%d: %s", (i1 == sub_select ? '>' : ' '), i1 + 1, e1.name);
        } else {
            sprintf(r1, "%c%d: Exit", (i1 == sub_select ? '>' : ' '), i1 + 1);
        }

        if (i2 < msg_cnt) {
            MsgEntry e2;
            EEPROM_Read_Buffer(Get_Entry_Addr(i2), (uint8_t*)&e2, sizeof(MsgEntry));
            sprintf(r2, "%c%d: %s", (i2 == sub_select ? '>' : ' '), i2 + 1, e2.name);
        } else {
            sprintf(r2, "%c%d: Exit", (i2 == sub_select ? '>' : ' '), i2 + 1);
        }

        LCD_I2C_WriteRow(1, r1);
        LCD_I2C_WriteRow(2, r2);

        uint8_t btn_now = (GPIOB->INDR & ENCODER_BTN_PIN) ? 1 : 0;
        if (btn_last_local == 1 && btn_now == 0) {
            Delay_Ms(30);
            if ((GPIOB->INDR & ENCODER_BTN_PIN) == 0) {
                while ((GPIOB->INDR & ENCODER_BTN_PIN) == 0);
                Delay_Ms(30);

                if (sub_select == msg_cnt) {
                    menu_mode = 0;
                    Redraw_Current_Page();
                    return;
                } else {
                    if (Prompt_Yes_No(" Delete Msg?    ")) {
                        EEPROM_Delete_Message_Slot(sub_select);
                        LCD_I2C_ClearText();
                        LCD_I2C_WriteRow(1, " Message Deleted");
                        Delay_Ms(1500);
                    }
                    menu_mode = 0;
                    Redraw_Current_Page();
                    return;
                }
            }
        }
        btn_last_local = btn_now;

        uint8_t a = (GPIOA->INDR & ENCODER_A_PIN) ? 1 : 0;
        uint8_t b = (GPIOA->INDR & ENCODER_B_PIN) ? 1 : 0;
        uint8_t curr_state = (a << 1) | b;

        if (curr_state != enc_prev) {
            if (enc_prev == 0x03) {
                if (curr_state == 0x01) enc_dir = 1;
                else if (curr_state == 0x02) enc_dir = -1;
            } else if (curr_state == 0x03) {
                if (enc_dir == 1) {
                    sub_select = (sub_select + 1 >= list_total) ? 0 : (sub_select + 1);
                } else if (enc_dir == -1) {
                    sub_select = (sub_select == 0) ? (list_total - 1) : (sub_select - 1);
                }
                enc_dir = 0;
            }
            enc_prev = curr_state;
        }
        Delay_Ms(2);
    }
}

/* Bitrate Confirmation Splash Screen ----------------------------------------*/
static void Display_Bitrate_Splash(uint16_t period_us) {
    LCD_I2C_ClearText();
    LCD_I2C_WriteRow(1, " Bitrate Set To ");

    char rate_buf[17] = {0};
    uint32_t bps = 1000000UL / period_us;

    if (bps >= 1000) {
        uint32_t kb_int = bps / 1000;
        uint32_t kb_dec = (bps % 1000) / 100;
        if (kb_dec > 0) {
            sprintf(rate_buf, "%lu.%lu kb/s (%uus)", (unsigned long)kb_int, (unsigned long)kb_dec, (unsigned int)period_us);
        } else {
            sprintf(rate_buf, "%lu kb/s (%uus)", (unsigned long)kb_int, (unsigned int)period_us);
        }
    } else {
        sprintf(rate_buf, "%lu b/s (%uus)", (unsigned long)bps, (unsigned int)period_us);
    }

    LCD_I2C_WriteRow(2, rate_buf);
    Delay_Ms(2000);
}

/* Set Bit Period UI ---------------------------------------------------------*/
static void Set_Bit_Period_UI(void) {
    uint16_t cur_period = g_bit_period_us;
    uint8_t btn_last_local = (GPIOB->INDR & ENCODER_BTN_PIN) ? 1 : 0;
    uint8_t b2_last_local  = (GPIOA->INDR & B2_Pin) ? 1 : 0;
    uint8_t enc_prev = 0x03;
    int8_t enc_dir = 0;

    LCD_I2C_ClearText();

    while (1) {
        char r1[17] = {0};
        char r2[17] = {0};

        sprintf(r1, "Period: %-5u us", cur_period);

        uint32_t bps = 1000000UL / cur_period;
        if (bps >= 1000) {
            uint32_t kb_int = bps / 1000;
            uint32_t kb_dec = (bps % 1000) / 100;
            if (kb_dec > 0) {
                sprintf(r2, "%lu.%lu kb/s  [OK]", (unsigned long)kb_int, (unsigned long)kb_dec);
            } else {
                sprintf(r2, "%lu kb/s    [OK]", (unsigned long)kb_int);
            }
        } else {
            sprintf(r2, "%lu b/s     [OK]", (unsigned long)bps);
        }

        LCD_I2C_WriteRow(1, r1);
        LCD_I2C_WriteRow(2, r2);

        // Confirm & Save
        uint8_t btn_now = (GPIOB->INDR & ENCODER_BTN_PIN) ? 1 : 0;
        if (btn_last_local == 1 && btn_now == 0) {
            Delay_Ms(30);
            if ((GPIOB->INDR & ENCODER_BTN_PIN) == 0) {
                while ((GPIOB->INDR & ENCODER_BTN_PIN) == 0);
                Delay_Ms(30);

                g_bit_period_us = cur_period;
                EEPROM_Set_Bit_Period(g_bit_period_us);
                Display_Bitrate_Splash(g_bit_period_us);

                menu_mode = 0;
                Redraw_Current_Page();
                return;
            }
        }
        btn_last_local = btn_now;

        // Cancel on Backspace (B2)
        uint8_t b2_now = (GPIOA->INDR & B2_Pin) ? 1 : 0;
        if (b2_last_local == 1 && b2_now == 0) {
            Delay_Ms(25);
            if ((GPIOA->INDR & B2_Pin) == 0) {
                while ((GPIOA->INDR & B2_Pin) == 0);
                menu_mode = 0;
                Redraw_Current_Page();
                return;
            }
        }
        b2_last_local = b2_now;

        // Rotary Encoder Adjustment with Magnitude Acceleration
        uint8_t a = (GPIOA->INDR & ENCODER_A_PIN) ? 1 : 0;
        uint8_t b = (GPIOA->INDR & ENCODER_B_PIN) ? 1 : 0;
        uint8_t curr_state = (a << 1) | b;

        if (curr_state != enc_prev) {
            if (enc_prev == 0x03) {
                if (curr_state == 0x01) enc_dir = 1;
                else if (curr_state == 0x02) enc_dir = -1;
            } else if (curr_state == 0x03) {
                int step = 1;
                if (cur_period >= 1000)      step = 100;
                else if (cur_period >= 100)  step = 10;
                else if (cur_period >= 20)   step = 2;
                else                         step = 1;

                if (enc_dir == 1) {
                    if (cur_period + step <= 10000) cur_period += step;
                } else if (enc_dir == -1) {
                    if (cur_period > step && (cur_period - step >= 2)) cur_period -= step;
                    else cur_period = 2;
                }
                enc_dir = 0;
            }
            enc_prev = curr_state;
        }
        Delay_Ms(2);
    }
}

/* Rotary Encoder Navigation -------------------------------------------------*/
void Handle_Rotary_Navigation(void) {
    uint8_t btn_now = (GPIOB->INDR & ENCODER_BTN_PIN) ? 1 : 0;
    
    if (btn_last == 1 && btn_now == 0) {
        Delay_Ms(20);
        if ((GPIOB->INDR & ENCODER_BTN_PIN) == 0) {
            if (menu_mode == 0) {
                menu_mode = 1; 
                menu_select = 0;
                Redraw_Current_Page();
            } else {
                if (menu_select == 0) {        // 1. Clear Message
                    memset(bitHistory, 0, sizeof(bitHistory));
                    count = 0; 
                    ascii_count = 0; 
                    binary_count = 0; 
                    viewer_page = 0; 
                    viewer_mode = 0;
                    menu_mode = 0;

                    LCD_I2C_ClearText();
                    LCD_I2C_WriteRow(1, "Message Cleared!");
                    Delay_Ms(800);
                    Redraw_Current_Page();
                } else if (menu_select == 1) { // 2. Exit Menu
                    menu_mode = 0;
                    Redraw_Current_Page();
                } else if (menu_select == 2) { // 3. Saved Msgs
                    Show_Saved_Messages_Menu();
                } else if (menu_select == 3) { // 4. Save Message
                    Save_Current_Message_UI();
                } else if (menu_select == 4) { // 5. Delete Msg
                    Delete_Message_UI();
                } else if (menu_select == 5) { // 6. Erase Msgs
                    if (Prompt_Yes_No("  Erase Msgs?   ")) {
                        EEPROM_Erase_Messages();
                        LCD_I2C_ClearText();
                        LCD_I2C_WriteRow(1, " Messages Erased ");
                        Delay_Ms(1500);
                    }
                    menu_mode = 0;
                    Redraw_Current_Page();
                } else if (menu_select == 6) { // 7. Set Bit Period
                    Set_Bit_Period_UI();
                } else if (menu_select == 7) { // 8. Erase EEPROM
                    if (Prompt_Yes_No(" Erase EEPROM?  ")) {
                        LCD_I2C_ClearText();
                        LCD_I2C_WriteRow(1, " Erasing EEPROM...");
                        EEPROM_Full_Erase();
                        LCD_I2C_ClearText();
                        LCD_I2C_WriteRow(1, "EEPROM Formatted!");
                        Delay_Ms(1500);
                    }
                    menu_mode = 0;
                    Redraw_Current_Page();
                } else if (menu_select == 8) { // 9. Run EEPROM Tst
                    Run_EEPROM_Test();
                }
            }
        }
    }
    btn_last = btn_now;

    static uint8_t encoder_prev_state = 0x03;
    static int8_t encoder_dir = 0;

    uint8_t a = (GPIOA->INDR & ENCODER_A_PIN) ? 1 : 0;
    uint8_t b = (GPIOA->INDR & ENCODER_B_PIN) ? 1 : 0;
    uint8_t curr_state = (a << 1) | b;

    if (curr_state != encoder_prev_state) {
        if (encoder_prev_state == 0x03) {
            if (curr_state == 0x01) {
                encoder_dir = 1;
            } else if (curr_state == 0x02) {
                encoder_dir = -1;
            }
        } 
        else if (curr_state == 0x03) {
            if (encoder_dir == 1) {
                if (menu_mode == 1) {
                    menu_select = (menu_select + 1 >= MENU_ITEMS_COUNT) ? 0 : (menu_select + 1);
                    Redraw_Current_Page();
                } else if (viewer_mode == 1) {
                    viewer_page = (viewer_page + 1 >= total_pages) ? 0 : (viewer_page + 1);
                    Redraw_Current_Page();
                }
            } else if (encoder_dir == -1) {
                if (menu_mode == 1) {
                    menu_select = (menu_select - 1 < 0) ? (MENU_ITEMS_COUNT - 1) : (menu_select - 1);
                    Redraw_Current_Page();
                } else if (viewer_mode == 1) {
                    viewer_page = (viewer_page == 0) ? (total_pages - 1) : (viewer_page - 1);
                    Redraw_Current_Page();
                }
            }
            encoder_dir = 0;
        }
        encoder_prev_state = curr_state;
    }
}

/* Redraw Display Logic ------------------------------------------------------*/
static void Redraw_Current_Page(void) {
    char row1_buf[17] = {0};
    char row2_buf[17] = {0};

    if (menu_mode == 1) {
        LCD_I2C_Set_Cursor_Blink(1, 1, 0);
        int item1 = (menu_select < MENU_ITEMS_COUNT - 1) ? menu_select : (MENU_ITEMS_COUNT - 2);
        int item2 = item1 + 1;

        if (item1 < MENU_ITEMS_COUNT) sprintf(row1_buf, "%c%s", (item1 == menu_select ? '>' : ' '), menu_items[item1] + 3);
        if (item2 < MENU_ITEMS_COUNT) sprintf(row2_buf, "%c%s", (item2 == menu_select ? '>' : ' '), menu_items[item2] + 3);

        LCD_I2C_WriteRow(1, row1_buf);
        LCD_I2C_WriteRow(2, row2_buf);
    } 
    else if (viewer_mode == 1) {
        LCD_I2C_Set_Cursor_Blink(1, 1, 0);

        total_pages = (ascii_count + 15) / 16;
        if (total_pages < 1) total_pages = 1;

        if (viewer_page >= total_pages) viewer_page = 0;
        if (viewer_page < 0) viewer_page = total_pages - 1;

        int page_start_char = viewer_page * 16;

        for (int ch = 0; ch < 16; ch++) {
            int global_char_idx = page_start_char + ch;
            if (global_char_idx >= ascii_count) {
                row1_buf[ch] = ' ';
                continue;
            }

            uint8_t byte_val = 0;
            int bit_start_index = global_char_idx * 8;
            for (int b = 0; b < 8; b++) {
                int current_bit_pos = bit_start_index + b;
                if ((bitHistory[current_bit_pos / 8] >> (current_bit_pos % 8)) & 1) {
                    byte_val |= (1 << (7 - b));
                }
            }
            row1_buf[ch] = (byte_val >= 32 && byte_val <= 126) ? byte_val : ' ';
        }
        row1_buf[16] = '\0';

        sprintf(row2_buf, "Page %d/%d", viewer_page + 1, total_pages);

        LCD_I2C_WriteRow(1, row1_buf);
        LCD_I2C_WriteRow(2, row2_buf);
    } 
    else {
        // --- TYPING MODE ---
        int cur_page_idx;
        if (binary_count == 0 && ascii_count > 0 && (ascii_count % 16) == 0) {
            cur_page_idx = (ascii_count - 1) / 16;
        } else {
            cur_page_idx = ascii_count / 16;
        }

        int cur_page = cur_page_idx + 1;
        int ascii_page_start = cur_page_idx * 16;

        // Render Row 1: Active 16 characters for the current page
        for (int ch = 0; ch < 16; ch++) {
            int global_char_idx = ascii_page_start + ch;
            if (global_char_idx >= ascii_count) {
                row1_buf[ch] = ' ';
                continue;
            }

            uint8_t byte_val = 0;
            int bit_start_index = global_char_idx * 8;
            for (int b = 0; b < 8; b++) {
                int current_bit_pos = bit_start_index + b;
                if ((bitHistory[current_bit_pos / 8] >> (current_bit_pos % 8)) & 1) {
                    byte_val |= (1 << (7 - b));
                }
            }
            row1_buf[ch] = (byte_val >= 32 && byte_val <= 126) ? byte_val : ' ';
        }
        row1_buf[16] = '\0';

        // Pre-fill Row 2 with blanks
        memset(row2_buf, ' ', 16);
        row2_buf[16] = '\0';

        // Left side of Row 2: Render currently uncommitted binary bits (0 to 7)
        int binary_start_index = ascii_count * 8;
        int safe_binary_count = (binary_count > 8) ? 8 : binary_count;
        for (int b = 0; b < safe_binary_count; b++) {
            int current_bit_pos = binary_start_index + b;
            uint8_t bit_val = (bitHistory[current_bit_pos / 8] >> (current_bit_pos % 8)) & 1;
            row2_buf[b] = '0' + bit_val;
        }

        // Right side of Row 2: Show "Page X" ONLY when Page 2 or higher is active
        if (cur_page > 1) {
            char page_buf[10];
            int page_len = sprintf(page_buf, "Page %d", cur_page);
            if (page_len <= 8) {
                memcpy(&row2_buf[16 - page_len], page_buf, page_len);
            }
        }

        LCD_I2C_WriteRow(1, row1_buf);
        LCD_I2C_WriteRow(2, row2_buf);
        LCD_I2C_Set_Cursor_Blink(2, safe_binary_count + 1, 1);
    }
}

/* Transmit Function (Continuous Locked Timeline) ----------------------------*/
void Transmit(uint8_t *data_array) {
    uint16_t t = (uint16_t)TIM2->CNT;

    // 1. PREAMBLE (100 toggles: Light ON, Light OFF)
    for (int i = 0; i < 100; i++) {
        LED_ON(); // Physical Light ON (Pin LOW)
        t += PREAMBLE_PERIOD_US;
        Wait_Until(t);

        LED_OFF(); // Physical Light OFF (Pin HIGH)
        t += PREAMBLE_PERIOD_US;
        Wait_Until(t);
    }

    // 2. LEADER PULSE (4x Bit Period Light ON)
    LED_ON();
    t += START_PULSE_US;
    Wait_Until(t);

    // 3. SYNC WORD: 0xA5 MSB-First
    uint8_t sync_word = 0xA5;
    for (int i = 0; i < 8; i++) {
        if ((sync_word >> (7 - i)) & 1) LED_ON(); else LED_OFF();
        t += BIT_PERIOD_US;
        Wait_Until(t);
    }

    // 4. 16-BIT PAYLOAD LENGTH HEADER (MSB-First)
    uint16_t payload_bytes = (count + 7) / 8;
    for (int i = 0; i < 16; i++) {
        if ((payload_bytes >> (15 - i)) & 1) LED_ON(); else LED_OFF();
        t += BIT_PERIOD_US;
        Wait_Until(t);
    }

    // 5. DATA PAYLOAD
    for (int i = 0; i < count; i++) {
        if ((data_array[i / 8] >> (i % 8)) & 1) LED_ON(); else LED_OFF();
        t += BIT_PERIOD_US;
        Wait_Until(t);
    }

    // 6. END MARKER (ASCII EOT 0x04)
    uint8_t end_marker = END_MARKER_BYTE;
    for (int b = 0; b < 8; b++) {
        if ((end_marker >> b) & 1) LED_ON(); else LED_OFF();
        t += BIT_PERIOD_US;
        Wait_Until(t);
    }

    LED_OFF(); // Return to dark Idle State (LED physically OFF)
}

/* Header Detection and Fixed Grid Sample Start ------------------------------*/
uint16_t Expect_Frame_Header(void) {
    uint16_t entry_time = (uint16_t)TIM2->CNT;
    uint32_t window_us = (uint32_t)PREAMBLE_PERIOD_US * 220 + 40000;
    if (window_us > 60000) window_us = 60000;

    while ((uint16_t)((uint16_t)TIM2->CNT - entry_time) < (uint16_t)window_us) {
        // 1. Wait for RX to be LOW if currently in a high pulse
        while (GPIOA->INDR & RX_Pin) {
            if ((uint16_t)((uint16_t)TIM2->CNT - entry_time) >= (uint16_t)window_us) return 0;
        }

        // 2. Wait for next Rising Edge (LOW -> HIGH)
        while (!(GPIOA->INDR & RX_Pin)) {
            if ((uint16_t)((uint16_t)TIM2->CNT - entry_time) >= (uint16_t)window_us) return 0;
        }

        // 3. Mark the Rising Edge timestamp
        uint16_t edge_start = (uint16_t)TIM2->CNT;

        // 4. Measure pulse width. If it holds for >= VALID_SYNC_THRESH_US, it is the Leader Pulse
        while (GPIOA->INDR & RX_Pin) {
            if ((uint16_t)((uint16_t)TIM2->CNT - edge_start) >= VALID_SYNC_THRESH_US) {
                
                // Align to center of Sync Word Bit 0 directly from Leader Pulse start
                uint16_t sample_time = edge_start + START_PULSE_US + BIT_CENTER_US;

                uint8_t received_sync = 0;
                for (int i = 0; i < 8; i++) {
                    Wait_Until(sample_time);
                    if (GPIOA->INDR & RX_Pin) {
                        received_sync |= (1 << (7 - i));
                    }
                    sample_time += BIT_PERIOD_US;
                }

                // Verify 0xA5 Sync Word with optical noise tolerance
                if (received_sync == 0xA5 || received_sync == 0xA4 || received_sync == 0xA1 || (received_sync & 0xF0) == 0xA0) {
                    return sample_time; // Sample center timestamp for Header Bit 0
                }
                return 0;
            }
        }
    }

    return 0;
}

/* Edge Phase Resynchronized Bit Sampler --------------------------------------*/
static uint8_t Sample_Bit_With_Resync(uint16_t *p_sample_time, uint8_t *p_last_raw_pin) {
    uint8_t edge_detected = 0;
    uint16_t edge_time = 0;
    uint8_t prev = *p_last_raw_pin;

    // Monitor for an optical edge transition up to the target sample center
    while ((int16_t)((uint16_t)TIM2->CNT - *p_sample_time) < 0) {
        uint8_t curr = (GPIOA->INDR & RX_Pin) ? 1 : 0;
        if (!edge_detected && curr != prev) {
            edge_detected = 1;
            edge_time = (uint16_t)TIM2->CNT;
            prev = curr;
        }
    }

    // Read bit value at center
    uint8_t bit_val = (GPIOA->INDR & RX_Pin) ? 1 : 0;
    *p_last_raw_pin = bit_val;

    // Resynchronize timeline if a transition occurred near the expected boundary
    if (edge_detected) {
        *p_sample_time = edge_time + BIT_PERIOD_US + BIT_CENTER_US;
    } else {
        *p_sample_time += BIT_PERIOD_US;
    }

    return bit_val;
}

/* Optical Frame Reception with Active Phase Tracking ------------------------*/
uint8_t Receive_And_Display(void) {
    static uint8_t temp_buffer[MAX_CHARS / 8];
    memset(temp_buffer, 0, sizeof(temp_buffer));

    uint16_t sample_time = Expect_Frame_Header();
    if (sample_time == 0) return 0; // Return immediately on noise or invalid sync

    uint8_t last_pin_state = (GPIOA->INDR & RX_Pin) ? 1 : 0;

    // 1. Read 16-Bit Payload Length Header (MSB-First) with Edge Resync
    uint16_t expected_bytes = 0;
    for (int b = 0; b < 16; b++) {
        uint8_t bit = Sample_Bit_With_Resync(&sample_time, &last_pin_state);
        if (bit) {
            expected_bytes |= (1 << (15 - b));
        }
    }

    if (expected_bytes == 0 || expected_bytes > (MAX_CHARS / 8)) {
        return 0;
    }

    // 2. Read Data Payload with Continuous Phase Realignment
    for (int byte_idx = 0; byte_idx < expected_bytes; byte_idx++) {
        uint8_t rx_byte = 0;
        for (int b = 0; b < 8; b++) {
            uint8_t bit = Sample_Bit_With_Resync(&sample_time, &last_pin_state);
            if (bit) {
                rx_byte |= (1 << b);
            }
        }
        temp_buffer[byte_idx] = rx_byte;
    }

    // 3. Skip End Marker
    for (int b = 0; b < 8; b++) {
        Sample_Bit_With_Resync(&sample_time, &last_pin_state);
    }

    int rx_bits = expected_bytes * 8;
    if (rx_bits > 0) {
        memcpy(bitHistory, temp_buffer, sizeof(bitHistory));
        count = rx_bits;
        ascii_count = count / 8;
        binary_count = 0;
        viewer_page = 0;
        viewer_mode = 1;
        return 1; // Valid frame decoded
    }

    return 0;
}

/* EEPROM Test Implementation ------------------------------------------------*/
void Run_EEPROM_Test(void) {
    LCD_I2C_ClearText();
    LCD_I2C_WriteText(1, 1, "EEPROM Testing..");
    
    uint8_t saved = 0;
    AT24C64_ReadByte(0x0000, &saved);
    
    AT24C64_WriteByte(0x0000, 0xAA);
    
    uint8_t read_val = 0;
    AT24C64_ReadByte(0x0000, &read_val);
    
    AT24C64_WriteByte(0x0000, saved);
    
    LCD_I2C_ClearText();
    if (read_val == 0xAA) {
        LCD_I2C_WriteText(1, 1, "EEPROM: PASS!");
        sprintf(buff, "R/W: 0x%02X OK", read_val);
        LCD_I2C_WriteText(2, 1, buff);
    } else {
        LCD_I2C_WriteText(1, 1, "EEPROM: FAIL!");
        sprintf(buff, "W:0xAA R:0x%02X", read_val);
        LCD_I2C_WriteText(2, 1, buff);
    }
    Delay_Ms(3000);
    
    menu_mode = 0;
    Redraw_Current_Page();
}