# Optical Communicator Firmware (CH32V203 @ 144MHz)

* Build environment: Mounriver Studio
* MCU: CH32V203F8P6

# Pin mapping
| Pin | Role | Notes |
| :--- | :--- | :--- |
| `PA0` | `'0'` Button / Prev Page | Internal pull-up (`B1_Pin`) |
| `PA1` | `'1'` Button / Next Page | Internal pull-up (`B0_Pin`) |
| `PA2` | Backspace / Cancel / Clear | Internal pull-up (`B2_Pin`) |
| `PA3` | Transmit Button | Hold to transmit (`B4_Pin`) |
| `PA4` | Optical TX Output | Active-low push-pull via `BSHR`/`BCR` registers (matches inverting `MMBT3904` driver) |
| `PA5`, `PA8` | Rotary Encoder A / B | Quadrature menu & page navigation |
| `PA7` | Optical RX Input | Digital input with internal pull-down (`RX_Pin`) |
| `PA9`, `PA10` | Software I2C (SDA / SCL) | Open-drain 50 MHz for 16x2 LCD & `AT24C64` EEPROM |
| `PB0` | Encoder Push Button | Menu enter / select |

# Protocol


* **1 µs Hardware Timebase (`TIM2`):** `TIM2` runs continuously at 1 MHz (`1 µs` per tick) using rollover-safe signed timestamp comparisons (`Wait_Until()`) for jitter-free bit timing.
* **Active-Low Direct Register TX:** `Transmit()` drives `PA4` directly via `GPIOA->BCR` and `GPIOA->BSHR` to minimize GPIO latency and match the inverting `MMBT3904` + `IRF540` laser stage (`PA4` LOW = Laser ON).
* **Frame Format:**
  1. **Preamble:** 100 ON/OFF bit-period toggles to settle the receiver analog front-end.
  2. **Leader Pulse:** `4 × bit_period` continuous ON pulse (validated in `Expect_Frame_Header()` if held `>= 2.5 × bit_period`).
  3. **Sync Word:** `0xA5` (MSB-first, with minor bit-error tolerance).
  4. **Length Header:** 16-bit payload byte count (MSB-first).
  5. **Payload:** Up to 8,192 bits (1,024 bytes).
  6. **End Marker:** `0x04` (ASCII EOT).
* **Active Edge-Phase Resynchronization (`Sample_Bit_With_Resync`):** Rather than relying on an open-loop timer that drifts over long messages, the receiver monitors for `PA7` edge transitions prior to each bit center and dynamically realigns the next sample window (`edge_time + 1.5 × bit_period`).
* **Configurable Bit Rate:** `g_bit_period_us` defaults to `10 µs` (`100 kb/s`) and can be adjusted in the UI from `2 µs` to `10,000 µs` with settings persisted to EEPROM.

# UI & EEPROM Storage (AT24C64)
* **Live Binary-to-ASCII Entry:** Users enter raw bits via the `'0'`/`'1'` buttons; every 8 bits automatically decodes to an ASCII character on Row 1 of the 16x2 LCD while uncommitted bits appear on Row 2.
* **Persistent Storage:** Custom `AT24C64` layout stores magic header (`MSG\x01`), configured bit period, and up to **8 named message slots** (save, load, delete, full erase, and self-test via the rotary encoder menu).
