# STM32F405 CAN Bus Thermal Node with FreeRTOS & Touch UI

A multi-tasking embedded system implemented on the STM32F405RGT6 microcontroller. Powered by FreeRTOS (CMSIS_V2), this project samples the MCU's internal temperature sensor, manages dual CAN Bus telemetry, and drives an ST7789 display with a resistive touch interface. It also implements a low-power firmware design via hardware sleep states.

## System Architecture & OS Tasks
The application runs deterministically under FreeRTOS, utilizing an inter-task communication queue (`TempQueue`) to pass telemetry data without blocking execution loops:

1. **Temp_Task (Priority: Normal):** Wakes up every 1000ms to poll the internal ADC temperature channel (`ADC_CHANNEL_TEMPSENSOR`). Computes the core temperature and pushes the float value into `TempQueue`.
2. **UI_Task (Priority: Normal):** Consumes temperature logs from the queue, formats the strings, and updates the ST7789 LCD over high-speed SPI. It handles localized display area refreshes to avoid screen flickering.
3. **CAN_Task / Touch_Task (Priority: Normal):** Scans the resistive touch screen controller by dynamically adjusting SPI prescalers (switching between touch sensing and high-speed LCD rendering). It decodes user actions (`PLAY` / `PAUSE` coordinates) to trigger CAN Bus transmission or put the MCU into Deep Sleep.

---

## Hardware Configuration & Pin Mapping

### Peripherals Base Clock
* **Main System Clock:** 168 MHz (via 8MHz External Crystal PLL)
* **APB1 Peripheral Clock:** 42 MHz (Timers: 84 MHz)
* **APB2 Peripheral Clock:** 84 MHz (Timers: 168 MHz)

### IO & Peripheral Matrix

| Peripheral / Mode | Pin Name | Hardware / Signal Name | Description / Configuration |
| :--- | :---: | :--- | :--- |
| **ADC1 (Analog)** | Internal | `ADC_CHANNEL_TEMPSENSOR` | Internal thermal sensor sampling (12-bit Resolution) |
| **CAN1 (Primary)** | PA11 / PA12 | `CAN1_RX` / `CAN1_TX` | Bitrate: **500 kbps** (Prescaler=6, BS1=11TQ, BS2=2TQ). Identifiers routing to FIFO0. |
| **CAN2 (Secondary)**| PB12 / PB13 | `CAN2_RX` / `CAN2_TX` | Bitrate: **500 kbps**. Filter boundary bank at 14 for loopback verification. |
| **SPI1 (Master)** | PB3 / PA6 / PA7| `SCK` / `MISO` / `MOSI` | Shared Bus. LCD Mode: 42 Mbps (PSC=2) / Touch Mode: ~650 kbps (PSC=128) |
| **GPIO Output** | PB10 / PB6 | `LCD_RST` / `LCD_BL` | Display Reset & Backlight PWM Control |
| | PB7 / PB8 | `LCD_CS` / `LCD_DC` | Display Chip Select & Data/Command Toggle |
| | PB9 | `TP_CS` | Touch Panel SPI Chip Select Pin |
| **GPIO Input** | PB4 | `TP_IRQ` | Touch Panel Data Ready Interrupt Request |
| **EXTI (Interrupt)**| PA0 | `WKUP / EXTI0` | Configured as Falling Edge Trigger to wake up MCU from Sleep |

---

## Technical Implementation Details

* **Shared SPI Bus Prescaler Switching:** To safely share the SPI1 bus between the high-speed ST7789 display controller and the low-speed touch sensor, the firmware uses a `vTaskSuspendAll()` critical section. It dynamically rewrites the SPI Baud Rate Prescaler registers directly before and after transactions to safely handle different hardware speed requirements.
* **Dual CAN Mailbox Loopback & Verification:** Implemented an on-chip dual CAN validation mechanism. Core temperature values are mirrored into 4-byte byte arrays via `memcpy` and broadcasted through CAN1 (StdId `0x123`). The secondary interface, CAN2, intercepts the frame using global acceptance filters, decodes it, and passes validation logs back to the user UI.
* **Low-Power WFI Sleep Implementation:** Pressing the `PAUSE` button triggers a safe power-down sequence: halts the HAL system tick (`HAL_SuspendTick`), disables the FreeRTOS scheduler heartbeats (`SysTick`), clears stale EXTI pending flags, and executes the `HAL_PWR_EnterSLEEPMode` using the **WFI (Wait For Interrupt)** instruction. The system instantly resumes its full RTOS operational context when a falling edge is caught on pin `PA0`.
* **Physical Touch De-noising:** Filters out false touch triggers and erratic hardware electrical fluctuations by discarding maximum/minimum RAW ADC boundary values ($0$ and $\ge 4095$) directly inside the coordinates collection routine.
