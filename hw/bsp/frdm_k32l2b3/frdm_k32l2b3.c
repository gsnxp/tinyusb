/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2018, hathach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#include "../board.h"
#include "fsl_device_registers.h"
#include "fsl_gpio.h"
#include "fsl_clock.h"
#include "fsl_lpuart.h"

#include "./frdm_k32l2b3.h"
#include "clock_config.h"
#include "pin_mux.h"

// LED
// #define LED_PINMUX            IOMUXC_GPIO_11_GPIOMUX_IO11
// #define LED_PORT              GPIO1
// #define LED_PIN               11
// #define LED_STATE_ON          0

// Button
// #define BUTTON_PINMUX         IOMUXC_GPIO_SD_05_GPIO2_IO05
// #define BUTTON_PORT           GPIO2
// #define BUTTON_PIN            5
// #define BUTTON_STATE_ACTIVE   0

// UART
#define UART_PORT             LPUART0
// #define UART_RX_PINMUX        IOMUXC_GPIO_09_LPUART1_RXD
// #define UART_TX_PINMUX        IOMUXC_GPIO_10_LPUART1_TXD


const uint8_t dcd_data[] = { 0x00 };

void board_init(void)
{
  // Init pins
  BOARD_InitLEDsPins();
  BOARD_InitBUTTONSPins();
  BOARD_InitDEBUG_UARTPins();

  // Init clock
  BOARD_BootClockRUN();
  SystemCoreClockUpdate();

#if CFG_TUSB_OS == OPT_OS_NONE
  // 1ms tick timer
  SysTick_Config(SystemCoreClock / 1000);
#elif CFG_TUSB_OS == OPT_OS_FREERTOS
  // If freeRTOS is used, IRQ priority is limit by max syscall ( smaller is higher )
//  NVIC_SetPriority(USB0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY );
#endif

  // LED
  LED_RED_INIT(LOGIC_LED_ON);

  // UART
  lpuart_config_t uartConfig;
  CLOCK_SetLpuart0Clock(1);
  LPUART_GetDefaultConfig(&uartConfig);
  uartConfig.baudRate_Bps = BOARD_DEBUG_UART_BAUDRATE;
  uartConfig.enableTx     = true;
  uartConfig.enableRx     = true;

  LPUART_Init(UART_PORT, &uartConfig, BOARD_DEBUG_UART_CLK_FREQ);

  //------------- USB0 -------------//
  // Clock
    SystemCoreClockUpdate();
    CLOCK_EnableUsbfs0Clock(kCLOCK_UsbSrcIrc48M, 48000000U);
}

//--------------------------------------------------------------------+
// USB Interrupt Handler
//--------------------------------------------------------------------+
void USB0_IRQHandler(void)
{
  #if CFG_TUSB_RHPORT0_MODE & OPT_MODE_HOST
    tuh_isr(0);
  #endif

  #if CFG_TUSB_RHPORT0_MODE & OPT_MODE_DEVICE
    tud_isr(0);
  #endif
}

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state)
{
  if (state) {LED_RED_ON();}
  else {LED_RED_OFF();}
}

uint32_t board_button_read(void)
{
  // active low
  return ( BUTTON_STATE_ACTIVE == GPIO_PinRead(BOARD_SW1_GPIO, BOARD_SW1_GPIO_PIN) );
}

int board_uart_read(uint8_t* buf, int len)
{
  LPUART_ReadBlocking(UART_PORT, buf, len);
  return len;
}

int board_uart_write(void const * buf, int len)
{
  LPUART_WriteBlocking(UART_PORT, (uint8_t*)buf, len);
  return len;
}

#if CFG_TUSB_OS == OPT_OS_NONE
volatile uint32_t system_ticks = 0;
void SysTick_Handler(void)
{
  system_ticks++;
}

uint32_t board_millis(void)
{
  return system_ticks;
}
#endif
