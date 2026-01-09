// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2025, Alex Taradov <alex@taradov.com>. All rights reserved.

/*- Includes ----------------------------------------------------------------*/
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "stm32l053xx.h"
#include "hal_gpio.h"
#include "usb.h"
#include "uart.h"

/*- Definitions -------------------------------------------------------------*/
HAL_GPIO_PIN(VCP_STATUS,       A, 5)

#define USB_BUFFER_SIZE        64
#define UART_WAIT_TIMEOUT      10 // ms
#define STATUS_TIMEOUT         150 // ms

/*- Variables ---------------------------------------------------------------*/
static uint64_t app_system_time = 0;
static uint64_t app_status_timeout = 0;

static alignas(4) uint8_t app_req_buf[USB_BUFFER_SIZE];
static alignas(4) uint8_t app_resp_buf[USB_BUFFER_SIZE];
static bool app_resp_free = true;

static alignas(4) uint8_t app_req_buf_hid[USB_BUFFER_SIZE];
static int app_req_buf_hid_size = 0;

static alignas(4) uint8_t app_req_buf_bulk[USB_BUFFER_SIZE];
static int app_req_buf_bulk_size = 0;

//cdc vcp
static alignas(4) uint8_t app_recv_buffer[USB_BUFFER_SIZE];
static alignas(4) uint8_t app_send_buffer[USB_BUFFER_SIZE];
static int app_recv_buffer_size = 0;
static int app_recv_buffer_ptr = 0;
static int app_send_buffer_ptr = 0;
static bool app_send_buffer_free = true;
static bool app_send_zlp = false;
static uint64_t app_uart_timeout = 0;
static uint64_t app_break_timeout = 0;
static bool app_vcp_event = false;
static bool app_vcp_open = false;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static void sys_init(void)
{
  // Internal Voltage Regulator Operating Range
  PWR->CR = (PWR->CR & ~PWR_CR_VOS) | PWR_CR_VOS_0; /* b01: 1.8 V (range 1) no wait-states needed at 16MHz */
  // PWR->CR = (PWR->CR & ~PWR_CR_VOS) | PWR_CR_VOS_1; /* b10: 1.5 V (range 2) no wait-states needed at 8MHz */

  /* LATENCY (wait-states)
   * For details search Ref Manual for "number of wait states".
   * Default after reset is 0 wait states.
   */
  // SET_BIT(FLASH->ACR, FLASH_ACR_LATENCY);

  /* PRFTEN
   * Setting this bit to 1 (with DISAB_BUF to 0) enables the prefetch. When the memory
   * interface does not have any operation in progress, the address following the last
   * address fetched is read and stored in a buffer.
   * instruction pre-fetch helps only if wait states > 0, otherwise not needed
   */
  // FLASH->ACR |= FLASH_ACR_PRFTEN;

  /* PRE_READ (for data, similar to PRE-FETCH for code)
   * Setting this bit to 1 (with DISAB_BUF to 0) enables the pre-read. When the memory
   * interface does not have any operation in progress or prefetch to execute, the address
   * following the last data address is read and stored in a buffer.
   */
  FLASH->ACR |= FLASH_ACR_PRE_READ;

  // Switch to HSI 16MHz for SYSCLK
  RCC->CR |= RCC_CR_HSION;
  while (0 == (RCC->CR & RCC_CR_HSIRDY));
  RCC->CFGR = ((RCC->CFGR & (~RCC_CFGR_SW)) | RCC_CFGR_SW_HSI);
  while ( ( RCC->CFGR & RCC_CFGR_SWS ) >> RCC_CFGR_SWS_Pos != RCC_CFGR_SW_HSI );

  /* AHB prescaler
   * 0xxx: SYSCLK not divided
   * 1000: SYSCLK divided by 2
   * 1001: SYSCLK divided by 4
   * 1010: SYSCLK divided by 8
   * 1011: SYSCLK divided by 16
   * 1100: SYSCLK divided by 64
   * 1101: SYSCLK divided by 128
   * 1110: SYSCLK divided by 256
   * 1111: SYSCLK divided by 512
   * RCC_CFGR_HPRE_3 | RCC_CFGR_HPRE_2 | RCC_CFGR_HPRE_1 | RCC_CFGR_HPRE_0;
   */
  RCC->CFGR &= ~RCC_CFGR_HPRE; // AHB prescaler = SYSCLK not divided
  // RCC->CFGR |= RCC_CFGR_HPRE_3 | RCC_CFGR_HPRE_0;  // AHB prescaler = SYSCLK divided by 4

  /* APB1 (PCLK1) and APB2 (PCLK2) clock dividers fom HCLK source
   * 0xx: HCLK not divided
   * 100: HCLK divided by 2
   * 101: HCLK divided by 4
   * 110: HCLK divided by 8
   * 111: HCLK divided by 16
   */
  RCC->CFGR &= ~RCC_CFGR_PPRE1;
  // RCC->CFGR |= RCC_CFGR_PPRE1_2 | RCC_CFGR_PPRE1_0; // APB1 prescaler PCLK1 (includes USB peripheral)
 
  RCC->CFGR &= ~RCC_CFGR_PPRE2;
  RCC->CFGR |= RCC_CFGR_PPRE2_2 | RCC_CFGR_PPRE2_0; // APB2 prescaler PCLK2

  // Clock enable for ports and peripherals in run mode
  RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN | RCC_IOPENR_GPIOCEN | RCC_IOPENR_GPIODEN | RCC_IOPENR_GPIOHEN;
  RCC->AHBENR |= RCC_AHBENR_DMA1EN | RCC_AHBENR_MIFEN | RCC_AHBENR_CRCEN;
  RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_SPI2EN | RCC_APB1ENR_USART2EN;
  RCC->APB2ENR |= RCC_APB2ENR_ADCEN | RCC_APB2ENR_SPI1EN | RCC_APB2ENR_SYSCFGEN;

  // Clocks to remain enabled in sleep mode
  RCC->IOPSMENR |= RCC_IOPSMENR_GPIOASMEN | RCC_IOPSMENR_GPIOBSMEN | RCC_IOPSMENR_GPIOCSMEN;
  // RCC->AHBSMENR |= RCC_AHBSMENR_MIFSMEN | RCC_AHBSMENR_SRAMSMEN;
  // RCC->APB2SMENR |= RCC_APB2SMENR_SPI1SMEN;
  // RCC->APB1SMENR |= RCC_APB1SMENR_SPI2SMEN | RCC_APB1SMENR_LPTIM1SMEN;

  /* Peripheral clock sources
   * By default all peripheral clocks are derived from the APB clock (optional HSI,LSE or SYSCLK)
   */
  RCC->CCIPR = 0;
}

//-----------------------------------------------------------------------------
static void serial_number_init(void)
{
  volatile uint8_t *uid = (volatile uint8_t *)UID_BASE;
  uint32_t sn = 5381;

  for (int i = 0; i < 12; i++)
    sn = ((sn << 5) + sn) ^ uid[i];

  for (int i = 0; i < 8; i++)
    usb_serial_number[i] = "0123456789ABCDEF"[(sn >> (i * 4)) & 0xf];

  usb_serial_number[8] = 0;
}

//-----------------------------------------------------------------------------
static void sys_time_init(void)
{
  SysTick->VAL  = 0;
  SysTick->LOAD = (F_CPU / 8) / 1000ul;
  SysTick->CTRL = SysTick_CTRL_ENABLE_Msk;
  app_system_time = 0;
}

//-----------------------------------------------------------------------------
static void sys_time_task(void)
{
  if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
    app_system_time++;
}

//-----------------------------------------------------------------------------
static void tx_task(void)
{
  while (app_recv_buffer_size)
  {
    if (!uart_write_byte(app_recv_buffer[app_recv_buffer_ptr]))
      break;

    app_recv_buffer_ptr++;
    app_recv_buffer_size--;
    app_vcp_event = true;

    if (0 == app_recv_buffer_size)
      usb_cdc_recv(app_recv_buffer, sizeof(app_recv_buffer));
  }
}

//-----------------------------------------------------------------------------
static void send_buffer(void)
{
  app_send_buffer_free = false;
  app_send_zlp = (USB_BUFFER_SIZE == app_send_buffer_ptr);

  usb_cdc_send(app_send_buffer, app_send_buffer_ptr);

  app_send_buffer_ptr = 0;
}

//-----------------------------------------------------------------------------
static void rx_task(void)
{
  int byte;

  if (!app_send_buffer_free)
    return;

  while (uart_read_byte(&byte))
  {
    int state = (byte >> 8) & 0xff;

    app_uart_timeout = app_system_time + UART_WAIT_TIMEOUT;
    app_vcp_event = true;

    if (state)
    {
      usb_cdc_set_state(state);
    }
    else
    {
      app_send_buffer[app_send_buffer_ptr++] = byte;

      if (USB_BUFFER_SIZE == app_send_buffer_ptr)
      {
        send_buffer();
        break;
      }
    }
  }
}

//-----------------------------------------------------------------------------
static void break_task(void)
{
  if (app_break_timeout && app_system_time > app_break_timeout)
  {
    uart_set_break(false);
    app_break_timeout = 0;
  }
}

//-----------------------------------------------------------------------------
static void uart_timer_task(void)
{
  if (app_uart_timeout && app_system_time > app_uart_timeout)
  {
    if (app_send_zlp || app_send_buffer_ptr)
      send_buffer();

    app_uart_timeout = 0;
  }
}

//-----------------------------------------------------------------------------
void usb_cdc_line_coding_updated(usb_cdc_line_coding_t *line_coding)
{
  uart_init(line_coding);
}

//-----------------------------------------------------------------------------
void usb_cdc_control_line_state_update(int line_state)
{
  bool status = line_state & USB_CDC_CTRL_SIGNAL_DTE_PRESENT;

  app_vcp_open        = status;
  app_send_buffer_ptr = 0;
  app_uart_timeout    = 0;
  app_break_timeout   = 0;

  if (app_vcp_open)
    uart_init(usb_cdc_get_line_coding());
  else
    uart_close();
}

//-----------------------------------------------------------------------------
void usb_cdc_send_break(int duration)
{
  if (USB_CDC_BREAK_DURATION_DISABLE == duration)
  {
    app_break_timeout = 0;
    uart_set_break(false);
  }
  else if (USB_CDC_BREAK_DURATION_INFINITE == duration)
  {
    app_break_timeout = 0;
    uart_set_break(true);
  }
  else
  {
    app_break_timeout = app_system_time + duration;
    uart_set_break(true);
  }
}

//-----------------------------------------------------------------------------
void usb_cdc_send_callback(void)
{
  app_send_buffer_free = true;
}

//-----------------------------------------------------------------------------
void usb_cdc_recv_callback(int size)
{
  app_recv_buffer_ptr = 0;
  app_recv_buffer_size = size;
}

//-----------------------------------------------------------------------------
void usb_hid_send_callback(void)
{
  app_resp_free = true;
}

//-----------------------------------------------------------------------------
void usb_hid_recv_callback(int size)
{
  app_req_buf_hid_size = size;
}

//-----------------------------------------------------------------------------
static void usb_bulk_send_callback(void)
{
  app_resp_free = true;
}

//-----------------------------------------------------------------------------
static void usb_bulk_recv_callback(int size)
{
  app_req_buf_bulk_size = size;
}

//-----------------------------------------------------------------------------
static void loopback_task(void)
{
  int interface, size;

  if (!app_resp_free)
    return;

  if (app_req_buf_hid_size)
  {
    app_vcp_event = true; // fake vcp event to blink activity led

    interface = USB_INTF_HID;
    size = app_req_buf_hid_size;
    app_req_buf_hid_size = 0;

    memcpy(app_req_buf, app_req_buf_hid, size);

    usb_hid_recv(app_req_buf_hid, sizeof(app_req_buf_hid));
  }
  else if (app_req_buf_bulk_size)
  {
    app_vcp_event = true; // fake vcp event to blink activity led

    interface = USB_INTF_BULK;
    size = app_req_buf_bulk_size;
    app_req_buf_bulk_size = 0;

    memcpy(app_req_buf, app_req_buf_bulk, size);

    usb_recv(USB_BULK_EP_RECV, app_req_buf_bulk, sizeof(app_req_buf_bulk));
  }
  else
  {
    return;
  }

  //size = process_request(app_req_buf, size, app_resp_buf, sizeof(app_resp_buf));
  //size = size; for loopback example, response size same as received size
  memcpy(app_resp_buf, app_req_buf, size);

  if (USB_INTF_BULK == interface)
    usb_send(USB_BULK_EP_SEND, app_resp_buf, size);
  else
    usb_hid_send(app_resp_buf, sizeof(app_resp_buf));

  app_resp_free = false;
}

//-----------------------------------------------------------------------------
void usb_configuration_callback(int config)
{
  app_resp_free = true;
  app_req_buf_hid_size = 0;
  app_req_buf_bulk_size = 0;

  usb_set_send_callback(USB_BULK_EP_SEND, usb_bulk_send_callback);
  usb_set_recv_callback(USB_BULK_EP_RECV, usb_bulk_recv_callback);

  usb_hid_recv(app_req_buf_hid, sizeof(app_req_buf_hid));
  usb_recv(USB_BULK_EP_RECV, app_req_buf_bulk, sizeof(app_req_buf_bulk));

#ifdef USB_CONFIG_ENABLE_VCP
  usb_cdc_recv(app_recv_buffer, sizeof(app_recv_buffer));
  app_send_buffer_free = true;
  app_send_buffer_ptr = 0;
#endif

  (void)config;
}

//-----------------------------------------------------------------------------
static void status_timer_task(void)
{
  if (app_system_time < app_status_timeout)
    return;

  app_status_timeout = app_system_time + STATUS_TIMEOUT;

  if (app_vcp_event)
    HAL_GPIO_VCP_STATUS_toggle();
  else
    HAL_GPIO_VCP_STATUS_write(app_vcp_open);

  app_vcp_event = false;
}

//-----------------------------------------------------------------------------
int main(void)
{
  sys_init();
  sys_time_init();
  usb_init();
  usb_hid_init();
  usb_cdc_init();
  serial_number_init();

  app_status_timeout = STATUS_TIMEOUT;

  HAL_GPIO_VCP_STATUS_out();
  HAL_GPIO_VCP_STATUS_clr();

  while (1)
  {
    sys_time_task();
    status_timer_task();
    usb_task();
    tx_task();
    rx_task();
    break_task();
    uart_timer_task();
    loopback_task();
  }

  return 0;
}
