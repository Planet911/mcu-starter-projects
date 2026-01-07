/*
 * Copyright (c) 2020, Alex Taradov <alex@taradov.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

//-----------------------------------------------------------------------------
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "stm32l053xx.h"
#include "hal_gpio.h"

//-----------------------------------------------------------------------------
#define PERIOD_FAST     100
#define PERIOD_SLOW     500

// STM32L053R8 Nucleo64 board
HAL_GPIO_PIN(LED,      A, 5)
HAL_GPIO_PIN(BUTTON,   C, 13)
HAL_GPIO_PIN(UART_TX,  A, 2)
HAL_GPIO_PIN(UART_RX,  A, 3)

//-----------------------------------------------------------------------------
static void timer_set_period(int i)
{
  TIM2->ARR = (F_CPU / 1000ul / 1000ul) * i;
  TIM2->CNT = 0;
}

//-----------------------------------------------------------------------------
void irq_handler_tim2(void)
{
  if (TIM2->SR & TIM_SR_UIF)
  {
    TIM2->SR &= ~TIM_SR_UIF;
    HAL_GPIO_LED_toggle();
  }
}

//-----------------------------------------------------------------------------
static void timer_init(void)
{
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

  TIM2->PSC = 999;

  timer_set_period(PERIOD_SLOW);

  TIM2->CR1 |= TIM_CR1_CEN;
  TIM2->DIER = TIM_DIER_UIE;

  NVIC_EnableIRQ(TIM2_IRQn);
}

//-----------------------------------------------------------------------------
static void uart_init(uint32_t baud)
{
  HAL_GPIO_UART_RX_alt(4);
  HAL_GPIO_UART_TX_alt(4);

  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

  USART2->CR1 = USART_CR1_RE | USART_CR1_TE;
  USART2->BRR = (F_CPU / baud);
  USART2->CR1 |= USART_CR1_UE;
}

//-----------------------------------------------------------------------------
static void uart_putc(char c)
{
  while (0 == (USART2->ISR & USART_ISR_TXE));
  USART2->TDR = c;
}

//-----------------------------------------------------------------------------
static bool uart_getc(char *c)
{
  if (USART2->ISR & USART_ISR_RXNE)
  {
    *c = USART2->RDR;
    return true;
  }

  return false;
}

//-----------------------------------------------------------------------------
static void uart_puts(char *s)
{
  while (*s)
    uart_putc(*s++);
}

//-----------------------------------------------------------------------------
static char invert_case(char c)
{
  if ('a' <= c && c <= 'z')
    return c + ('A' - 'a');
  else if ('A' <= c && c <= 'Z')
    return c - ('A' - 'a');
  return c;
}

//-----------------------------------------------------------------------------
static void sys_init(void)
{
  // Internal Voltage Regulator Operating Range
  PWR->CR = (PWR->CR & ~PWR_CR_VOS) | PWR_CR_VOS_0; /* b01: 1.8 V (range 1) no wait-states needed at 16MHz */
  //PWR->CR = (PWR->CR & ~PWR_CR_VOS) | PWR_CR_VOS_1; /* b10: 1.5 V (range 2) no wait-states needed at 8MHz */

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
  //RCC->CFGR |= RCC_CFGR_PPRE1_2 | RCC_CFGR_PPRE1_0; // APB1 prescaler PCLK1 (includes USB peripheral)
 
  RCC->CFGR &= ~RCC_CFGR_PPRE2;
  RCC->CFGR |= RCC_CFGR_PPRE2_2 | RCC_CFGR_PPRE2_0; // APB2 prescaler PCLK2

  // Clock enable for ports and peripherals in run mode
  RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN | RCC_IOPENR_GPIOCEN | RCC_IOPENR_GPIODEN | RCC_IOPENR_GPIOHEN;
  RCC->AHBENR |= RCC_AHBENR_DMA1EN | RCC_AHBENR_MIFEN | RCC_AHBENR_CRCEN;
  RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_SPI2EN | RCC_APB1ENR_USART2EN;
  RCC->APB2ENR |= RCC_APB2ENR_ADCEN | RCC_APB2ENR_SPI1EN | RCC_APB2ENR_SYSCFGEN;

  // Clocks to remain enabled in sleep mode
  RCC->IOPSMENR |= RCC_IOPSMENR_GPIOASMEN | RCC_IOPSMENR_GPIOBSMEN | RCC_IOPSMENR_GPIOCSMEN;
  RCC->AHBSMENR |= RCC_AHBSMENR_MIFSMEN | RCC_AHBSMENR_SRAMSMEN;
  RCC->APB2SMENR |= RCC_APB2SMENR_SPI1SMEN;
  RCC->APB1SMENR |= RCC_APB1SMENR_SPI2SMEN | RCC_APB1SMENR_LPTIM1SMEN;

  /* Peripheral clock sources
   * By default all peripheral clocks are derived from the APB clock (optional HSI,LSE or SYSCLK)
   */
  RCC->CCIPR = 0;
}

//-----------------------------------------------------------------------------
int main(void)
{
  uint32_t cnt = 0;
  bool fast = false;
  char rxch;

  sys_init();
  timer_init();
  uart_init(115200);

  uart_puts("\r\nHello, world!\r\n");

  HAL_GPIO_LED_out();
  HAL_GPIO_LED_set();

  HAL_GPIO_BUTTON_in();
  HAL_GPIO_BUTTON_pullup();

  while (1)
  {
    if (HAL_GPIO_BUTTON_read())
      cnt = 0;
    else if (cnt < 5001)
      cnt++;

    if (5000 == cnt)
    {
      fast = !fast;
      timer_set_period(fast ? PERIOD_FAST : PERIOD_SLOW);
      uart_putc('.');
    }

    if (uart_getc(&rxch))
    {
      uart_putc(fast ? invert_case(rxch) : rxch);
    }
  }

  return 0;
}

