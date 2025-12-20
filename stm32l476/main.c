/*
 * Copyright (c) 2019, Alex Taradov <alex@taradov.com>
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
#include "stm32l4xx.h"
#include "stm32l476xx.h"
#include "system_stm32l4xx.h"
#include "hal_gpio.h"

//-----------------------------------------------------------------------------
#define PERIOD_FAST     100
#define PERIOD_SLOW     500

//-----------------------------------------------------------------------------
// NUCLEO-L476RG pinouts
HAL_GPIO_PIN(LED2,    A, 5)   // PA.5  = Green LED2
HAL_GPIO_PIN(BUTTON,  C, 13)  // PC.13 = Blue user button B1
HAL_GPIO_PIN(UART_RX, A, 3)   // PA.3  = USART2_RX (AF7) Nucleo VCP
HAL_GPIO_PIN(UART_TX, A, 2)   // PA.2  = USART2_TX (AF7)  

//-----------------------------------------------------------------------------
static void timer_set_period(int i)
{
  TIM2->ARR = (F_CPU / 1000ul / 1000ul) * i;
  TIM2->CNT = 0;
}

//-----------------------------------------------------------------------------
void irq_handler_timer2(void)
{
  if (TIM2->SR & TIM_SR_UIF)
  {
    TIM2->SR &= ~TIM_SR_UIF;
    HAL_GPIO_LED2_toggle();
  }
}

//-----------------------------------------------------------------------------
static void timer_init(void)
{
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;

  TIM2->PSC = 999;

  timer_set_period(PERIOD_SLOW);

  TIM2->CR1 |= TIM_CR1_CEN;
  TIM2->DIER = TIM_DIER_UIE;

  NVIC_EnableIRQ(TIM2_IRQn);
}

//-----------------------------------------------------------------------------
static void uart_init(uint32_t baud)
{
  HAL_GPIO_UART_RX_alt(7);
  HAL_GPIO_UART_TX_alt(7);

  RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;

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
  RCC->AHB2ENR = RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN |
      RCC_AHB2ENR_GPIODEN | RCC_AHB2ENR_GPIOFEN;
  
  /* Enable FPU, set CP10 and CP11 Full Access */
  SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));
}

//-----------------------------------------------------------------------------
int main(void)
{
  uint32_t cnt = 0;
  bool fast = false;
  char rxch;

  /* sysclock is currently 4MHz MSI */
  MODIFY_REG(RCC->CR, RCC_CR_MSIRANGE, RCC_CR_MSIRANGE_8);
  /* sysclock is currently 16MHz MSI */

  /* switch to HSI 16MHz for better clock frequency accuracy */
  RCC->CR |= RCC_CR_HSION;
  while (!(RCC->CR & RCC_CR_HSIRDY));
  MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_HSI);
  /* sysclock is currently 16MHz HSI */

  SystemCoreClockUpdate();   /* calculate SystemCoreClock value (system_stm32l4xx.c) */

  sys_init();
  timer_init();
  uart_init(115200);

  uart_puts("\r\nHello, world!\r\n");

  HAL_GPIO_LED2_out();
  HAL_GPIO_LED2_set();

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
