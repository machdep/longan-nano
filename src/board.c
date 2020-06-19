/*-
 * Copyright (c) 2020 Ruslan Bukin <br@bsdpad.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
#include <sys/console.h>
#include <sys/of.h>
#include <sys/thread.h>
#include <sys/malloc.h>

#include <machine/cpufunc.h>

#include <dev/gpio/gpio.h>
#include <dev/uart/uart.h>
#include <dev/intc/intc.h>

#include <arch/riscv/include/clic.h>
#include <arch/riscv/gigadevice/gd32v.h>

#include "lcd.h"

static struct mdx_device rcu;
static struct mdx_device gpioa;
struct mdx_device gpiob;
static struct mdx_device usart;
static struct mdx_device timer0;
static struct mdx_device clic;
struct mdx_device dma;
struct mdx_device i2c0;
struct mdx_device spi;

#define	CCS811_SCL	6	/* Port B */
#define	CCS811_SDA	7	/* Port B */

void
board_init(void)
{
	uint32_t reg;

	/* Add some memory so OF could allocate devices and their softc. */
	mdx_fl_init();
	mdx_fl_add_region(0x20007800, 0x800);

	gd32v_rcu_init(&rcu, BASE_RCU);
	gd32v_rcu_setup(&rcu,
	    AHBEN_DMA0EN,
	    APB1EN_TIMER1EN | APB1EN_I2C0EN,
	    APB2EN_USART0EN | APB2EN_TIMER0EN | APB2EN_PAEN |
	    APB2EN_PBEN | APB2EN_SPI0EN | APB2EN_AFEN);

	gd32v_gpio_init(&gpioa, BASE_GPIOA);
	gd32v_gpio_init(&gpiob, BASE_GPIOB);

	gd32v_usart_init(&usart, BASE_USART0, 8000000);

	mdx_uart_setup(&usart, 115200, UART_DATABITS_8,
	    UART_STOPBITS_1, UART_PARITY_NONE);

	mdx_console_register_uart(&usart);

	gd32v_timer_init(&timer0, BASE_TIMER0, 8000000);
	gd32v_i2c_init(&i2c0, BASE_I2C0);
	gd32v_dma_init(&dma, BASE_DMA0);

	mdx_gpio_configure(&gpioa, 0, 9,
	    MDX_GPIO_OUTPUT | MDX_GPIO_ALT_FUNC |
	    MDX_GPIO_SPEED_LOW | MDX_GPIO_PUSH_PULL);

	/* CCS811 */
	reg = MDX_GPIO_OUTPUT | MDX_GPIO_SPEED_MEDIUM | MDX_GPIO_PUSH_PULL;
	mdx_gpio_configure(&gpioa, 0, 12, reg); //wak
	mdx_gpio_configure(&gpioa, 0, 11, reg); //int
	mdx_gpio_configure(&gpioa, 0, 8, reg); //rst
	mdx_gpio_configure(&gpiob, 0, 15, reg); //add

	mdx_gpio_set(&gpioa, 0, 12, 0);
	mdx_gpio_set(&gpioa, 0, 11, 0);
	mdx_gpio_set(&gpioa, 0, 8, 0);
	mdx_gpio_set(&gpiob, 0, 15, 0);

#if 0
	mdx_gpio_configure(&gpiob, 0, 6,
	    MDX_GPIO_ALT_FUNC | MDX_GPIO_INPUT | MDX_GPIO_PULL_UP);
	mdx_gpio_configure(&gpiob, 0, 7,
	    MDX_GPIO_ALT_FUNC | MDX_GPIO_INPUT | MDX_GPIO_PULL_UP);
#endif

	reg = MDX_GPIO_OUTPUT | MDX_GPIO_SPEED_HIGH;
	reg |= MDX_GPIO_OPEN_DRAIN | MDX_GPIO_ALT_FUNC;
	mdx_gpio_configure(&gpiob, 0, CCS811_SCL, reg);
	mdx_gpio_configure(&gpiob, 0, CCS811_SDA, reg);

	/* LCD */
	reg = MDX_GPIO_OUTPUT | MDX_GPIO_ALT_FUNC | MDX_GPIO_SPEED_HIGH | MDX_GPIO_PUSH_PULL;
	mdx_gpio_configure(&gpioa, 0, LCD_MOSI, reg);
	mdx_gpio_configure(&gpioa, 0, LCD_SCK, reg);

	reg = MDX_GPIO_OUTPUT | MDX_GPIO_SPEED_MEDIUM | MDX_GPIO_PUSH_PULL;
	mdx_gpio_configure(&gpiob, 0, LCD_DC, reg);
	mdx_gpio_configure(&gpiob, 0, LCD_RST, reg);
	mdx_gpio_configure(&gpiob, 0, LCD_CS, reg);

	gd32v_spi_init(&spi, BASE_SPI0);

	clic_init(&clic, BASE_ECLIC);

	/* DMA0 channel2 global interrupt */
	mdx_intc_setup(&clic, 32, gd32v_dma_intr, &dma);
	mdx_intc_enable(&clic, 32);

	mdx_intc_setup(&clic, 46, gd32v_timer_intr, &timer0);
	mdx_intc_enable(&clic, 46);

	mdx_intc_setup(&clic, 50, gd32v_i2c_event_intr, &i2c0);
	mdx_intc_enable(&clic, 50);

	mdx_intc_setup(&clic, 51, gd32v_i2c_error_intr, &i2c0);
	mdx_intc_enable(&clic, 51);

	printf("mdepx initialized\n");
}
