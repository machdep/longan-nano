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

#include <dev/gpio/gpio.h>
#include <dev/uart/uart.h>

#include <arch/riscv/gigadevice/gd32v.h>

static struct mdx_device rcu;
static struct mdx_device gpioa;
static struct mdx_device usart;

#if 0
static struct clint_softc clint_sc;
#endif

void
board_init(void)
{

	/* Add some memory so OF could allocate devices and their softc. */
	mdx_fl_init();
	mdx_fl_add_region(0x20004000, 0x4000);

#if 0
	e300g_clint_init(&clint_sc, CLINT_BASE, BOARD_OSC_FREQ);
#endif

	gd32v_rcu_init(&rcu, BASE_RCU);
	gd32v_rcc_setup(&rcu, 0, 0, APB2EN_USART0 | APB2EN_PAEN);

	gd32v_gpio_init(&gpioa, BASE_GPIOA);
	gd32v_usart_init(&usart, BASE_USART0, 8000000);

	mdx_uart_setup(&usart, 115200, UART_DATABITS_8,
	    UART_STOPBITS_1, UART_PARITY_NONE);

	mdx_console_register_uart(&usart);

	mdx_gpio_configure(&gpioa, 0, 9,
	    MDX_GPIO_OUTPUT | MDX_GPIO_ALT_FUNC |
	    MDX_GPIO_SPEED_LOW | MDX_GPIO_PUSH_PULL);

	printf("mdepx initialized\n");

}
