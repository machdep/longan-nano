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

#include <dev/spi/spi.h>
#include <dev/gpio/gpio.h>
#include <dev/st7789v/st7789v.h>

#include "lcd.h"


extern struct mdx_device spi;
extern struct mdx_device gpiob;

static const uint8_t init_seq[] = {
	0x21, 0xff,
	0xb1, 0x05, 0x3a, 0x3a, 0xff,
	0xb2, 0x05, 0x3a, 0x3a, 0xff,
	0xb3, 0x05, 0x3a, 0x3a, 0x05, 0x3a, 0x3a, 0xff,
	0xb4, 0x03, 0xff,
	0xc0, 0x62, 0x02, 0x04, 0xff,
	0xc1, 0xc0, 0xff,
	0xc2, 0x0d, 0x00, 0xff,
	0xc3, 0x8d, 0x6a, 0xff,
	0xc4, 0x8d, 0xee, 0xff,
	0xc5, 0x0e, 0xff,
	0xe0, 0x10, 0x0e, 0x02, 0x03, 0x0e, 0x07, 0x02, 0x07, 0x0a, 0x12, 0x27, 0x37, 0x00, 0x0d, 0x0e, 0x10, 0xff,
	0xe1, 0x10, 0x0e, 0x03, 0x03, 0x0f, 0x06, 0x02, 0x08, 0x0a, 0x13, 0x26, 0x36, 0x00, 0x0d, 0x0e, 0x10, 0xff,
	0x3a, 0x55, 0xff,
	0x36, 0x78, 0xff,
	0x29, 0xff,
	0x11, 0xff,
	0xff
};

static void
lcd_delay(void)
{
	int i;

	for (i = 0; i < 10000; i++)
		;
}

static void
lcd_command(void)
{

	mdx_gpio_set(&gpiob, 0, LCD_DC, 0);
	lcd_delay();
}

static void
lcd_data(void)
{

	mdx_gpio_set(&gpiob, 0, LCD_DC, 1);
	lcd_delay();
}

static void
cs_enable(void)
{

	mdx_gpio_set(&gpiob, 0, LCD_CS, 0);
	lcd_delay();
}

static void
cs_disable(void)
{

	mdx_gpio_set(&gpiob, 0, LCD_CS, 1);
	lcd_delay();
}

void
lcd_init(void)
{
	const uint8_t *p;
	uint8_t cmd;

	mdx_gpio_set(&gpiob, 0, LCD_RST, 1);

	cs_enable();

	for (p = init_seq; *p != 0xff; p++) {
		cmd = *p++;
		//printf("transmitting command %x\n", cmd);
		lcd_command();
		mdx_spi_transfer(&spi, &cmd, NULL, 1);
		if (*p == 0xff)
			continue;
		lcd_data();
		while(*p != 0xff) {
			cmd = *p++;
			//printf("transmitting data %x\n", cmd);
			mdx_spi_transfer(&spi, &cmd, NULL, 1);
		}
	}

	uint8_t data[5];

	data[0] = ST7789V_CASET;
	lcd_command();
	mdx_spi_transfer(&spi, data, NULL, 1);

	data[0] = 0;
	data[1] = 1;
	data[2] = 0;
	data[3] = 160;
	lcd_data();
	mdx_spi_transfer(&spi, data, NULL, 4);

	data[0] = ST7789V_RASET;
	lcd_command();
	mdx_spi_transfer(&spi, data, NULL, 1);

	data[0] = 0;
	data[1] = 26;
	data[2] = 0;
	data[3] = 80+25;
	lcd_data();
	mdx_spi_transfer(&spi, data, NULL, 4);

	data[0] = ST7789V_RAMWR;
	lcd_command();
	mdx_spi_transfer(&spi, data, NULL, 1);

	lcd_data();
	int i;
	for (i = 0; i < 160*80; i++) {
		data[0] = 0x4f;
		data[1] = 0x4f;
		mdx_spi_transfer(&spi, data, NULL, 2);
	}

	cs_disable();
}
