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
#include <sys/systm.h>

#include <dev/spi/spi.h>
#include <dev/gpio/gpio.h>
#include <dev/st7789v/st7789v.h>

#include "../fonts/draw.h"

#include "lcd.h"

#include <arch/riscv/gigadevice/gd32v_spi.h>
#include <arch/riscv/gigadevice/gd32v_dma.h>

extern struct mdx_device spi;
extern struct mdx_device gpiob;
extern struct mdx_device dma;

#define	dprintf(fmt, ...)

#define	SPI0_DATA	(0x40013000 + 0x0C)
#define	SPI0_CHAN	2

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

static uint8_t buffer[LCD_WIDTH * LCD_HEIGHT * LCD_BYTES / 2] __aligned(16);

static int
transfer_dma(mdx_device_t dev, uint8_t *out,
    uint8_t *in, uint32_t len)
{
	struct dma_desc desc;
	struct gd32v_spi_config conf;

	conf.master = true;
	conf.prescaler = 2;
	conf.ff16 = false;
	conf.dma_tx = true;
	conf.dma_rx = false;

	desc.src_addr = (uint32_t)out;
	desc.dst_addr = SPI0_DATA;
	desc.src_inc = 1;
	desc.dst_inc = 0;
	desc.src_width = 8;
	desc.dst_width = 8;
	desc.direction = DMA_MEM_TO_DEV;
	desc.count = len;

	gd32v_spi_setup(&spi, &conf);
	gd32v_dma_setup(&dma, SPI0_CHAN, &desc);

	return (0);
}

static void
lcd_delay(void)
{

}

static void
lcd_command(void)
{

	mdx_gpio_set(&gpiob, LCD_DC, 0);
	lcd_delay();
}

static void
lcd_data(void)
{

	mdx_gpio_set(&gpiob, LCD_DC, 1);
	lcd_delay();
}

static void
cs_enable(void)
{

	mdx_gpio_set(&gpiob, LCD_CS, 0);
	lcd_delay();
}

static void
cs_disable(void)
{

	mdx_gpio_set(&gpiob, LCD_CS, 1);
	lcd_delay();
}

static void
lcd_set_addr(int xs, int ys, int xe, int ye)
{
	uint8_t data[4];

	data[0] = ST7789V_CASET;
	lcd_command();
	transfer_dma(&spi, data, NULL, 1);

	data[0] = 0;
	data[1] = xs + 1;
	data[2] = 0;
	data[3] = xe;
	lcd_data();
	transfer_dma(&spi, data, NULL, 4);

	data[0] = ST7789V_RASET;
	lcd_command();
	transfer_dma(&spi, data, NULL, 1);

	data[0] = 0;
	data[1] = ys + 25 + 1;
	data[2] = 0;
	data[3] = ye + 25;
	lcd_data();
	transfer_dma(&spi, data, NULL, 4);

	data[0] = ST7789V_RAMWR;
	lcd_command();
	transfer_dma(&spi, data, NULL, 1);
	lcd_data();
}

static void
lcd_clear(void)
{
	struct gd32v_spi_config conf;
	struct dma_desc desc;
	uint32_t data;
	int len;

	data = 0x4f4f;

	lcd_set_addr(0, 0, LCD_WIDTH, LCD_HEIGHT);

	len = LCD_WIDTH * LCD_HEIGHT * LCD_BYTES;

	desc.src_addr = (uintptr_t)&data;
	desc.dst_addr = SPI0_DATA;
	desc.src_inc = 0;
	desc.dst_inc = 0;
	desc.src_width = 16;
	desc.dst_width = 16;
	desc.direction = DMA_MEM_TO_DEV;
	desc.count = len / 2;

	conf.master = true;
	conf.prescaler = 2;
	conf.ff16 = true;
	conf.dma_tx = true;
	conf.dma_rx = false;

	gd32v_spi_setup(&spi, &conf);
	gd32v_dma_setup(&dma, SPI0_CHAN, &desc);
}

static void
lcd_clear_buf(void)
{
	int len;
	int i;

	/* Half buffer */
	len = (LCD_WIDTH * LCD_HEIGHT * LCD_BYTES) / 2;

	for (i = 0; i < len; i += 1)
		buffer[i] = 0x4f;
}

static void
lcd_flush(int x, int y)
{
	struct gd32v_spi_config conf;
	struct dma_desc desc;
	int len;

	lcd_set_addr(x, y, LCD_WIDTH, y + (LCD_HEIGHT / 2));

	/* Half buffer */
	len = LCD_WIDTH * LCD_HEIGHT * LCD_BYTES / 2;

	conf.master = true;
	conf.prescaler = 2;
	conf.ff16 = true;
	conf.dma_tx = true;
	conf.dma_rx = false;

	desc.src_addr = (uint32_t)buffer;
	desc.dst_addr = SPI0_DATA;
	desc.src_inc = 1;
	desc.dst_inc = 0;
	desc.src_width = 16;
	desc.dst_width = 16;
	desc.direction = DMA_MEM_TO_DEV;
	desc.count = len / 2;

	gd32v_spi_setup(&spi, &conf);
	gd32v_dma_setup(&dma, SPI0_CHAN, &desc);
}

void
lcd_init(void)
{
	const uint8_t *p;
	uint8_t cmd;

	mdx_gpio_set(&gpiob, LCD_RST, 1);

	cs_enable();
	for (p = init_seq; *p != 0xff; p++) {
		cmd = *p++;
		dprintf("transmitting command %x\n", cmd);
		lcd_command();
		transfer_dma(&spi, &cmd, NULL, 1);
		if (*p == 0xff)
			continue;
		lcd_data();
		while(*p != 0xff) {
			cmd = *p++;
			dprintf("transmitting data %x\n", cmd);
			transfer_dma(&spi, &cmd, NULL, 1);
		}
	}
	lcd_clear();
	cs_disable();
}

void
lcd_update(int line, char *text)
{

	cs_enable();

	lcd_clear_buf();
	lvgl_draw(buffer, text);

	if (line == 0)
		lcd_flush(0, 0);
	else
		lcd_flush(0, LCD_HEIGHT / 2);

	cs_disable();
}
