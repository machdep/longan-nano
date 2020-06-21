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

#include "lcd.h"
#include <lib/lvgl/src/lv_font/lv_font.h>
#include <lib/lvgl/src/lv_font/lv_font_fmt_txt.h>

#include <arch/riscv/gigadevice/gd32v_spi.h>
#include <arch/riscv/gigadevice/gd32v_dma.h>

extern struct mdx_device spi;
extern struct mdx_device gpiob;
extern struct mdx_device dma;
extern lv_font_t tahoma_40;

#define	dprintf(fmt, ...)

#define	LCD_BYTES	2

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

static struct global_data {
	uint8_t buffer[LCD_WIDTH * LCD_HEIGHT * LCD_BYTES / 2];
	uint8_t *ptr;
} g_data __aligned(16);

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
draw_pixel(lv_font_glyph_dsc_t *g, int y, int x, int pixel)
{
	uint16_t *buf;

	buf = g_data.ptr;

	if (pixel)
		buf[(x + y * LCD_WIDTH)] = 0;
	else
		buf[(x + y * LCD_WIDTH)] = 0x4f4f;
}

static void
lcd_clear(void)
{
	struct gd32v_spi_config conf;
	struct dma_desc desc;
	uint16_t data;
	int len;

	data = 0x4f4f;

	lcd_set_addr(0, 0, LCD_WIDTH, LCD_HEIGHT);

	len = LCD_WIDTH * LCD_HEIGHT * LCD_BYTES;

	desc.src_addr = &data;
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
		g_data.buffer[i] = 0x4f;
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

	desc.src_addr = (uint32_t)g_data.buffer;
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

static void
lvgl_letter(lv_font_t *font_p, lv_font_glyph_dsc_t *g, const uint8_t *map_p)
{
	uint32_t bitmask_init;
	uint32_t bitmask;
	uint32_t col_bit_max;
	uint32_t col_bit_row_ofs;
	uint32_t bit_ofs;
	uint32_t col_bit;
	uint32_t bpp;
	int32_t pos_x, pos_y;
	int32_t col, row;
	int32_t width_bit;
	int32_t row_start, col_start;
	int32_t row_end, col_end;
	uint8_t letter_px;
	int x, y;

	bpp = g->bpp;
	if (bpp == 3)
		bpp = 4;

	switch(bpp) {
	case 1:
		bitmask_init = 0x80;
		break;
	case 2:
		bitmask_init = 0xC0;
		break;
	case 4:
		bitmask_init = 0xF0;
		break;
	case 8:
		bitmask_init = 0xFF;
		break;
	default:
		panic("invalid bpp");
	}

	x = 0;
	y = 0;

	pos_x = x + g->ofs_x;
	pos_y = y + (font_p->line_height - font_p->base_line)
	    - g->box_h - g->ofs_y;

	dprintf("pos_x %d pos_y %d\n", pos_x, pos_y);

	width_bit = g->box_w * bpp; /* Letter width in bits. */

	/* Calculate the col/row start/end on the map. */
	col_start = 0;
	row_start = 0;
	col_end = g->box_w;
	row_end = g->box_h;

	/* Move on the map too. */
	bit_ofs = (row_start * width_bit) + (col_start * bpp);
	map_p += bit_ofs >> 3;

	dprintf("row_end %d col_end %d\n", row_end, col_end);

	col_bit = bit_ofs & 0x7;
	col_bit_max = 8 - bpp;
	col_bit_row_ofs = (g->box_w + col_start - col_end) * bpp;

	for (row = row_start ; row < row_end; row++) {
		bitmask = bitmask_init >> col_bit;

		for (col = col_start; col < col_end; col++) {
			letter_px = (*map_p & bitmask) >>
			    (col_bit_max - col_bit);
			draw_pixel(g, row, col, letter_px);

			/* Go to the next column. */
			if (col_bit < col_bit_max) {
				col_bit += bpp;
				bitmask = bitmask >> bpp;
			} else {
				col_bit = 0;
				bitmask = bitmask_init;
				map_p++;
			}
		}

		col_bit += col_bit_row_ofs;
		map_p += (col_bit >> 3);
		col_bit = col_bit & 0x7;
	}
}

static void
lvgl_draw(char *z)
{
	lv_font_glyph_dsc_t g;
	const uint8_t *map_p;
	lv_font_t *font_p;
	bool g_ret;
	int x;
	int y;
	int i;

	font_p = &tahoma_40;

	x = 4;
	y = 4;

	g_data.ptr = (uint8_t *)g_data.buffer;
	g_data.ptr += (x + (y * LCD_WIDTH)) * LCD_BYTES;

	for (i = 0; i < strlen(z); i++) {
		g_ret = lv_font_get_glyph_dsc(font_p, &g, z[i], '\0');
		if (g_ret == false)
			panic("error");

		dprintf("g.box_h %d %d\n", g.box_h, g.box_w);

		map_p = lv_font_get_glyph_bitmap(font_p, z[i]);
		if (map_p == NULL)
			panic("error 1");

		lvgl_letter(font_p, &g, map_p);

		x += g.box_w;
		g_data.ptr += g.box_w * LCD_BYTES;
	}
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
lcd_update(int val)
{
	char text[16];

	sprintf(text, "%d", val);

	cs_enable();
	lcd_clear_buf();
	lvgl_draw(text);
	lcd_flush(0, 0);
	cs_disable();

	sprintf(text, "Co2(ppm)", val);

	cs_enable();
	lcd_clear_buf();
	lvgl_draw(text);
	lcd_flush(0, LCD_HEIGHT / 2);
	cs_disable();
}
