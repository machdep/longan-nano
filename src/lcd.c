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
extern uint32_t sfont;
extern lv_font_t tahoma_40;

#define	LCD_BYTES	2

const uint8_t _lv_bpp1_opa_table[2]  = {0, 255};          /*Opacity mapping with bpp = 1 (Just for compatibility)*/
const uint8_t _lv_bpp2_opa_table[4]  = {0, 85, 170, 255}; /*Opacity mapping with bpp = 2*/

const uint8_t _lv_bpp3_opa_table[8]  = {0, 36,  73, 109,   /*Opacity mapping with bpp = 3*/
                                        146, 182,  219, 255
                                       };

const uint8_t _lv_bpp4_opa_table[16] = {0,  17, 34,  51,  /*Opacity mapping with bpp = 4*/
                                        68, 85, 102, 119,
                                        136, 153, 170, 187,
                                        204, 221, 238, 255
                                       };
const uint8_t _lv_bpp8_opa_table[256] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
                                         16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
                                         32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47,
                                         48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63,
                                         64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79,
                                         80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95,
                                         96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111,
                                         112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127,
                                         128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143,
                                         144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159,
                                         160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175,
                                         176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191,
                                         192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207,
                                         208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223,
                                         224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239,
                                         240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255
                                        };

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
	uint8_t buffer[LCD_WIDTH * LCD_HEIGHT * LCD_BYTES];
	uint8_t *ptr;
	//struct font_info font;
} g_data __aligned(16);

#define	SPI0_DATA	(0x40013000 + 0x0C)
#define	SPI0_CHAN	2

static int
mdx_spi_transfer2(mdx_device_t dev, uint8_t *out,
    uint8_t *in, uint32_t len)
{
	struct dma_desc desc;

	if (1 == 1) {
		desc.src_addr = (uint32_t)out;
		desc.dst_addr = SPI0_DATA;
		desc.src_inc = 1;
		desc.dst_inc = 0;
		desc.src_width = 8;
		desc.dst_width = 8;
		desc.direction = DMA_MEM_TO_DEV;
		desc.count = len;

		/* dma */
		gd32v_spi_test(&spi, 8);
		gd32v_dma_setup(&dma, SPI0_CHAN, &desc);
	} else {
		mdx_spi_transfer(dev, out, in, len);
	}

	return (0);
}

static void
lcd_delay(void)
{
	//int i;

	//for (i = 0; i < 100000; i++)
	//	;
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
	mdx_spi_transfer2(&spi, data, NULL, 1);

	data[0] = 0;
	data[1] = xs + 1;
	data[2] = 0;
	data[3] = xe;
	lcd_data();
	mdx_spi_transfer2(&spi, data, NULL, 4);

	data[0] = ST7789V_RASET;
	lcd_command();
	mdx_spi_transfer2(&spi, data, NULL, 1);

	data[0] = 0;
	data[1] = ys + 25 + 1;
	data[2] = 0;
	data[3] = ye + 25;
	lcd_data();
	mdx_spi_transfer2(&spi, data, NULL, 4);

	data[0] = ST7789V_RAMWR;
	lcd_command();
	mdx_spi_transfer2(&spi, data, NULL, 1);
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
lcd_clear_buf(void)
{
	int i;

#if 0
	for (i = 0; i < LCD_WIDTH * LCD_HEIGHT * LCD_BYTES; i += 2) {
		g_data.buffer[i] = 0x4f;
		g_data.buffer[i + 1] = 0;
	}
	return;
#endif

	for (i = 0; i < LCD_WIDTH * LCD_HEIGHT * LCD_BYTES; i += 1)
		g_data.buffer[i] = 0x4f;
}

static void
lcd_flush(void)
{
	struct dma_desc desc;
	int len;

	lcd_set_addr(0, 0, LCD_WIDTH, LCD_HEIGHT);

	len = LCD_WIDTH * LCD_HEIGHT * LCD_BYTES;

	if (1 == 0) {
		mdx_spi_transfer(&spi, g_data.buffer, NULL, len);
	} else {
		desc.src_addr = (uint32_t)g_data.buffer;
		desc.dst_addr = SPI0_DATA;
		desc.src_inc = 1;
		desc.dst_inc = 0;
		desc.src_width = 16;
		desc.dst_width = 16;
		desc.direction = DMA_MEM_TO_DEV;
		desc.count = len / 2;

		gd32v_spi_test(&spi, 16);
		gd32v_dma_setup(&dma, SPI0_CHAN, &desc);
	}
}

static void
lvgl_letter(lv_font_t *font_p, lv_font_glyph_dsc_t *g, const uint8_t *map_p)
{
	const uint8_t * bpp_opa_table_p;
	uint32_t bpp;

	uint32_t bitmask_init;
	uint32_t bitmask;
	//uint32_t shades;

	bpp = g->bpp;
	if (bpp == 3)
		bpp = 4;

	switch(bpp) {
	case 1:
		bpp_opa_table_p = _lv_bpp1_opa_table;
		bitmask_init = 0x80;
		//shades = 2;
		break;
	case 2:
		bpp_opa_table_p = _lv_bpp2_opa_table;
		bitmask_init = 0xC0;
		//shades = 4;
		break;
	case 4:
		bpp_opa_table_p = _lv_bpp4_opa_table;
		bitmask_init = 0xF0;
		//shades = 16;
		break;
	case 8:
		bpp_opa_table_p = _lv_bpp8_opa_table;
		bitmask_init = 0xFF;
		//shades = 256;
		break;
	default:
		panic("invalid bpp");
	}

	int32_t pos_x;
	int32_t pos_y;
	int32_t col, row;
	int32_t box_w;
	int32_t box_h;
	int32_t width_bit;
	int32_t row_start, col_start;
	int32_t row_end, col_end;
	uint32_t bit_ofs;

	int x, y;
	x = 0;
	y = 0;

	pos_x = x + g->ofs_x;
	pos_y = y + (font_p->line_height - font_p->base_line)
	    - g->box_h - g->ofs_y;

	if (1 == 0)
		printf("pos_x %d pos_y %d\n", pos_x, pos_y);

	box_w = g->box_w;
	box_h = g->box_h;

	width_bit = box_w * bpp; /* Letter width in bits. */

	/* Calculate the col/row start/end on the map. */
	col_start = 0;
	row_start = 0;
	col_end = box_w;
	row_end = box_h;

	/* Move on the map too. */
	bit_ofs = (row_start * width_bit) + (col_start * bpp);
	map_p += bit_ofs >> 3;

	//printf("row_end %d col_end %d\n", row_end, col_end);

	uint32_t col_bit;
	uint8_t letter_px;

	col_bit = bit_ofs & 0x7; /* "& 0x7" equals to "% 8" just faster */

	//int32_t mask_p_start;
	uint32_t col_bit_max;
	uint32_t col_bit_row_ofs;

	col_bit_max = 8 - bpp;
	col_bit_row_ofs = (box_w + col_start - col_end) * bpp;

	for (row = row_start ; row < row_end; row++) {
		//mask_p_start = mask_p;
		bitmask = bitmask_init >> col_bit;

		for (col = col_start; col < col_end; col++) {
			/* Load the pixel's opacity into the mask. */
			letter_px = (*map_p & bitmask) >>
			    (col_bit_max - col_bit);
			if (letter_px) {
				//mask_buf[mask_p] = bpp_opa_table_p[letter_px];
			} else {
				//mask_buf[mask_p] = 0;
			}
			//printf("%d/%d: %x\n", row, col, letter_px);
			draw_pixel(g, row, col, bpp_opa_table_p[letter_px]);

			/* Go to the next column. */
			if (col_bit < col_bit_max) {
				col_bit += bpp;
				bitmask = bitmask >> bpp;
			} else {
				col_bit = 0;
				bitmask = bitmask_init;
				map_p++;
			}

			/* Next mask byte. */
			//mask_p++;
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
	int x;
	int i;

	font_p = &tahoma_40;

	g_data.ptr = (uint8_t *)g_data.buffer;
	g_data.ptr += (4 + (4 * LCD_WIDTH)) * LCD_BYTES;
	x = 4;
	bool g_ret;

	for (i = 0; i < strlen(z); i++) {
		g_ret = lv_font_get_glyph_dsc(font_p, &g, z[i], '\0');
		if (g_ret == false)
			panic("error");

		//printf("g.box_h %d %d\n", g.box_h, g.box_w);

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
		//printf("transmitting command %x\n", cmd);
		lcd_command();
		mdx_spi_transfer2(&spi, &cmd, NULL, 1);
		if (*p == 0xff)
			continue;
		lcd_data();
		while(*p != 0xff) {
			cmd = *p++;
			//printf("transmitting data %x\n", cmd);
			mdx_spi_transfer2(&spi, &cmd, NULL, 1);
		}
	}
	cs_disable();
}

void
lcd_update(int val)
{
	char text[16];

	sprintf(text, "%d ppm", val);

	if (1 == 1) {
		lcd_clear_buf();
		lvgl_draw(text);
	}

	cs_enable();
	lcd_flush();
	cs_disable();
}
