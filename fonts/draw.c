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

#include <lib/lvgl/src/lv_font/lv_font.h>
#include <lib/lvgl/src/lv_font/lv_font_fmt_txt.h>

#include "draw.h"

#define dprintf(fmt, ...)

extern lv_font_t tahoma_40;

static void
draw_pixel(uint8_t *ptr, lv_font_glyph_dsc_t *g, int y, int x, int pixel)
{
	uint16_t *buf;

	buf = (uint16_t *)ptr;

	if (pixel)
		buf[(x + y * LCD_WIDTH)] = 0;
	else
		buf[(x + y * LCD_WIDTH)] = 0x4f4f;
}

static void
lvgl_letter(uint8_t *buf, lv_font_t *font_p,
    lv_font_glyph_dsc_t *g, const uint8_t *map_p)
{
	uint32_t bitmask_init;
	uint32_t bitmask;
	uint32_t col_bit_max;
	uint32_t col_bit_row_ofs;
	uint32_t bit_ofs;
	uint32_t col_bit;
	uint32_t bpp;
	int32_t col, row;
	int32_t width_bit;
	int32_t row_start, col_start;
	int32_t row_end, col_end;
	uint8_t letter_px;

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
			draw_pixel(buf, g, row, col, letter_px);

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

void
lvgl_draw(uint8_t *buf, char *z)
{
	lv_font_glyph_dsc_t g;
	const uint8_t *map_p;
	uint8_t *ptr;
	lv_font_t *font_p;
	bool g_ret;
	int x;
	int y;
	int i;

	font_p = &tahoma_40;

	x = 4;
	y = 4;

	ptr = buf + (x + (y * LCD_WIDTH)) * LCD_BYTES;

	for (i = 0; i < strlen(z); i++) {
		g_ret = lv_font_get_glyph_dsc(font_p, &g, z[i], '\0');
		if (g_ret == false)
			panic("error");

		dprintf("g.box_h %d %d\n", g.box_h, g.box_w);

		map_p = lv_font_get_glyph_bitmap(font_p, z[i]);
		if (map_p == NULL)
			panic("error 1");

		lvgl_letter(ptr, font_p, &g, map_p);

		x += g.box_w;
		ptr += g.box_w * LCD_BYTES;
	}
}
