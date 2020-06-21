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
#include <sys/callout.h>
#include <sys/systm.h>
#include <sys/malloc.h>
#include <sys/thread.h>
#include <sys/mbuf.h>

#include <dev/i2c/i2c.h>
#include <dev/ccs811/ccs811.h>

#include "ccs811.h"

extern struct mdx_device i2c0;

static uint8_t
ccs811_read(uint8_t reg, uint8_t *buf, int len)
{
	struct i2c_msg msgs[2];
	uint8_t i2c_addr;
	int error;

	i2c_addr = 0x5a;

	/* Write register */
	msgs[0].slave = i2c_addr;
	msgs[0].buf = &reg;
	msgs[0].len = 1;
	msgs[0].flags = 0;

	/* Read data */
	msgs[1].slave = i2c_addr;
	msgs[1].buf = buf;
	msgs[1].len = len;
	msgs[1].flags = IIC_M_RD;

	error = mdx_i2c_transfer(&i2c0, msgs, 2);

	return (error);
}

static int
ccs811_write(uint8_t reg, uint8_t *val, int len)
{
	struct i2c_msg msgs[2];
	uint8_t buf[16];
	uint8_t i2c_addr;
	int error;

	i2c_addr = 0x5a;

	buf[0] = reg;
	memcpy(&buf[1], val, len);

	/* Write register */
	msgs[0].slave = i2c_addr;
	msgs[0].buf = buf;
	msgs[0].len = 1 + len;
	msgs[0].flags = 0;

	error = mdx_i2c_transfer(&i2c0, msgs, 1);

	return (error);
}

int
ccs811_init(void)
{
	uint8_t val;
	int error;

	error = ccs811_read(CCS811_STATUS, &val, 1);
	if (error) {
		printf("%s: error %d\n", __func__, error);
		return (MDX_ERROR);
	}
	printf("status %x\n", val);

	error = ccs811_write(CCS811_APP_START, 0, 0);
	if (error) {
		printf("%s: error %d\n", __func__, error);
		return (MDX_ERROR);
	}

	val = (1 << 4);
	error = ccs811_write(CCS811_MEAS_MODE, &val, 1);
	if (error) {
		printf("%s: error %d\n", __func__, error);
		return (MDX_ERROR);
	}

	error = ccs811_read(CCS811_STATUS, &val, 1);
	if (error) {
		printf("%s: error %d\n", __func__, error);
		return (MDX_ERROR);
	}
	printf("hello status %x\n", val);

	error = ccs811_read(CCS811_MEAS_MODE, &val, 1);
	if (error) {
		printf("%s: error %d\n", __func__, error);
		return (MDX_ERROR);
	}
	printf("hello measmode %x\n", val);

	return (0);
}

int
ccs811_read_data(uint16_t *eco2, uint16_t *tvoc)
{
	uint8_t data[8];
	int error;

	error = ccs811_read(CCS811_ALG_RESULT_DATA, data, 8);
	if (error) {
		printf("%s: error %d\n", __func__, error);
		return (MDX_ERROR);
	}

	*eco2 = data[0] << 8 | data[1];
	*tvoc = data[2] << 8 | data[3];

	return (0);
}
