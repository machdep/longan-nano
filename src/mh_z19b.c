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

#include <dev/mh_z19b/mh_z19b.h>
#include <dev/uart/uart.h>

#include "mh_z19b.h"

extern struct mdx_device usart1;

static void
drain_fifo(void)
{
	bool ready;

	for (;;) {
		ready = mdx_uart_rxready(&usart1);
		if (!ready)
			break;
		mdx_uart_getc(&usart1);
	}
}

int
mh_z19b_init(void)
{
	uint8_t reply[9];
	uint8_t req[9];

	printf("%s\n", __func__);

	drain_fifo();
	mh_z19b_set_range_req(req, 2000);
	mh_z19b_cycle(&usart1, req, reply, 2);

	return (0);
}

int
mh_z19b_read_data(uint32_t *co2)
{
	uint8_t reply[9];
	uint8_t req[9];
	int error;

	drain_fifo();
	mh_z19b_read_co2_req(req);
	mh_z19b_cycle(&usart1, req, reply, 9);

	error = mh_z19b_read_co2_reply(reply, co2);
	if (error) {
		printf("Failed to read CO2 data, error %d\n",
		    error);
		return (MDX_ERROR);
	}

	return (0);
}
