/*-
 * Copyright (c) 2018 Ruslan Bukin <br@bsdpad.com>
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

#include <machine/asm.h>

	/* Jump to logical address to ensure correct operation of RAM region */
ASM_ENTRY(__start)
	la	a0, __start
	li	a1, 1
	slli	a1, a1, 29
	bleu	a1, a0, __start1
	srli	a1, a1, 2
	bleu	a1, a0, __start1
	la	a0, __start1
	add	a0, a0, a1
	jr	a0

ASM_ENTRY(__start1)
	la	sp, idle_thread_stack + MDX_THREAD_STACK_SIZE
	la	t0, cpu_exception_handler

	/*
	 * Use ECLIC. See
	 * Bumblebee Processor Core
	 * Instruction Set Architecture Manual
	 */
	or	t0, t0, 3
	csrw	mtvec, t0

	/* Clear BSS. */
	la	t0, _sbss
	la	t1, _ebss
1:
	beq	t0, t1, 2f
	li	t2, 0
	sb	t2, 0(t0)
	addi	t0, t0, 1
	j	1b
2:

	/* Relocate data */
	la	t0, _smem
	la	t1, _sdata
	la	t2, _edata
1:
	beq	t1, t2, 2f
	lb	t3, 0(t0)
	sb	t3, 0(t1)
	addi	t0, t0, 1
	addi	t1, t1, 1
	j	1b
2:

	li	a0, 0
	j	md_init

	/* NOT REACHED */
