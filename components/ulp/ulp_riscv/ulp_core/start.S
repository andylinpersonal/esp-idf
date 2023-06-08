/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
	.section .text.vectors
	.global irq_vector
	.global reset_vector

/* The reset vector, jumps to startup code */
reset_vector:
	j __start

/* Shared interrupt handler */
.balign 16
irq_vector:
	j irq_handler

	.section .text

.type __start, %function
__start:
	/* setup the stack pointer */
	la sp, __stack_top
	call ulp_riscv_rescue_from_monitor
	call main
	call ulp_riscv_halt
loop:
	j loop
	.size __start, .-__start

.type default_irq_handler, %function
default_irq_handler:
	ret
	.size	default_irq_handler, .-default_irq_handler

.weak irq_handler
	.set irq_handler, default_irq_handler
