/*
 * Copyright (C) 2008 by NXP Semiconductors
 * All rights reserved.
 *
 * @Author: Kevin Wells
 * @Descr: Phytec 3250 board low level init code
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */


#include <config.h>
#include <version.h>

#define IRAM_STK_TEMP 0x0003C000
.globl phy3250_sys_init
.globl dcache_kill

.globl lowlevel_init
lowlevel_init:
	/*
	 * Setup basic stack and save return address
	 */
	ldr	sp, =IRAM_STK_TEMP
	stmia	sp, {lr}

	/*
	 * Initialize Phytec 3250 board
	 */
	bl	phy3250_sys_init

	ldmia	sp, {r0}
	mov	pc, r0

dcache_kill:
	/*
	 * The data cache may have been enabled when booting from
	 * S1l, so disable it now to prevent problems.
	 */
flush1:
	mrc	p15, 0, r15, c7, c10, 3
	bne	flush1
	mcr	p15, 0, r0, c1, c0, 0
	bic	r0, r0, #4
	mrc	p15, 0, r0, c1, c0, 0
	mov	pc, lr

