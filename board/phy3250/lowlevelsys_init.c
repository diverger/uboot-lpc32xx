/*
 * Copyright (C) 2008 by NXP Semiconductors
 * All rights reserved.
 *
 * @Author: Kevin Wells
 * @Descr: Phytec 3250 board support functions
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

#include <common.h>
#include <configs/phy3250.h>
#include <asm/mach-types.h>
#include <lpc3250.h>
#include <net.h>
#include "phy3250_prv.h"

PHY_HW_T phyhwdesc;

static int ssp_read(void *buffer,
			int max_fifo)
{
	volatile unsigned long tmp1;
	int count = 0;
	u_char *data8 = (u_char *) buffer;

	while ((max_fifo > 0) && ((SSP0->sr & SSP_SR_RNE) != 0))
	{
		tmp1 = SSP0->data;
		*data8 = (u_char) tmp1;
		data8++;

		/* Increment data count and decrement buffer size count */
		count++;
		max_fifo--;
	}

	return count;
}

static int ssp_write(void *buffer,
                 int n_fifo)
{
	int count = 0;
	u_char *data8 = (u_char *) buffer;

	/* Loop until transmit ring buffer is full or until n_bytes
	   expires */
	while ((n_fifo > 0) && ((SSP0->sr & SSP_SR_TNF) != 0))
	{
		SSP0->data = (unsigned long) * data8;
		data8++;

		/* Increment data count and decrement buffer size count */
		count++;
		n_fifo--;
	}

	return count;
}

static void ssp_set_clock(unsigned long target_clock)
{
	unsigned long control, prescale, cr0_div, cmp_clk, ssp_clk;

	/* Get clock for SSP */
	ssp_clk = sys_get_rate(CLKPWR_HCLK);

	/* Find closest divider to get at or under the target frequency.
	   Use smallest prescaler possible and rely on the divider to get
	   the closest target frequency */
	cr0_div = 0;
	cmp_clk = 0xFFFFFFFF;
	prescale = 2;
	while (cmp_clk > target_clock)
	{
		cmp_clk = ssp_clk / ((cr0_div + 1) * prescale);
		if (cmp_clk > target_clock)
		{
			cr0_div++;
			if (cr0_div > 0xFF)
			{
				cr0_div = 0;
				prescale += 2;
			}
		}
	}

	/* Write computed prescaler and divider back to register */
	control = SSP0->cr0 &= ~(SSP_CR0_SCR(0xFF));
	SSP0->cr0 = control | SSP_CR0_SCR(cr0_div - 1);
	SSP0->cpsr = prescale;
}

/*
 * Initialize SSP0 for interface to the serial EEPROM
 */
static void phy3250_ssp0_init(void)
{
	volatile unsigned long tmp;

	/* Enable SSP0 clock */
	CLKPWR->clkpwr_ssp_blk_ctrl |= CLKPWR_SSPCTRL_SSPCLK0_EN;

	SSP0->cr0 = (SSP_CR0_DSS(8) | SSP_CR0_FRF_SPI);
	SSP0->cr1 |= SSP_CR1_SSP_ENABLE;

	/* Empty FIFO */
	while ((SSP0->sr & SSP_SR_RNE) != 0)
	{
		tmp = SSP0->data;
	}

	ssp_set_clock(500000);

	/* Clear latched interrupts */
	SSP0->icr = (SSP_ICR_RORIC | SSP_ICR_RTIC);

	/* Disable interrupts */
	SSP0->imsc = 0; // TBD (SSP_IMSC_RORIM | SSP_IMSC_RTIM | SSP_IMSC_RXIM);
}

static int phy3250_sspxfer(u_char *out,
                               u_char *in,
                               int bytes)
{
  int rbytes = 0, xfrd = 0;

	/* Assert chip select */
	GPIO->p3_outp_clr = OUTP_STATE_GPIO(5);
	ssp_write(out, bytes);
	while (rbytes < bytes)
	{
		rbytes += ssp_read(&in [rbytes], 1);
	}

	GPIO->p3_outp_set = OUTP_STATE_GPIO(5);
	xfrd = 1;

	return xfrd;
}

static u_char phy3250_sspread(int index)
{
	u_char datai [8], datao [8];
	u_char byte = 0;

	/* Read byte */
	datao [0] = SEEPROM_READ;
	datao [1] = (u_char)((index >> 8) & 0xFF);
	datao [2] = (u_char)((index >> 0) & 0xFF);
	datao [3] = 0xFF;
	phy3250_sspxfer(datao, datai, 4);
	byte = datai [3];

	return byte;
}

void phy3250_get_board_info(void)
{
	u_char *p8;
	int idx;

	/* Initialize SSP0 */
	phy3250_ssp0_init();

	/* Read data from EEPROM - this needs to be done here as the
	   SDRAM configuration depends on these settings. */
	p8 = (u_char *) & phyhwdesc;
	for (idx = 0; idx < sizeof(phyhwdesc); idx++)
	{
		*p8 = phy3250_sspread(PHY3250_SEEPROM_CFGOFS + idx);
		p8++;
	}

	if (phyhwdesc.fieldvval != PHY_HW_VER_VAL)
	{
		/* Set some defaults */
		phyhwdesc.dramcfg = (PHYHW_DRAM_TYPE_LPSDRAM | PHYHW_DRAM_SIZE_64M);
		phyhwdesc.syscfg = PHYHW_SDIO_POP;
		phyhwdesc.fieldvval = PHY_HW_VER_VAL;

		/* Default MAC address in index order of 0:1:2:3:4:5 */
		phyhwdesc.mac [0] = 0x00;
		phyhwdesc.mac [1] = 0x01;
		phyhwdesc.mac [2] = 0x90;
		phyhwdesc.mac [3] = 0x00;
		phyhwdesc.mac [4] = 0xC0;
		phyhwdesc.mac [5] = 0x81;
		phyhwdesc.mac [6] = 0x00;
		phyhwdesc.mac [7] = 0x00;

	}
}

/*
 * Miscellaneous platform dependent initializations
 */
void phy3250_sys_init(void)
{
	/* Get board information to determine DRAM size and MAC address */
	phy3250_get_board_info();

#ifndef CFG_BOOT_USES1L
	/* Initialize system including clocking, SDRAM, muxing, MMU, etc. */
	/* TBD stubbed */
#endif
}

