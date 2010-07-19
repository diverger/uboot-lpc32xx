/*
 * Copyright (C) 2008 by NXP Semiconductors
 * All rights reserved.
 * 
 * @Author: Kevin Wells
 * @Descr: LPC3250 DMA controller interface support functions
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
#include <lpc3250.h>
#include <asm/io.h>


/* Some optimization stuff */
#ifndef unlikely
#define likely(x)	__builtin_expect(!!(x), 1)
#define unlikely(x)	__builtin_expect(!!(x), 0)
#endif

#define DMA_CLK_ENABLE      1
/**********************************************************************
* DMA controller register structures
**********************************************************************/
static uint32_t alloc_ch;

int lpc32xx_dma_get_channel(void)
{
	int i;
	uint32_t status = 0;

	if (!alloc_ch) { /* First time caller */
		CLKPWR->clkpwr_dmaclk_ctrl |= DMA_CLK_ENABLE;
		/* Make sure DMA controller and all channels are disabled.
		*        Controller is in little-endian mode. Disable sync signals */
		dma_base->config = 0;
		dma_base->sync = 0;

		/* Clear interrupt and error statuses */
		dma_base->int_tc_clear = 0xFF;
		dma_base->raw_tc_stat = 0xFF;
		dma_base->int_err_clear = 0xFF;
		dma_base->raw_err_stat = 0xFF;

		/* Enable DMA controller */
		dma_base->config = DMAC_CTRL_ENABLE;
	}

	for (i = 0; i < DMA_NO_OF_CHANNELS && (status & _BIT(i)); i++)
	       ;

	/* Check if all the available channles are busy */
	if (unlikely(i == DMA_NO_OF_CHANNELS)) return -1;
	alloc_ch |= _BIT(i);
	return i;
}

int lpc32xx_dma_start_xfer(int channel, const dmac_ll_t *desc, uint32_t config)
{
	if (unlikely((_BIT(channel) & alloc_ch) == 0)) {
		printf ("ERR: Request for xfer on "
		       "unallocated channel %d\r\n", channel);
		BUG();
	}
	dma_base->int_tc_clear = _BIT(channel);
	dma_base->int_err_clear = _BIT(channel);
	dma_base->dma_chan[channel].src_addr = desc->dma_src;
	dma_base->dma_chan[channel].dest_addr = desc->dma_dest;
	dma_base->dma_chan[channel].lli = desc->next_lli;
	dma_base->dma_chan[channel].control = desc->next_ctrl;
	dma_base->dma_chan[channel].config_ch = config;

	return 0;
}

int lpc32xx_dma_wait_status(int channel)
{
	while((
	      (dma_base->raw_tc_stat | dma_base->raw_err_stat) &
	      _BIT(channel)) == 0
	     ) ;

	if (unlikely(dma_base->raw_err_stat & _BIT(channel))) {
		dma_base->int_err_clear |= _BIT(channel);
		dma_base->raw_err_stat |= _BIT(channel);
		return -1;
	}
	dma_base->int_tc_clear |= _BIT(channel);
	dma_base->raw_tc_stat |= _BIT(channel);
	return 0;
}

void lpc32xx_dma_put_channel(int channel)
{
	/* Check if given channel no is valid */
	if (channel >= DMA_NO_OF_CHANNELS || channel < 0)
		return ;
	alloc_ch &= ~_BIT(channel);

	/* Shut down channel */
	dma_base->dma_chan [channel].control = 0;
	dma_base->dma_chan [channel].config_ch = 0;
	dma_base->sync &= ~_BIT(channel);

	if (!alloc_ch) {
		/* Disable DMA controller */
		dma_base->config &= ~DMAC_CTRL_ENABLE;

		/* If all channels are free disable the clock */
		CLKPWR->clkpwr_dmaclk_ctrl &= ~DMA_CLK_ENABLE;
	}
}

