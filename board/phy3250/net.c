/*
 * Copyright (C) 2008 by NXP Semiconductors
 * All rights reserved.
 *
 * @Author: Kevin Wells
 * @Descr: Phytec 3250 net interface support functions
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <common.h>
#include <command.h>
#include <net.h>
#include <lpc3250.h>
#include <configs/phy3250.h>
#include "phy3250_prv.h"
#include "miiphy.h"

static unsigned long g_dmabase;
static unsigned long gdma_size;
static TXRX_DESC_T *pTXDesc;
static unsigned long *pTXStatus;
static unsigned long pTXVBuffs [ENET_MAX_TX_PACKETS];
static TXRX_DESC_T *pRXDesc;
static RX_STATUS_T *pRXStatus;
static unsigned long pRXVBuffs [ENET_MAX_RX_PACKETS];
#ifndef USE_IRAM_FOR_ETH_BUFFERS
static unsigned long ethbuffs[4096];
#endif
extern PHY_HW_T phyhwdesc;

static void msDelay(unsigned long ms)
{
	udelay(ms * 1000);
}

//------------------------------------------------------------------------------
static int RMII_Write (unsigned long PhyReg, unsigned long Value)
{
	unsigned long mst = 250;
	int sts = 0;

	// Write value at PHY address and register
	ENETMAC->madr = (PHYDEF_PHYADDR << 8) | PhyReg;
	ENETMAC->mwtd = Value;

	// Wait for unbusy status
	while (mst > 0)
	{
		if ((ENETMAC->mind & MIND_BUSY) == 0)
		{
			mst = 0;
			sts = 1;
		}
		else
		{
			mst--;
			msDelay(1);
		}
	}

	return sts;
}

//------------------------------------------------------------------------------
int RMII_Read(unsigned long PhyReg, unsigned long *data) 
{
	unsigned long mst = 250;
	int sts = 0;

	// Read value at PHY address and register
	ENETMAC->madr = (PHYDEF_PHYADDR << 8) | PhyReg;
	ENETMAC->mcmd = MCMD_READ;

	// Wait for unbusy status
	while (mst > 0)
	{
		if ((ENETMAC->mind & MIND_BUSY) == 0)
		{
			mst = 0;
			*data = ENETMAC->mrdd;
			sts = 1;
		}
		else
		{
			mst--;
			msDelay(1);
		}
	}

	ENETMAC->mcmd = 0;

	return sts;
}

static int phy_get_link_status (void)
{
	unsigned long status;

	/* Status is read once to clear old link state */
	RMII_Read(PHY_BMSR,&status);

	/*
	 * Wait if the link is up, and autonegotiation is in progress
	 * (ie - we're capable and it's not done)
	 */
	status = 0;
	RMII_Read(PHY_BMSR,&status);
	if ((status & PHY_BMSR_LS) && (status & PHY_BMSR_AUTN_ABLE)
			&& !(status & PHY_BMSR_AUTN_COMP)) {
		int i = 0;

		while (!(status & PHY_BMSR_AUTN_COMP)) {
			/*
			 * Timeout reached ?
			 */
			if (i > 1000) {
				/* Time Out Occur */
				printf("Timeout\n");
				return 1;
			}
			i++;
			msDelay(1);   /* 1 ms */
			RMII_Read(PHY_BMSR,&status);
		}
		return 0;
	} else {
		if (status & PHY_BMSR_LS)
			return 0;
		else
			return 1;
	}
	return 1;
}

//------------------------------------------------------------------------------
int txrx_setup(void)
{
	int idx;
	unsigned long *pTXStatusL, pbase1, pbase2, pbase3;
	TXRX_DESC_T *pTXRXDesc;
	RX_STATUS_T *pRXStatusL;

	// Get physical address and size of DMA buffers
#ifdef USE_IRAM_FOR_ETH_BUFFERS
	g_dmabase = (unsigned long) IRAM_ETH_BUFF_BASE;
	gdma_size = IRAM_ETH_BUFF_SIZE;
#else
	g_dmabase = (unsigned long) &ethbuffs;
	gdma_size = sizeof(ethbuffs);
#endif

	// Setup base pointers
	pbase1 = g_dmabase;     // Start of descriptors
	pbase2 = pbase1 + 256;  // Start of statuses
	pbase3 = pbase1 + 1024; // Start of buffers

	// Setup pointers to TX structures
	ENETMAC->txdescriptor =  pbase1;
	ENETMAC->txstatus = pbase2;
	ENETMAC->txdescriptornumber = (ENET_MAX_TX_PACKETS - 1);

	// Save base address of TX descriptor table and TX status
	pTXRXDesc = (TXRX_DESC_T *) pbase1;
	pTXStatusL = (unsigned long *) pbase2;
	pTXDesc = pTXRXDesc;
	pTXStatus = pTXStatusL;

	// Build TX descriptors
	for (idx = 0; idx < ENET_MAX_TX_PACKETS; idx++)
	{
		pTXRXDesc->packet = pbase3;
		pTXRXDesc->control = 0;
		*pTXStatusL = 0;

		// Save virtual address of buffer
#ifdef USE_IRAM_FOR_ETH_BUFFERS
		pTXVBuffs [idx] = (unsigned long) pbase3;
#else
#pragma error "NOT supported"
#endif

		// Next descriptor and status
		pTXRXDesc++;
		pTXStatusL++;
		pbase1 += sizeof (TXRX_DESC_T);
		pbase2 += sizeof (unsigned long);
		pbase3 += ENET_MAXF_SIZE;
	}

	// Setup pointers to RX structures
	ENETMAC->rxdescriptor = pbase1;
	ENETMAC->rxstatus = pbase2;
	ENETMAC->rxdescriptornumber = (ENET_MAX_RX_PACKETS - 1);

	// Save base address of RX descriptor table and RX status
	pRXDesc = pTXRXDesc;
	pRXStatus = pRXStatusL = (RX_STATUS_T *) pTXStatusL;

	// Build RX descriptors
	for (idx = 0; idx < ENET_MAX_TX_PACKETS; idx++)
	{
		pTXRXDesc->packet = pbase3;
		pTXRXDesc->control = 0x80000000 | (ENET_MAXF_SIZE - 1);
		pRXStatusL->statusinfo = 0;
		pRXStatusL->statushashcrc = 0;

		// Save virtual address of buffer
#ifdef USE_IRAM_FOR_ETH_BUFFERS
		pRXVBuffs [idx] = (unsigned long) pbase3;
#else
#pragma error "NOT supported"
#endif

		// Next descriptor and status
		pTXRXDesc++;
		pRXStatusL++;
		pbase1 += sizeof (TXRX_DESC_T);
		pbase2 += sizeof (unsigned long);
		pbase3 += ENET_MAXF_SIZE;
	}

    return 1;
}

//------------------------------------------------------------------------------
int HWInit(bd_t * bd)
{
	int btemp;
	unsigned int duplex, speed;
	unsigned long tmp1, mst = 250;
	int val = 0;

	// Enable MAC interface
	CLKPWR->clkpwr_macclk_ctrl = (CLKPWR_MACCTRL_HRCCLK_EN |
		CLKPWR_MACCTRL_MMIOCLK_EN | CLKPWR_MACCTRL_DMACLK_EN |
#ifdef USE_PHY_RMII
		CLKPWR_MACCTRL_USE_RMII_PINS);
#else
		CLKPWR_MACCTRL_USE_MII_PINS);
#endif

	// Set RMII management clock rate. This clock should be slower
	// than 12.5MHz (for NXP PHYs only). For a divider of 28, the
	// clock rate when HCLK is 150MHz will be 5.4MHz
	ENETMAC->mcfg = MCFG_CLOCK_SELECT(MCFG_CLOCK_HOST_DIV_28);

	// Reset all MAC logic
	ENETMAC->mac1 = (MAC1_SOFT_RESET | MAC1_SIMULATION_RESET |
		MAC1_RESET_MCS_TX | MAC1_RESET_TX | MAC1_RESET_MCS_RX |
		MAC1_RESET_RX);
	ENETMAC->command = (COMMAND_REG_RESET | COMMAND_TXRESET |
		COMMAND_RXRESET);
	msDelay(10);

	// Initial MAC initialization
	ENETMAC->mac1 = MAC1_PASS_ALL_RX_FRAMES;
	ENETMAC->mac2 = (MAC2_PAD_CRC_ENABLE | MAC2_CRC_ENABLE);
	ENETMAC->maxf = ENET_MAXF_SIZE;

	// Maximum number of retries, 0x37 collision window, gap */
	ENETMAC->clrt = (CLRT_LOAD_RETRY_MAX(0xF) |
		CLRT_LOAD_COLLISION_WINDOW(0x37));
	ENETMAC->ipgr = IPGR_LOAD_PART2(0x12);

#ifdef USE_PHY_RMII
	// RMII setup
	ENETMAC->command = (COMMAND_RMII | COMMAND_PASSRUNTFRAME);
	ENETMAC->supp = SUPP_RESET_RMII;
	msDelay(10);
#else
	// MII setup
	ENETMAC->command = COMMAND_PASSRUNTFRAME;
#endif

	mst = 1000;
	btemp = 1;
	while(mst > 0)
	{
		/* Wait for Link status to set UP or Timeout */
		if(phy_get_link_status() == 0) {
			mst = 0;
			btemp = 0;
			printf("ENET:auto-negotiation complete\n");
		}
		else {
			mst--;
			msDelay(1);
		}
	}
	if(btemp) {
		printf("ENET:auto-negotiation failed.\n");
		return 0;
	}

	/*
	 *  Read Vendor specific PHY Ctrl/Status Register to determine
	 *  Ethernet Configuration
	 */
	tmp1 = 0;
	RMII_Read (LAN8700_PHY_STATUS,&tmp1);
	val = (tmp1 & 0x1c) >> 2;
	switch (val) {
		case 1:
			/* 10Mbps Half Duplex */
			duplex = 0;
			speed = 0;
			break;
		case 5:
			/* 10Mbps Full Duplex */
			duplex = 1;
			speed = 0;
			break;
		case 2:
			/* 100Mbps Half Duplex */
			duplex = 0;
			speed = 1;
			break;
		case 6:
			/* 100Mbps Full Duplex */
			duplex = 1;
			speed = 1;
			break;
	}

	// Configure Full/Half Duplex mode
	if (duplex == 1)
	{
		// 10MBase full duplex is supported
		ENETMAC->mac2 |= MAC2_FULL_DUPLEX;
		ENETMAC->command |= COMMAND_FULLDUPLEX;
		ENETMAC->ipgt = IPGT_LOAD(0x15);
		printf("ENET:FULL DUPLEX\n");
	}
	else
	{
		ENETMAC->ipgt = IPGT_LOAD(0x12);
		printf("ENET:HALF DUPLEX\n");
	}

	// Configure 100MBit/10MBit mode
	if (speed == 1)
	{
		// 100MBase mode
		ENETMAC->supp = SUPP_SPEED;
		printf("ENET:100MBase\n");
	}
	else
	{
		// 10MBase mode
		ENETMAC->supp = 0;
		printf("ENET:10Base\n");
	}

	// Save station address
	ENETMAC->sa [2] = (unsigned long) (bd->bi_enetaddr[0] | (bd->bi_enetaddr[1] << 8));
	ENETMAC->sa [1] = (unsigned long) (bd->bi_enetaddr[2] | (bd->bi_enetaddr[3] << 8));
	ENETMAC->sa [0] = (unsigned long) (bd->bi_enetaddr[4] | (bd->bi_enetaddr[5] << 8));

	// Setup TX and RX descriptors
	txrx_setup();

	// Enable broadcast and matching address packets
	ENETMAC->rxfliterctrl = (RXFLTRW_ACCEPTUBROADCAST |
		RXFLTRW_ACCEPTPERFECT);

	// Clear and enable interrupts
	ENETMAC->intclear = 0xFFFF;
	ENETMAC->intenable = 0;

	// Enable receive and transmit mode of MAC ethernet core
	ENETMAC->command |= (COMMAND_RXENABLE | COMMAND_TXENABLE);
	ENETMAC->mac1 |= MAC1_RECV_ENABLE;

	// Perform a 'dummy' send of the first ethernet frame with a size of 0
	// to 'prime' the MAC. The first packet after a reset seems to wait
	// until at least 2 packets are ready to go.
	tmp1 = 0;
	eth_send(&tmp1, 4);

	return 1;
}

//------------------------------------------------------------------------------
int HWDeInit(void)
{
	// Reset all MAC logic
	ENETMAC->mac1 = (MAC1_SOFT_RESET | MAC1_SIMULATION_RESET |
		MAC1_RESET_MCS_TX | MAC1_RESET_TX | MAC1_RESET_MCS_RX |
		MAC1_RESET_RX);
	ENETMAC->command = (COMMAND_REG_RESET | COMMAND_TXRESET |
		COMMAND_RXRESET);
	msDelay(2);

	// Disable MAC clocks, but keep MAC interface active
#ifdef USE_PHY_RMII
	CLKPWR->clkpwr_macclk_ctrl = CLKPWR_MACCTRL_USE_RMII_PINS;
#else
	CLKPWR->clkpwr_macclk_ctrl = CLKPWR_MACCTRL_USE_MII_PINS;
#endif

	return 1;
}

void eth_halt (void)
{
	HWDeInit();
}

int eth_init (bd_t * bd)
{
	int rc;

	// Set MAC address from hardware
	bd->bi_enetaddr[0] = phyhwdesc.mac[0];	bd->bi_enetaddr[1] = phyhwdesc.mac[1];	bd->bi_enetaddr[2] = phyhwdesc.mac[2];	bd->bi_enetaddr[3] = phyhwdesc.mac[3];	bd->bi_enetaddr[4] = phyhwdesc.mac[4];	bd->bi_enetaddr[5] = phyhwdesc.mac[5];
	printf ("\tHW MAC address:  "
		"%02X:%02X:%02X:%02X:%02X:%02X\n",
		bd->bi_enetaddr[0], bd->bi_enetaddr[1],
		bd->bi_enetaddr[2], bd->bi_enetaddr[3],
		bd->bi_enetaddr[4], bd->bi_enetaddr[5] );

	rc = HWInit(bd);

	// De-init if an error occurred
	if (rc == 0)
	{
		printf ("ENET init failure\n");
		HWDeInit();
		return -1;
	}

	return 0;
}

/* Get a data block via Ethernet */
int eth_rx (void)
{
	unsigned long idx, length;

	// Determine if a frame has been received
	length = 0;
	idx = ENETMAC->rxconsumeindex;
	if (ENETMAC->rxproduceindex != idx)
	{
		// Clear interrupt
		ENETMAC->intclear = MACINT_RXDONEINTEN;

		// Frame received, get size of RX packet
		length = (pRXStatus[idx].statusinfo & 0x7FF);

		/* Pass the packet up to the protocol layer */
		if (length > 0)
		{
		        memcpy((void *) NetRxPackets[0], (void *) pRXVBuffs [idx], length);
			NetReceive (NetRxPackets[0], (unsigned short) length);
		}

		// Return DMA buffer
		idx++;
		if (idx >= ENET_MAX_TX_PACKETS)
		{
			idx = 0;
		}
		ENETMAC->rxconsumeindex = (unsigned long) idx;
	}

	return (int) length;
}

/* Send a data block via Ethernet. */
int eth_send (volatile void *packet, int length)
{
	unsigned long idx, cidx, fb;

	// Determine number of free buffers and wait for a buffer if needed
	fb = 0;
	while (fb == 0)
	{
		idx = ENETMAC->txproduceindex;
		cidx = ENETMAC->txconsumeindex;

		if (idx == cidx)
		{
			// Producer and consumer are the same, all buffers are free
			fb = ENET_MAX_TX_PACKETS;
		}
		else if (cidx > idx)
		{
			fb = (ENET_MAX_TX_PACKETS - 1) -
				((idx + ENET_MAX_TX_PACKETS) - cidx);
		}
		else
		{
			fb = (ENET_MAX_TX_PACKETS - 1) - (cidx - idx);
		}
	}

	// Update descriptor with new frame size
	pTXDesc[idx].control = (length | 0x40000000);

	// Move data to buffer
	memcpy((void *) pTXVBuffs [idx], (void *) packet, length);

	// Get next index for transmit data DMA buffer and descriptor
	idx++;
	if (idx >= ENET_MAX_TX_PACKETS)
	{
		idx = 0;
	}
	ENETMAC->txproduceindex = idx;

	return 0;
}

