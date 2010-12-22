/*
 * Copyright (C) 2008 by NXP Semiconductors
 * All rights reserved.
 *
 * @Author: Kevin Wells
 * @Descr: LPC3250 SLC NAND controller interface support functions
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
#include "lpc3250.h"
#include <nand.h>
#include <asm/errno.h>
#include <asm/io.h>

#define	NAND_ALE_OFFS	4
#define	NAND_CLE_OFFS	8

#define NAND_LARGE_BLOCK_PAGE_SIZE	2048
#define NAND_SMALL_BLOCK_PAGE_SIZE	512

static struct nand_ecclayout lpc32xx_nand_oob_16 = {
        .eccbytes = 6,
        .eccpos = {10, 11, 12, 13, 14, 15},
        .oobfree = {
                {.offset = 0,
                 . length = 4},
                {.offset = 6,
                 . length = 4}
		}
};

extern int nand_correct_data(struct mtd_info *mtd, u_char *dat,
                             u_char *read_ecc, u_char *calc_ecc);
/*
 * DMA Descriptors
 * For Large Block: 17 descriptors = ((16 Data and ECC Read) + 1 Spare Area)
 * For Small Block: 5 descriptors = ((4 Data and ECC Read) + 1 Spare Area)
 */
static dmac_ll_t dmalist[(CONFIG_SYS_NAND_ECCSIZE/256) * 2 + 1];
static uint32_t ecc_buffer[8]; /* MAX ECC size */
static int dmachan = -1;
#define XFER_PENDING ((SLCNAND->slc_stat & SLCSTAT_DMA_FIFO) | SLCNAND->slc_tc)

static void lpc32xx_nand_init(void)
{
	/* Enable clocks to the SLC NAND controller */
	CLKPWR->clkpwr_nand_clk_ctrl = (CLKPWR_NANDCLK_SEL_SLC |
					CLKPWR_NANDCLK_SLCCLK_EN);

        /* Reset SLC NAND controller & clear ECC */
        SLCNAND->slc_ctrl = (SLCCTRL_SW_RESET | SLCCTRL_ECC_CLEAR);

        /* 8-bit bus, no DMA, CE normal */
        SLCNAND->slc_cfg = 0;

        /* Interrupts disabled and cleared */
        SLCNAND->slc_ien = 0;
        SLCNAND->slc_icr = (SLCSTAT_INT_TC | SLCSTAT_INT_RDY_EN);

        SLCNAND->slc_tac = LPC32XX_SLC_NAND_TIMING;
}

static void lpc32xx_nand_hwcontrol(struct mtd_info *mtd, int cmd, unsigned int ctrl)
{
	struct nand_chip *this = mtd->priv;
	ulong  IO_ADDR_W;

	if (ctrl & NAND_CTRL_CHANGE) {
		IO_ADDR_W = (ulong) this->IO_ADDR_W;
		IO_ADDR_W &= ~(NAND_CLE_OFFS | NAND_ALE_OFFS);

		if ( ctrl & NAND_CLE ) {
			IO_ADDR_W |= NAND_CLE_OFFS;
		}
		else if ( ctrl & NAND_ALE ) {
			IO_ADDR_W |= NAND_ALE_OFFS;
		}

		if ( ctrl & NAND_NCE ) {
			SLCNAND->slc_cfg |= SLCCFG_CE_LOW;
		}
		else {
			SLCNAND->slc_cfg &= ~SLCCFG_CE_LOW;
		}

		this->IO_ADDR_W = (void *) IO_ADDR_W;
	}

	if (cmd != NAND_CMD_NONE) {
		writel(cmd, this->IO_ADDR_W);
	}
}

static int lpc32xx_nand_ready(struct mtd_info *mtd)
{
	/* Check the SLC NAND controller status */
	return (SLCNAND->slc_stat & SLCSTAT_NAND_READY);
}

static u_char lpc32xx_read_byte(struct mtd_info *mtd)
{
	struct nand_chip *this = mtd->priv;
	unsigned long *pReg = (unsigned long *) this->IO_ADDR_R;
	volatile unsigned long tmp32;
	tmp32 = *pReg;
	return (u_char) tmp32;
}

/*
 * lpc32xx_verify_buf - [DEFAULT] Verify chip data against buffer
 * mtd:	MTD device structure
 * buf:	buffer containing the data to compare
 * len:	number of bytes to compare
 *
 * Default verify function for 8bit buswith
 */
static int lpc32xx_verify_buf(struct mtd_info *mtd, const u_char *buf, int len)
{
	int i;
	struct nand_chip *this = mtd->priv;
	unsigned long *pReg = (unsigned long *) this->IO_ADDR_R;
	volatile unsigned long tmp32;

	for (i=0; i<len; i++) {
		tmp32 = *pReg;
		if (buf[i] != (u_char) tmp32)
			return -EFAULT;
	}
	return 0;
}

/* Prepares DMA descriptors for NAND RD/WR operations */
/* If the size is < 256 Bytes then it is assumed to be
 * an OOB transfer */
static void lpc32xx_nand_dma_configure(struct nand_chip *chip,
		const void * buffer, int size, int read)
{
	uint32_t i, dmasrc, ctrl, ecc_ctrl, oob_ctrl, dmadst;
	void __iomem * base = chip->IO_ADDR_R;
	uint32_t *ecc_gen = ecc_buffer;

	/*
	 * CTRL descriptor entry for reading ECC
	 * Copy Multiple times to sync DMA with Flash Controller
	 */
	ecc_ctrl =  (0x5 |
			DMAC_CHAN_SRC_BURST_1 |
			DMAC_CHAN_DEST_BURST_1 |
			DMAC_CHAN_SRC_WIDTH_32 |
			DMAC_CHAN_DEST_WIDTH_32 |
			DMAC_CHAN_DEST_AHB1);

	/* CTRL descriptor entry for reading/writing Data */
	ctrl =  64 | /* 256/4 */
			DMAC_CHAN_SRC_BURST_4 |
			DMAC_CHAN_DEST_BURST_4 |
			DMAC_CHAN_SRC_WIDTH_32 |
			DMAC_CHAN_DEST_WIDTH_32 |
			DMAC_CHAN_DEST_AHB1;

	/* CTRL descriptor entry for reading/writing Spare Area */
	oob_ctrl =  ((CONFIG_SYS_NAND_OOBSIZE / 4) |
			DMAC_CHAN_SRC_BURST_4 |
			DMAC_CHAN_DEST_BURST_4 |
			DMAC_CHAN_SRC_WIDTH_32 |
			DMAC_CHAN_DEST_WIDTH_32 |
			DMAC_CHAN_DEST_AHB1);

	if (read) {
		dmasrc = (uint32_t) (base + offsetof(SLCNAND_REGS_T, slc_dma_data));
		dmadst = (uint32_t) (buffer);
		ctrl |= DMAC_CHAN_DEST_AUTOINC;
	} else {
		dmadst = (uint32_t) (base + offsetof(SLCNAND_REGS_T, slc_dma_data));
		dmasrc = (uint32_t) (buffer);
		ctrl |= DMAC_CHAN_SRC_AUTOINC;
	}

	/*
	 * Write Operation Sequence for Small Block NAND
	 * ----------------------------------------------------------
	 * 1. X'fer 256 bytes of data from Memory to Flash.
	 * 2. Copy generated ECC data from Register to Spare Area
	 * 3. X'fer next 256 bytes of data from Memory to Flash.
	 * 4. Copy generated ECC data from Register to Spare Area.
	 * 5. X'fer 16 byets of Spare area from Memory to Flash.
	 * Read Operation Sequence for Small Block NAND
	 * ----------------------------------------------------------
	 * 1. X'fer 256 bytes of data from Flash to Memory.
	 * 2. Copy generated ECC data from Register to ECC calc Buffer.
	 * 3. X'fer next 256 bytes of data from Flash to Memory.
	 * 4. Copy generated ECC data from Register to ECC calc Buffer.
	 * 5. X'fer 16 bytes of Spare area from Flash to Memory.
	 * Write Operation Sequence for Large Block NAND
	 * ----------------------------------------------------------
	 * 1. Steps(1-4) of Write Operations repeate for four times
	 * which generates 16 DMA descriptors to X'fer 2048 bytes of
	 * data & 32 bytes of ECC data.
	 * 2. X'fer 64 bytes of Spare area from Memory to Flash.
	 * Read Operation Sequence for Large Block NAND
	 * ----------------------------------------------------------
	 * 1. Steps(1-4) of Read Operations repeate for four times
	 * which generates 16 DMA descriptors to X'fer 2048 bytes of
	 * data & 32 bytes of ECC data.
	 * 2. X'fer 64 bytes of Spare area from Flash to Memory.
	 */

	for (i = 0; i < size/256; i++) {
		dmalist[i*2].dma_src = (read ?(dmasrc) :(dmasrc + (i*256)));
		dmalist[i*2].dma_dest = (read ?(dmadst + (i*256)) :dmadst);
		dmalist[i*2].next_lli = (uint32_t) & dmalist[(i*2)+1];
		dmalist[i*2].next_ctrl = ctrl;

		dmalist[(i*2) + 1].dma_src = (uint32_t)
		        (base + offsetof(SLCNAND_REGS_T, slc_ecc));
		dmalist[(i*2) + 1].dma_dest =  (uint32_t) & ecc_gen[i];
		dmalist[(i*2) + 1].next_lli = (uint32_t) & dmalist[(i*2)+2];
		dmalist[(i*2) + 1].next_ctrl = ecc_ctrl;
	}

	if (i) { /* Data only transfer */
		dmalist[(i*2) - 1].next_lli = 0;
		dmalist[(i*2) - 1].next_ctrl |= DMAC_CHAN_INT_TC_EN;
		return ;
	}

	/* OOB only transfer */
	if (read) {
		dmasrc = (uint32_t) (base + offsetof(SLCNAND_REGS_T, slc_dma_data));
		dmadst = (uint32_t) (buffer);
		oob_ctrl |= DMAC_CHAN_DEST_AUTOINC;
	} else {
		dmadst = (uint32_t) (base + offsetof(SLCNAND_REGS_T, slc_dma_data));
		dmasrc = (uint32_t) (buffer);
		oob_ctrl |= DMAC_CHAN_SRC_AUTOINC;
	}

	/* Read/ Write Spare Area Data To/From Flash */
	dmalist[i*2].dma_src = dmasrc;
	dmalist[i*2].dma_dest = dmadst;
	dmalist[i*2].next_lli = 0;
	dmalist[i*2].next_ctrl = (oob_ctrl | DMAC_CHAN_INT_TC_EN);
}

static void lpc32xx_nand_xfer(struct mtd_info *mtd, const u_char *buf, int len, int read)
{
	struct nand_chip *chip = mtd->priv;
	uint32_t config;

	/* DMA Channel Configuration */
	config = (read ? DMAC_CHAN_FLOW_D_P2M : DMAC_CHAN_FLOW_D_M2P) |
		(read ? DMAC_DEST_PERIP(0) : DMAC_DEST_PERIP(DMA_PERID_NAND1)) |
		(read ? DMAC_SRC_PERIP(DMA_PERID_NAND1) : DMAC_SRC_PERIP(0)) |
		DMAC_CHAN_ENABLE;

	/* Prepare DMA descriptors */
	lpc32xx_nand_dma_configure(chip, buf, len, read);

	/* Setup SLC controller and start transfer */
	if (read)
		SLCNAND->slc_cfg |= SLCCFG_DMA_DIR;
	else  /* NAND_ECC_WRITE */
		SLCNAND->slc_cfg &= ~SLCCFG_DMA_DIR;
	SLCNAND->slc_cfg |= SLCCFG_DMA_BURST;

	/* Write length for new transfers */
	if (!XFER_PENDING)
		SLCNAND->slc_tc = len +
			(len != mtd->oobsize ? mtd->oobsize : 0);

	SLCNAND->slc_ctrl |= SLCCTRL_DMA_START;

	/* Start DMA transfers */
	lpc32xx_dma_start_xfer(dmachan, dmalist, config);

	/* Wait for NAND to be ready */
	while(!lpc32xx_nand_ready(mtd));

	/* Wait till DMA transfer is DONE */
	if (lpc32xx_dma_wait_status(dmachan)) {
		printk(KERN_ERR "NAND DMA transfer error!\r\n");
	}

	/* Stop DMA & HW ECC */
	SLCNAND->slc_ctrl &= ~SLCCTRL_DMA_START;
	SLCNAND->slc_cfg &= ~(SLCCFG_DMA_DIR | SLCCFG_DMA_BURST |
				SLCCFG_ECC_EN | SLCCFG_DMA_ECC);
}

static uint32_t slc_ecc_copy_to_buffer(uint8_t * spare,
		const uint32_t * ecc, int count)
{
	int i;
	for (i = 0; i < (count * 3); i += 3) {
		uint32_t ce = ecc[i/3];
		ce = ~(ce << 2) & 0xFFFFFF;
		spare[i+2] = (uint8_t)(ce & 0xFF); ce >>= 8;
		spare[i+1] = (uint8_t)(ce & 0xFF); ce >>= 8;
		spare[i]   = (uint8_t)(ce & 0xFF);
	}
	return 0;
}

static int lpc32xx_ecc_calculate(struct mtd_info *mtd, const uint8_t *dat,
					     uint8_t *ecc_code)
{
	return slc_ecc_copy_to_buffer(ecc_code, ecc_buffer,
			CONFIG_SYS_NAND_ECCSIZE == NAND_LARGE_BLOCK_PAGE_SIZE ? 8 : 2);
}

/*
 * Enables and prepares SLC NAND controller
 * for doing data transfers with H/W ECC enabled.
 */
static void lpc32xx_hwecc_enable(struct mtd_info *mtd, int mode)
{
	/* Clear ECC */
	SLCNAND->slc_ctrl = SLCCTRL_ECC_CLEAR;

	/* Setup SLC controller for H/W ECC operations */
	SLCNAND->slc_cfg |= (SLCCFG_ECC_EN | SLCCFG_DMA_ECC);
}

/*
 * lpc32xx_write_buf - [DEFAULT] write buffer to chip
 * mtd:	MTD device structure
 * buf:	data buffer
 * len:	number of bytes to write
 *
 * Default write function for 8bit buswith
 */
static void lpc32xx_write_buf(struct mtd_info *mtd, const u_char *buf, int len)
{
	lpc32xx_nand_xfer(mtd, buf, len, 0);
}

/*
 * lpc32xx_read_buf - [DEFAULT] read chip data into buffer
 * mtd:	MTD device structure
 * buf:	buffer to store date
 * len:	number of bytes to read
 *
 * Default read function for 8bit buswith
 */
static void lpc32xx_read_buf(struct mtd_info *mtd, u_char *buf, int len)
{
	lpc32xx_nand_xfer(mtd, buf, len, 1);
}

int board_nand_init(struct nand_chip *nand)
{
	/* Initial NAND interface */
	lpc32xx_nand_init();

	/* Acquire a channel for our use */
	dmachan = lpc32xx_dma_get_channel();
	if (unlikely(dmachan < 0)){
		printk(KERN_INFO "Unable to get a free DMA "
				"channel for NAND transfers\r\n");
		return -1;
	}

	/* ECC mode and size */
	nand->ecc.mode = NAND_ECC_HW;
	nand->ecc.bytes	= CONFIG_SYS_NAND_ECCBYTES;
	nand->ecc.size = CONFIG_SYS_NAND_ECCSIZE;

	if(CONFIG_SYS_NAND_ECCSIZE != NAND_LARGE_BLOCK_PAGE_SIZE)
		nand->ecc.layout = &lpc32xx_nand_oob_16;

	nand->ecc.calculate = lpc32xx_ecc_calculate;
	nand->ecc.correct = nand_correct_data;
	nand->ecc.hwctl = lpc32xx_hwecc_enable;
	nand->cmd_ctrl = lpc32xx_nand_hwcontrol;
	nand->dev_ready = lpc32xx_nand_ready;
	nand->chip_delay = 2000;

	nand->read_buf = lpc32xx_read_buf;
	nand->write_buf = lpc32xx_write_buf;
	nand->read_byte = lpc32xx_read_byte;
	nand->verify_buf = lpc32xx_verify_buf;

	return 0;
}
