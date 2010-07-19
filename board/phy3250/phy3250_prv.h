/***********************************************************************
 * $Id:: phy3250_board.h 970 2008-07-28 21:01:39Z wellsk               $
 *
 * Project: Phytec 3250 board definitions
 *
 * Description:
 *     This file contains board specific information such as the
 *     chip select wait states, and other board specific information.
 *
 ***********************************************************************
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * NXP Semiconductors assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. NXP Semiconductors
 * reserves the right to make changes in the software without
 * notification. NXP Semiconductors also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.
 **********************************************************************/

#ifndef PHY3250_PRV_H
#define PHY3250_PRV_H

/* Structure used to define the hardware for the Phytec board */
typedef struct
{
  unsigned long dramcfg;    /* DRAM config word */
  unsigned long syscfg;     /* Configuration word */
  /* MAC address, use lower 6 bytes only, index 0 is first byte */
  u_char  mac[8];     /* Only the first 6 are used */
  unsigned long rsvd [5];   /* Reserved, must be 0 */
  unsigned long fieldvval;  /* Must be PHY_HW_VER_VAL */
} PHY_HW_T;
extern PHY_HW_T phyhwdesc;

/* Phytec hardware verification value for configuation field */
#define PHY_HW_VER_VAL 0x000A3250

/***********************************************************************
 * DRAM config word (dramcfg) bits
 * Bits        Value             Description
 * 1..0        00                Low power SDRAM
 * 1..0        01                SDRAM
 * 1..0        1x                Reserved
 * 4..2        000               SDRAM 16M,  16 bits x 2 devices, 0xa5
 * 4..2        001               SDRAM 32M,  16 bits x 2 devices, 0xa9
 * 4..2        010               SDRAM 64M,  16 bits x 2 devices, 0xad
 * 4..2        011               SDRAM 128M, 16 bits x 2 devices, 0xb1
 * 31..5       0                 Reserved
 **********************************************************************/
/* Mask for get the DRAM type */
#define PHYHW_DRAM_TYPE_MASK       0x3
/* DRAM types (2 bits) */
#define PHYHW_DRAM_TYPE_LPSDRAM    0x0
#define PHYHW_DRAM_TYPE_SDRAM      0x1
#define PHYHW_DRAM_TYPE_LPDDR      0x2
#define PHYHW_DRAM_TYPE_DDR        0x3
/* DRAM configuration mask */
#define PHYHW_DRAM_SIZE_MASK       _SBF(2, 0x3)
/* DRAM configurations (3 bits) */
#define PHYHW_DRAM_SIZE_16M        _SBF(2, 0x0)
#define PHYHW_DRAM_SIZE_32M        _SBF(2, 0x1)
#define PHYHW_DRAM_SIZE_64M        _SBF(2, 0x2)
#define PHYHW_DRAM_SIZE_128M       _SBF(2, 0x3)

/***********************************************************************
 * Configuation word (syscfg) bits
 * Bits        Value             Description
 * 0           0                 SDIO not populated
 * 0           1                 SDIO populated
 * 31..1       0                 Reserved
 **********************************************************************/
/* SDIO populated bit */
#define PHYHW_SDIO_POP             _BIT(0)

/***********************************************************************
 * NAND timing information
 **********************************************************************/
/* NAND256R3A2CZA6 device */
#define PHY_NAND_TCEA_DELAY   22222222
#define PHY_NAND_BUSY_DELAY   10000000
#define PHY_NAND_NAND_TA      33333333
#define PHY_NAND_RD_HIGH      66666666
#define PHY_NAND_RD_LOW       33333333
#define PHY_NAND_WR_HIGH      50000000
#define PHY_NAND_WR_LOW       25000000

/***********************************************************************
 * Functions
 **********************************************************************/

/* Serial EEPROM commands (SPI via SSP) */
#define SEEPROM_WREN          0x06
#define SEEPROM_WRDI          0x04
#define SEEPROM_RDSR          0x05
#define SEEPROM_WRSR          0x01
#define SEEPROM_READ          0x03
#define SEEPROM_WRITE         0x02

/* Size of serial EEPROM */
#define PHY3250_SEEPROM_SIZE  0x8000

/* Offset into serial EEPROM where board configuration information is
   saved */
#define PHY3250_SEEPROM_CFGOFS (PHY3250_SEEPROM_SIZE - 0x100)




// Enable the following define to setup for RMII mode
#define USE_PHY_RMII

// PHY address (configured via PHY ADRx pins)
#define PHYDEF_PHYADDR           0x00

// LAN8700 Ethernet Phy Ctrl/Status Register
#define LAN8700_PHY_STATUS	0x1F

// Maximum ethernet frame size, maximum RX and TX packets
#define ENET_MAXF_SIZE             1536
#define ENET_MAX_TX_PACKETS        16
#define ENET_MAX_RX_PACKETS        16



#endif /* PHY3250_PRV_H */
