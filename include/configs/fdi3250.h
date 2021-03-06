/*
 * Copyright (C) 2010 by NXP Semiconductors
 * All rights reserved.
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

/*
 * FDI LPC3250 Board configuation data
 */

#ifndef __FDI3250_H__
#define __FDI3250_H__

/*
 * Ethernet buffer support in uncached IRAM and buffer size
 */
#define USE_IRAM_FOR_ETH_BUFFERS
#define IRAM_ETH_BUFF_BASE 0x08010000 /* Uncached IRAM */
#define IRAM_ETH_BUFF_SIZE 0x00010000

/*
 * Linux machine type
 */
#define MACH_TYPE_FDI3250 (2513)
#define MACH_TYPE_UBOOTSYS MACH_TYPE_FDI3250

/*
 * System UART selection, valid selections include UART1 via UART7
 */
#define CFG_UART_SEL UART1
#define USE_HIGH_SPEED_UART /* Only enable for 1, 2, or 7 */

/*
 * SDRAM physical Memory Map
 */
#define CONFIG_NR_DRAM_BANKS    1
#define PHYS_SDRAM_1		0x80000000 /* SDRAM Bank #1 */

/*
 * NOR FLASH not supported
 */
#define CONFIG_SYS_NO_FLASH
#undef CONFIG_ENV_IS_IN_FLASH

/*
 * 1KHz clock tick
 */
#define CONFIG_SYS_HZ		1000

/*
 * ARM926ejs options
 */
#define CONFIG_ARM926EJS	   1 /* This is an arm926ejs CPU core  */

/*
 *
 * u-boot specific options
 *
 */

/*
 * Address and size of Environment Data
 */
#define CONFIG_ENV_IS_IN_NAND	1
#define CONFIG_ENV_SIZE		0x40000    /* 2 blocks */
#define CONFIG_ENV_OFFSET	0x100000   /* Blocks 8/9 */
#define CONFIG_ENV_ADDR		0x80000100 /* Passed to kernel here */

/*
 * Area and size for malloc
 */
#define CONFIG_SYS_MALLOC_LEN (CONFIG_ENV_SIZE + 128 * 1024)
#define CONFIG_SYS_GBL_DATA_SIZE 128

/*
 * Stack sizes
 *
 * The stack sizes are set up in start.S using the settings below
 */
#define CONFIG_STACKSIZE (32*1024) /* 32K stack */

/*
 * ATAG support
 */
#define CONFIG_CMDLINE_TAG		1
#define CONFIG_SETUP_MEMORY_TAGS	1
#define CONFIG_INITRD_TAG		1

/*
 * Default baud rate and baud rate table, console config
 */
#define CONFIG_CONS_INDEX	   1
#define CONFIG_BAUDRATE		115200
#define CONFIG_SYS_BAUDRATE_TABLE	{ 9600, 19200, 38400, 57600, 115200 }

/*
 * Default load address for programs
 */
#define CONFIG_SYS_LOAD_ADDR	0x80100000 /* default load address */

/*
 * Default boot delay is 3 seconds
 */
#define CONFIG_BOOTDELAY 3
#define CONFIG_ZERO_BOOTDELAY_CHECK	/* check for keypress on bootdelay==0 */

/*
 * Interrupts are not supported in this boot loader
 */
#undef CONFIG_USE_IRQ

/*
 * Use verbose help
 */
#define CONFIG_SYS_LONGHELP

/*
 * Command line configuration.
 */
#include <config_cmd_default.h>

#define CONFIG_CMD_DHCP
#define CONFIG_CMD_ENV
#define CONFIG_CMD_ECHO /* ECHO command */
#define CONFIG_CMD_CACHE /* Cache support */
#define CONFIG_CMD_RUN
#define CONFIG_CMD_LOADB
#define CONFIG_CMD_LOADS
#define CONFIG_CMD_SAVES
#define CONFIG_CMD_MEMORY
#define CONFIG_CMD_PING
#define CONFIG_CMD_NET
#define CONFIG_CMD_BDI
#define CONFIG_CMD_SAVEENV
#define CONFIG_CMD_ELF
#define CONFIG_CMD_FAT
#undef CONFIG_CMD_MISC

/*
 * Prompt, command buffer
 */
#define	CONFIG_SYS_CBSIZE		256		/* Console I/O Buffer Size	*/
#define	CONFIG_SYS_PROMPT	"uboot> "	/* Monitor Command Prompt	*/
#define	CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE+sizeof(CONFIG_SYS_PROMPT)+16) /* Print Buffer Size */
#define	CONFIG_SYS_MAXARGS		16		/* max number of command args	*/
#define CONFIG_SYS_BARGSIZE		CONFIG_SYS_CBSIZE	/* Boot Argument Buffer Size	*/

/*
 * Default range for the memory tests
 */
#define CONFIG_SYS_MEMTEST_START 0x80010000
#define CONFIG_SYS_MEMTEST_END 0x81000000

/*
 * Support for NAND FLASH, environment store in NAND at block 100
 */
#define CONFIG_CMD_NAND
#define CONFIG_SYS_LPC32XX_NAND    /* Enable SLC NAND controller driver */
#define CONFIG_SYS_MAX_NAND_DEVICE 1
#define CONFIG_SYS_NAND_BASE 0x20020000 /* SLC NAND controller */
#define CFG_ENV_IS_IN_NAND
#define LPC32XX_SLC_NAND_TIMING (SLCTAC_WDR(14) | \
                    SLCTAC_WWIDTH(5) | \
                    SLCTAC_WHOLD(2) | \
                    SLCTAC_WSETUP(1) | \
                    SLCTAC_RDR(14) | \
                    SLCTAC_RWIDTH(4) | \
                    SLCTAC_RHOLD(2) | \
                    SLCTAC_RSETUP(1))

/*
 * NAND H/W ECC specific settings
 */
#define CONFIG_SYS_LPC32XX_DMA            /* DMA support required */
#define CONFIG_SYS_NAND_ECCSIZE      2048 /* ECC generated per page */
#define CONFIG_SYS_NAND_ECCBYTES       24 /* 3 Bytes ECC per 256 Bytes */
#define CONFIG_SYS_NAND_OOBSIZE        64 /* OOB size in bytes */

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE

/*
 * Support for various capabilities
 */
#define CONFIG_AUTO_COMPLETE
#define CONFIG_CMDLINE_EDITING
#define CONFIG_SYS_LOADS_BAUD_CHANGE

/*
 * Network setup
 */
#define CONFIG_NETMASK		255.255.255.0
#define CONFIG_IPADDR		192.168.1.101
#define CONFIG_SERVERIP		192.168.1.41
#define CONFIG_ETHADDR		00:1a:f1:00:00:00
#define CONFIG_GATEWAYIP	192.168.1.1

#define CONFIG_BOOTFILE		uImage
#define CONFIG_LOADADDR		0x80100000
#define CONFIG_ROOTPATH		/home/user/ltib/rootfs

/* Boot arguments for JFFS2 root file system in NAND */
#define MTDROOTCOMMAND "mtdboot="				\
	"setenv bootargs "					\
	"console=ttyTX0,115200n8 "				\
	"root=/dev/mtdblock3 rw rootfstype=jffs2 "		\
	"ip=${ipaddr} ethaddr=${ethaddr}\0"

/* Boot arguments for NFS based root file system */
#define NFSROOTCOMMAND "nfsboot="				\
	"setenv bootargs "					\
	"console=ttyTX0,115200n8 "				\
	"root=/dev/nfs rw nfsroot=${serverip}:${rootpath} "	\
	"ip=${ipaddr} ethaddr=${ethaddr}\0"

/* Boot arguments for ramdisk image loaded via TFTP */
#define RDROOTCOMMAND "ramdiskboot="				\
	"setenv bootargs "					\
	"console=ttyTX0,115200n8 "				\
	"root=/dev/ram0 rw "					\
	"ip=${ipaddr} ethaddr=${ethaddr}\0"

/* Kernel boot using tftp with static IP */
#define TFTPSTATICIPKERNELBOOT "tftpstatickernel="		\
	"tftpboot ${loadaddr} ${serverip}:${bootfile}\0"

/* Kernel boot using tftp with DHCP obtained IP Address */
#define TFTPDHCPKERNELBOOT "tftpdhcpkernel=dhcp\0"

/*Kernel boot from NAND */
#define MTDKERNELBOOT "mtdkernel=nboot.jffs2 ${loadaddr} "	\
	"0 0x00140000\0"

/* Command to burn kernel image into NAND FLASH */
#define MTDKERNELBURN "mtdkernelburn="				\
	"nand erase 0x00140000 0x00400000;"			\
	"nand write.jffs2 ${loadaddr} 0x00140000 0x00400000\0"

/* Root filesystem image name and load address */
#define ROOTFSNAME "rootfile=rootfs.jffs2\0"
#define ROOTFSLOADADDR "rootloadaddr=0x81000000\0"
#define ROOTFSLOADSIZE "rootloadsize=0x01000000\0"

/* Load JFFS2 root file system using TFTP with static IP */
#define TFTPSTATICROOTFSLOAD "tftpstaticloadroot="		\
	"tftpboot ${rootloadaddr} ${serverip}:${rootfile}\0"

/* Load JFFS2 root file system using TFTP with DHCP */
#define TFTPDHCPROOTFSLOAD "tftpdhcploadroot="			\
	"dhcp ${rootloadaddr} ${serverip}:${rootfile}\0"

/* Command to load root file system into RAM (ramdisk) */
#define RDROOTFSLOAD "ramdiskload="				\
	"nand read.jffs2 ${rootloadaddr} 0x00540000 "		\
	"${rootloadsize}\0"

/* Command to burn root file system image into NAND FLASH */
#define MTDROOTBURN "mtdrootburn="				\
	"nand erase 0x00540000 0x0FAC0000;"			\
	"nand write.jffs2 ${rootloadaddr} 0x00540000"		\
	" ${rootloadsize}\0"

/*
 * Other preset environment variables and example bootargs string
 */
#define CONFIG_EXTRA_ENV_SETTINGS				\
	MTDROOTCOMMAND						\
	NFSROOTCOMMAND						\
	RDROOTCOMMAND						\
	TFTPSTATICIPKERNELBOOT					\
	TFTPDHCPKERNELBOOT					\
	MTDKERNELBOOT						\
	MTDKERNELBURN						\
	ROOTFSNAME						\
	ROOTFSLOADADDR						\
	ROOTFSLOADSIZE						\
	TFTPSTATICROOTFSLOAD					\
	TFTPDHCPROOTFSLOAD					\
	RDROOTFSLOAD						\
	MTDROOTBURN

/* Default boot command */
#define CONFIG_BOOTCOMMAND					\
	"run nfsboot;run tftpstatickernel; bootm ${loadaddr}"

/*
 * BOOTP options
 */
#define CONFIG_BOOTP_SUBNETMASK
#define CONFIG_BOOTP_GATEWAY
#define CONFIG_BOOTP_BOOTPATH
#define CONFIG_BOOTP_HOSTNAME
#define CONFIG_BOOTP_BOOTFILESIZE

#endif  /* __FDI3250_H__*/

