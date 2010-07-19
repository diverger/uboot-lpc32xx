/*
 * Copyright (C) 2009 by NXP Semiconductors
 * All rights reserved.
 *
 * @Author: Kevin Wells
 * @Descr: Phytec 3250 board configuration file
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
 * Phytec 3250 board configuation data
 */

#ifndef __PHY3250_H__
#define __PHY3250_H__

/*
 *
 * Chip speific options
 *
 */

/*
 * Ethernet buffer support in uncached IRAM and buffer size
 */
#define USE_IRAM_FOR_ETH_BUFFERS
#define IRAM_ETH_BUFF_BASE 0x08010000 /* Uncached IRAM */
#define IRAM_ETH_BUFF_SIZE 0x00010000

/*
 * There are 2 boot options for u-boot on the Phytec 3250 board. Option 1
 * or option 2. In either cases, u-boot does not need to be relocated.
 *
 * Option 1 - define CFG_BOOT_USES1L
 * With this option, the S1L loader present in the board initializes the
 * system (including SDRAM, MMUs, some MUX states, etc.). U-boot is loaded
 * into an already initialized system in SDRAM at address 0x83FC0000 (the
 * end of SDRAM in a 64M system). Because most of the system is already
 * initialized, system init is not performed again.
 *
 * Option 2 - undefine CFG_BOOT_USES1L
 * With this option, u-boot is the primary boot loader that is loaded and
 * started from the Phytec kickstart loader (see documentation with the
 * Phytec board for the kickstart loader). In this configuration, u-boot
 * loads and runs from RAM at address 0x00000000 and requires complete
 * system initialization. The kickstart loader will copy the u-boot image
 * from FLASH starting at block 1 into IRAM and start it at address 0x0.
 */
#define CFG_BOOT_USES1L

#ifdef CFG_BOOT_USES1L
/*
 * Skip low level init of MMU, SDRAM, muxing, etc. if u-boot is loaded
 * and executed from S1L
 */
#define CONFIG_SKIP_LOWLEVEL_INIT
#endif

/*
 * Linux machine type
 */
#define MACH_TYPE_PHY3250 (2511)
#define MACH_TYPE_UBOOTSYS MACH_TYPE_PHY3250

/*
 * System UART selection, valid selections include UART3, UART4,
 * UART5, and UART6
 */
#define CFG_UART_SEL UART5

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
#define CONFIG_ENV_SIZE		0x4000 /* 1 block */
#define CONFIG_ENV_OFFSET	0x168000 /* Block 90 */
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
#define CONFIG_MISC_INIT_R
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
#define CONFIG_SYS_MAX_NAND_DEVICE 1
#define CONFIG_SYS_NAND_BASE 0x20020000 /* SLC NAND controller */
#define CFG_ENV_IS_IN_NAND

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
#define CONFIG_NETMASK		   255.255.255.0
#define CONFIG_IPADDR		   192.168.1.193
#define CONFIG_SERVERIP		   192.168.1.45
#define CONFIG_GATEWAYIP	   192.168.1.1
#define CONFIG_NETMASK		   255.255.255.0
#define CONFIG_BOOTFILE		   "uImage"  /* File to load */
#define CONFIG_BOOTARGS		   "console=ttyS0,115200n8 root=/dev/nfs rw nfsroot=192.168.1.51:/home/user/ltib/rootfs ethaddr=${ethaddr} ip=192.168.1.193"
/*
 * BOOTP options
 */
#define CONFIG_BOOTP_SUBNETMASK
#define CONFIG_BOOTP_GATEWAY
#define CONFIG_BOOTP_BOOTPATH
#define CONFIG_BOOTP_HOSTNAME
#define CONFIG_BOOTP_BOOTFILESIZE

#endif  /* __PHY3250_H__*/

