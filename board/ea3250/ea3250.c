/*
 * Copyright (C) 2008 by NXP Semiconductors
 * All rights reserved.
 *
 * @Author: Based on code by Kevin Wells
 * @Descr: Embedded Artists LPC3250 OEM Board support functions
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
#include <configs/ea3250.h>
#include <asm/mach-types.h>
#include <lpc3250.h>
#include <net.h>
#include "ea3250_prv.h"
		
DECLARE_GLOBAL_DATA_PTR;
extern int dcache_kill(void);

/* ISP1301 USB transceiver I2C registers */
#define	ISP1301_MODE_CONTROL_1		0x04	/* u8 read, set, +1 clear */

#define	MC1_SPEED_REG		(1 << 0)
#define	MC1_SUSPEND_REG		(1 << 1)
#define	MC1_DAT_SE0		(1 << 2)
#define	MC1_TRANSPARENT		(1 << 3)
#define	MC1_BDIS_ACON_EN	(1 << 4)
#define	MC1_OE_INT_EN		(1 << 5)
#define	MC1_UART_EN		(1 << 6)
#define	MC1_MASK		0x7f

#define	ISP1301_MODE_CONTROL_2		0x12	/* u8 read, set, +1 clear */

#define	MC2_GLOBAL_PWR_DN	(1 << 0)
#define	MC2_SPD_SUSP_CTRL	(1 << 1)
#define	MC2_BI_DI		(1 << 2)
#define	MC2_TRANSP_BDIR0	(1 << 3)
#define	MC2_TRANSP_BDIR1	(1 << 4)
#define	MC2_AUDIO_EN		(1 << 5)
#define	MC2_PSW_EN		(1 << 6)
#define	MC2_EN2V7		(1 << 7)

#define	ISP1301_OTG_CONTROL_1		0x06	/* u8 read, set, +1 clear */
#	define	OTG1_DP_PULLUP		(1 << 0)
#	define	OTG1_DM_PULLUP		(1 << 1)
#	define	OTG1_DP_PULLDOWN	(1 << 2)
#	define	OTG1_DM_PULLDOWN	(1 << 3)
#	define	OTG1_ID_PULLDOWN	(1 << 4)
#	define	OTG1_VBUS_DRV		(1 << 5)
#	define	OTG1_VBUS_DISCHRG	(1 << 6)
#	define	OTG1_VBUS_CHRG		(1 << 7)
#define	ISP1301_OTG_STATUS		0x10	/* u8 readonly */
#	define	OTG_B_SESS_END		(1 << 6)
#	define	OTG_B_SESS_VLD		(1 << 7)

#define ISP1301_I2C_ADDR 0x2D

#define ISP1301_I2C_MODE_CONTROL_1 0x4
#define ISP1301_I2C_MODE_CONTROL_2 0x12
#define ISP1301_I2C_OTG_CONTROL_1 0x6
#define ISP1301_I2C_OTG_CONTROL_2 0x10
#define ISP1301_I2C_INTERRUPT_SOURCE 0x8
#define ISP1301_I2C_INTERRUPT_LATCH 0xA
#define ISP1301_I2C_INTERRUPT_FALLING 0xC
#define ISP1301_I2C_INTERRUPT_RISING 0xE
#define ISP1301_I2C_REG_CLEAR_ADDR 1


#define I2C_START_BIT (1 << 8)
#define I2C_STOP_BIT  (1 << 9)

#define I2C_READ      0x01
#define I2C_WRITE     0x00
#define DUMMY_BYTE    0x55

void reset_timer (void)
{
	unsigned int clkdlycnt, tbaseclk;

	/* Reset timer */
	TIMER_CNTR0->tcr = TIMER_CNTR_TCR_RESET;
	TIMER_CNTR0->tcr = 0;
	TIMER_CNTR0->tc = 0;

	/* Clear and enable match function */
	TIMER_CNTR0->ir = TIMER_CNTR_MTCH_BIT(0);

	/* Count mode is PCLK edge */
	TIMER_CNTR0->ctcr = TIMER_CNTR_SET_MODE(TIMER_CNTR_CTCR_TIMER_MODE);

	/* Set prescale counter value for a 1mS tick */
	tbaseclk = sys_get_rate(CLKPWR_PERIPH_CLK);
	clkdlycnt = (tbaseclk / CONFIG_SYS_HZ);
	TIMER_CNTR0->pr = clkdlycnt - 1;

	/* Enable the timer */
	TIMER_CNTR0->tcr = TIMER_CNTR_TCR_EN;
}

ulong get_timer (ulong base)
{
	ulong tcr = TIMER_CNTR0->tc;
	return tcr;
}

void timer_init(void)
{
	/* Enable timer system clock */
	CLKPWR->clkpwr_timers_pwms_clk_ctrl_1 |=
		CLKPWR_TMRPWMCLK_TIMER0_EN;

	reset_timer();
}

/*
 * Use the watchdog timer to reset the system
 */
void reset_cpu(ulong addr)
{
	/* Enable the WDT clock */
	CLKPWR->clkpwr_timer_clk_ctrl |= CLKPWR_PWMCLK_WDOG_EN;

	/* Force RESETOUT_N active */
	WDT->wdtim_mctrl = WDT_RESFRC1;
	WDT->wdtim_ctrl = WDT_COUNT_ENAB;

	/* Wait forever */
	while (1);
}

void udelay(unsigned long usec)
{
	unsigned long tbaseclk;

	/* Enable timer system clock */
	CLKPWR->clkpwr_timers_pwms_clk_ctrl_1 |=
		CLKPWR_TMRPWMCLK_TIMER1_EN;

	/* Reset timer */
	TIMER_CNTR1->tcr = TIMER_CNTR_TCR_RESET;
	TIMER_CNTR1->tcr = 0;
	TIMER_CNTR1->tc = 0;
	TIMER_CNTR1->pr = 0;

	/* Clear and enable match function */
	TIMER_CNTR1->ir = TIMER_CNTR_MTCH_BIT(0);

	/* Count mode is PCLK edge */
	TIMER_CNTR1->ctcr = TIMER_CNTR_SET_MODE(TIMER_CNTR_CTCR_TIMER_MODE);

	/* Set prescale counter value for a 1uS tick */
	tbaseclk = sys_get_rate(CLKPWR_PERIPH_CLK);
	tbaseclk = (tbaseclk / 1000000) - 1;
	TIMER_CNTR1->pr = tbaseclk;

	/* Set match for number of usecs */
	TIMER_CNTR1->mr[0] = usec;

	/* Interrupt on match 0 */
	TIMER_CNTR1->mcr = TIMER_CNTR_MCR_MTCH(0);

	/* Enable the timer */
	TIMER_CNTR1->tcr = TIMER_CNTR_TCR_EN;

	/* Loop until match occurs */
	while ((TIMER_CNTR1->ir & TIMER_CNTR_MTCH_BIT(0)) == 0);

	/* Stop timer */
	TIMER_CNTR1->tcr = 0;

	/* Disable timer system clock */
	CLKPWR->clkpwr_timers_pwms_clk_ctrl_1 &=
		~CLKPWR_TMRPWMCLK_TIMER1_EN;
}

/*
 * Miscellaneous platform dependent initialisations
 */
int board_init (void)
{
	/* Kill data cache */
	dcache_kill();

	/* arch number of Logic-Board - MACH_TYPE_LPC3XXX */
	gd->bd->bi_arch_number = MACH_TYPE_UBOOTSYS;

	/* adress of boot parameters */
	gd->bd->bi_boot_params = CONFIG_ENV_ADDR;

	return 0;
}

int dram_init (void)
{
	gd->bd->bi_dram[0].start = PHYS_SDRAM_1;

	/* 64M */
	gd->bd->bi_dram[0].size = 0x04000000; /* 64M */

	return 0;
}


static int i2c_wait_reset(int timeout)
{
	while(timeout > 0 && (OTG->otg_i2c.otg_i2c_ctrl & I2C_RESET))
	{
		udelay(1000);
		timeout--;
	}

	return (timeout <= 0);	
}


static int isp1301_get_value(int reg)
{
	int n = 0;

	/* send isp1301 address */
	OTG->otg_i2c.otg_i2c_txrx = ((ISP1301_I2C_ADDR << 1) | I2C_START_BIT);   

	/* register (offset) to read from */
	OTG->otg_i2c.otg_i2c_txrx = reg;   

	/* repeat start */
	OTG->otg_i2c.otg_i2c_txrx = ((ISP1301_I2C_ADDR << 1) | I2C_START_BIT | I2C_READ);
	OTG->otg_i2c.otg_i2c_txrx = (I2C_STOP_BIT | DUMMY_BYTE);

	while (((OTG->otg_i2c.otg_i2c_stat & I2C_TDI) != I2C_TDI) && n++ < 100000);

	if (n>= 100000)
	{
		printf("isp1301_get_value: ERROR TDI not set\n");
	}

	/* clear TDI */
	OTG->otg_i2c.otg_i2c_stat |= I2C_TDI;

	return (OTG->otg_i2c.otg_i2c_txrx & 0xff);
}

static int isp1301_set_value(int reg, int value)
{
	int n = 0;

	/* send isp1301 address */
	OTG->otg_i2c.otg_i2c_txrx = ((ISP1301_I2C_ADDR << 1) | I2C_START_BIT);   

	/* offset to write to */
	OTG->otg_i2c.otg_i2c_txrx = (reg | I2C_WRITE);   
	/* value to write */
	OTG->otg_i2c.otg_i2c_txrx = (value | I2C_STOP_BIT);  
  
	/* wait for transmit done (TDI) */
	while (((OTG->otg_i2c.otg_i2c_stat & I2C_TDI) != I2C_TDI) && n++ < 100000);

	if (n>= 100000)
	{
		printf("isp1301_set_value: ERROR TDI not set\n");
		return -1;		
	}
  
	/* clear TDI */
	OTG->otg_i2c.otg_i2c_stat |= I2C_TDI;     

	return 0;
}

static void isp1301_configure(void)
{
	OTG->otg_i2c.otg_i2c_clk_hi = 0x3f;
	OTG->otg_i2c.otg_i2c_clk_lo = 0x3f; 

	OTG->otg_i2c.otg_i2c_ctrl |= I2C_RESET;	
	i2c_wait_reset(100);

	/* LPC32XX only supports DAT_SE0 USB mode */
	/* This sequence is important */

	/* Disable transparent UART mode first */
	isp1301_set_value((ISP1301_I2C_MODE_CONTROL_1 |
		ISP1301_I2C_REG_CLEAR_ADDR), MC1_UART_EN);

	isp1301_set_value((ISP1301_I2C_MODE_CONTROL_1 |
		ISP1301_I2C_REG_CLEAR_ADDR), ~MC1_SPEED_REG);
	isp1301_set_value(ISP1301_I2C_MODE_CONTROL_1, MC1_SPEED_REG);
	isp1301_set_value((ISP1301_I2C_MODE_CONTROL_2 | ISP1301_I2C_REG_CLEAR_ADDR), ~0);
	isp1301_set_value(ISP1301_I2C_MODE_CONTROL_2, 
		(MC2_BI_DI | MC2_PSW_EN | MC2_SPD_SUSP_CTRL));

	isp1301_set_value((ISP1301_I2C_OTG_CONTROL_1 | ISP1301_I2C_REG_CLEAR_ADDR), ~0);
	isp1301_set_value(ISP1301_I2C_MODE_CONTROL_1, MC1_DAT_SE0);
	isp1301_set_value(ISP1301_I2C_OTG_CONTROL_1, 
		(OTG1_DM_PULLDOWN | OTG1_DP_PULLDOWN));
	isp1301_set_value((ISP1301_I2C_OTG_CONTROL_1 | ISP1301_I2C_REG_CLEAR_ADDR),
		(OTG1_DM_PULLUP | OTG1_DP_PULLUP));
	isp1301_set_value((ISP1301_I2C_INTERRUPT_LATCH | ISP1301_I2C_REG_CLEAR_ADDR), ~0);
	isp1301_set_value((ISP1301_I2C_INTERRUPT_FALLING | ISP1301_I2C_REG_CLEAR_ADDR), ~0);
	isp1301_set_value((ISP1301_I2C_INTERRUPT_RISING | ISP1301_I2C_REG_CLEAR_ADDR), ~0);

	/* Enable usb_need_clk clock after transceiver is initialized */
	CLKPWR->clkpwr_usb_ctrl |= (CLKPWR_USBCTRL_USBDVND_EN);


	printf("\nISP1301 Vendor ID  : 0x%04x\n", 
		(isp1301_get_value(0x00) | (isp1301_get_value(0x01) << 8)));
	printf("ISP1301 Product ID : 0x%04x\n", 
		(isp1301_get_value(0x02) | (isp1301_get_value(0x03) << 8)));
	printf("ISP1301 Version ID : 0x%04x\n", 
		(isp1301_get_value(0x14) | (isp1301_get_value(0x15) << 8)));
	
}

static void usbpll_setup(void)
{
	int n = 0;

	/* make sure clocks are disabled */
	CLKPWR->clkpwr_usb_ctrl &= ~(CLKPWR_USBCTRL_CLK_EN1 | CLKPWR_USBCTRL_CLK_EN2);


	/* start PLL clock input */
	CLKPWR->clkpwr_usb_ctrl |= CLKPWR_USBCTRL_CLK_EN1;


	/* 
         * Setup PLL.  
         *
         */
	CLKPWR->clkpwr_usb_ctrl |= CLKPWR_USBCTRL_FDBK_PLUS1(192-1);
	CLKPWR->clkpwr_usb_ctrl |= CLKPWR_USBCTRL_POSTDIV_2POW(0x01);
	CLKPWR->clkpwr_usb_ctrl |= CLKPWR_USBCTRL_PLL_PWRUP;
 
	
	while ((CLKPWR->clkpwr_usb_ctrl & CLKPWR_USBCTRL_PLL_STS) == 0)
	{
		if (n++ >= 100000)
		{
			printf("usbpll_setup: ERROR PLL doesn't lock\n");
			break;
		}
	}

	/* enable PLL output */
	CLKPWR->clkpwr_usb_ctrl |= CLKPWR_USBCTRL_CLK_EN2;		
}


int usb_board_init(void)
{
	/* enable AHB slave USB clock */
	CLKPWR->clkpwr_usb_ctrl |= 
		(CLKPWR_USBCTRL_HCLK_EN | CLKPWR_USBCTRL_BUS_KEEPER);

	/* enable I2C clock in OTG block if it isn't */
	if ((OTG->otg_clk_sts & OTG_CLK_I2C_EN) != OTG_CLK_I2C_EN)
	{
		OTG->otg_clk_ctrl = OTG_CLK_I2C_EN;

		while (OTG->otg_clk_sts != OTG_CLK_I2C_EN);
	}

	/* Configure ISP1301 */
	isp1301_configure();

	/* setup USB clocks and PLL */
	usbpll_setup();

	/* enable usb_host_need_clk */
	CLKPWR->clkpwr_usb_ctrl |= CLKPWR_USBCTRL_USBHSTND_EN;

	/* enable all needed USB clocks */
	OTG->otg_clk_ctrl = (OTG_CLK_AHB_EN|OTG_CLK_OTG_EN
		|OTG_CLK_I2C_EN|OTG_CLK_HOST_EN);

	while ((OTG->otg_clk_ctrl & (OTG_CLK_AHB_EN|OTG_CLK_OTG_EN
		|OTG_CLK_I2C_EN|OTG_CLK_HOST_EN)) !=
	       (OTG_CLK_AHB_EN|OTG_CLK_OTG_EN
		|OTG_CLK_I2C_EN|OTG_CLK_HOST_EN)) ;
	
	OTG->otg_sts_ctrl |= OTG_HOST_EN;
	isp1301_set_value(ISP1301_I2C_OTG_CONTROL_1, OTG1_VBUS_DRV);

	return 0;
}

int usb_board_stop(void)
{
	/* vbus off */
	isp1301_set_value(
		(ISP1301_I2C_OTG_CONTROL_1| ISP1301_I2C_REG_CLEAR_ADDR), 
		OTG1_VBUS_DRV);

	OTG->otg_sts_ctrl &= ~OTG_HOST_EN;

	CLKPWR->clkpwr_usb_ctrl &= ~CLKPWR_USBCTRL_HCLK_EN;

	return 0;
}

int usb_board_init_fail(void)
{
	usb_board_stop();

	return 0;
}
