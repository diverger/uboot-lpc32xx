/***********************************************************************
 *
 * Copyright (C) 2008 by NXP Semiconductors
 * All rights reserved.
 *
 * @Author: Kevin Wells
 * @Descr: Definitions for LPC3250 chip
 * @References: NXP LPC3250 User's Guide
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
 *
 ***********************************************************************/

#ifndef __LPC3250_H
#define __LPC3250_H

/*
 * AHB physical address bases used in u-boot
 */
#define SSP0_BASE       0x20084000 	/* SSP0 registers base */
#define MLC_BASE	0x200A8000	/* MLC NAND Flash registers base*/
#define ETHERNET_BASE	0x31060000	/* Ethernet ctrl register base*/
#define EMC_BASE        0x31080000 	/* EMC registers base*/
#define SLC_BASE	0x20020000	/* SLC NAND Flash registers base*/
#define SD_BASE		0x20098000	/* SD card interface registers base*/
#define DMA_BASE	0x31000000	/* DMA controller registers base*/

/*
 * FAB physical address bases used in u-boot
 */
#define TIMER0_BASE     0x40044000
#define TIMER1_BASE     0x4004C000
#define CLK_PM_BASE	0x40004000	/* System control registers base*/
#define WDTIM_BASE	0x4003C000	/* Watchdog timer registers base*/
#define UART_CTRL_BASE	0x40054000	/* general UART ctrl regs base*/
#define GPIO_BASE	0x40028000	/* GPIO registers base*/

/*
 * APB physical address bases used in u-boot
 */
#define UART3_BASE	0x40080000	/* UART 3 registers base*/
#define UART4_BASE	0x40088000	/* UART 4 registers base*/
#define UART5_BASE	0x40090000	/* UART 5 registers base*/
#define UART6_BASE	0x40098000	/* UART 6 registers base*/

/*
 * Internal memory physical address bases used in u-boot
 */
#define IRAM_BASE       0x08000000  	/* Internal RAM address */

/*
 * External Static Memory Bank Address Space physical address bases
 */
#define EMC_CS0_BASE	0xE0000000
#define EMC_CS1_BASE	0xE1000000
#define EMC_CS2_BASE	0xE2000000
#define EMC_CS3_BASE	0xE3000000

/*
 * External SDRAM Memory Bank Address Space Bases
 */
#define EMC_DYCS0_BASE	0x80000000  	/* SDRAM DYCS0 base address */
#define EMC_DYCS1_BASE	0xA0000000  	/* SDRAM DYCS1 base address */

/*
 * Clock and crystal information
 */
#define MAIN_OSC_FREQ	13000000
#define CLOCK_OSC_FREQ	32768

/* bit position macro */
#define _BIT(n)		(0x1 << (n))

#define _SBF(f,v) (((unsigned long)(v)) << (f))

/*
 * Standard UART register structures
 */
typedef struct
{
	volatile unsigned int dll_fifo;     /* UART data FIFO/holding/dll reg */
	volatile unsigned int dlm_ier;      /* UART high divisor/int enable reg */
	volatile unsigned int iir_fcr;      /* UART int pending/FIFO contrl reg */
	volatile unsigned int lcr;          /* UART line control reg */
	volatile unsigned int modem_ctrl;   /* UART modem control reg */
	volatile unsigned int lsr;          /* UART line status reg */
	volatile unsigned int modem_status; /* UART modem status reg */
	volatile unsigned int rxlev;        /* UART RX FIFO level reg */
} UART_REGS_T;

/*
 * UART control structure
 */
typedef struct
{
	volatile unsigned int ctrl;         /* General UART control register */
	volatile unsigned int clkmode;      /* UART clock control register */
	volatile unsigned int loop;         /* UART loopmode enable/disable */
} UART_CNTL_REGS_T;

/*
 * UART dll and dlm register definitions
 */
#define UART_LOAD_DM(div)          ((div) & 0xFF)

/*
 * UART ier register definitions
 */
/* Bit for enabling the modem status interrupt */
#define UART_IER_MODEM_STS         _BIT(3)
/* Bit for enabling the RX line status interrupt(s) */
#define UART_IER_RXLINE_STS        _BIT(2)
/* Bit for enabling the transmit holding register empty interrupt */
#define UART_IER_THRE              _BIT(1)
/* Bit for enabling the receive data available (RDA) interrupt */
#define UART_IER_RDA               _BIT(0)

/*
 * UART iir register definitions
 */
/* Bit for masking interrupt pending status */
#define UART_IIR_INT_PENDING       _BIT(0)
/* Mask for getting interrupt source */
#define UART_IIR_INT_MASK          0xE
/* Interrupt sources */
#define UART_IIR_MODEM_STS         0x0
#define UART_IIR_INTSRC_THRE       0x2
#define UART_IIR_INTSRC_RDA        0x4
#define UART_IIR_INTSRC_RXLINE     0x6
#define UART_IIR_INTSRC_CTI        0xC /* Character timeout */
/* Interrupt bits mask word */
#define UART_IIR_INTSRC_MASK       0xE

/*
 * UART fcr register definitions
 */
/* Receive FIFO trigger level selections */
#define UART_FCR_RXFIFO_TL16       0x0
#define UART_FCR_RXFIFO_TL32       _BIT(6)
#define UART_FCR_RXFIFO_TL48       _BIT(7)
#define UART_FCR_RXFIFO_TL60       (_BIT(7) | _BIT(6))
/* Transmit FIFO trigger level selections */
#define UART_FCR_TXFIFO_TL0        0x0
#define UART_FCR_TXFIFO_TL4        _BIT(4)
#define UART_FCR_TXFIFO_TL8        _BIT(5)
#define UART_FCR_TXFIFO_TL16       (_BIT(5) | _BIT(4))
/* Enable FIFO bit - must be set with UART_FCR_FIFO_EN */
#define UART_FCR_FIFO_CTRL         _BIT(3)
/* Clear TX FIFO bit */
#define UART_FCR_TXFIFO_FLUSH      _BIT(2)
/* Clear RX FIFO bit */
#define UART_FCR_RXFIFO_FLUSH      _BIT(1)
/* Enable FIFO bit - must be set with UART_FCR_FIFO_CTRL */
#define UART_FCR_FIFO_EN           _BIT(0)

/*
 * UART lcr register definitions
 */
/* Bit for enabling divisor latch and IER register */
#define UART_LCR_DIVLATCH_EN       _BIT(7)
/* Bit for enabling break transmission (forces TX low) */
#define UART_LCR_BREAK_EN          _BIT(6)
/* Parity selection */
#define UART_LCR_PARITY_ODD        0x0
#define UART_LCR_PARITY_EVEN       _BIT(4)
#define UART_LCR_PARITY_FORCE1     _BIT(5)
#define UART_LCR_PARITY_FORCE0     (_BIT(5) | _BIT(4))
/* Parity selection mask */
#define UART_LCR_PARITY_MASK       (_BIT(5) | _BIT(4))
/* Parity enable bit */
#define UART_LCR_PARITY_ENABLE     _BIT(3)
/* Stop bit selection */
#define UART_LCR_STOP1BIT          0x0
#define UART_LCR_STOP2BITS         _BIT(2)
/* Word length selections */
#define UART_LCR_WLEN_5BITS        0x0
#define UART_LCR_WLEN_6BITS        _BIT(0)
#define UART_LCR_WLEN_7BITS        _BIT(1)
#define UART_LCR_WLEN_8BITS        (_BIT(1) | _BIT(0))
/* Word length mask */
#define UART_LCR_WLEN_MASK         (_BIT(1) | _BIT(0))

/*
 * UART modem_ctrl register definitions
 */
/* Bit for enabling modem loopback mode */
#define UART_MDMC_LOOPB_EN         _BIT(4)
/* Bit for driving RTS low */
#define UART_MDMC_RTS_LOW          _BIT(1)
/* Bit for driving DTR low */
#define UART_MDMC_DTR_LOW          _BIT(0)

/*
 * UART lsr register definitions
 */
/* Bit for masking FIFO RX error status */
#define UART_LSR_FIFORX_ERR        _BIT(7)
/* Bit for masking transmitter empty status */
#define UART_LSR_TEMT              _BIT(6)
/* Bit for masking transmit FIFO trip point status */
#define UART_LSR_THRE              _BIT(5)
/* Bit for masking break interrupt status */
#define UART_LSR_BI                _BIT(4)
/* Bit for masking framing error status */
#define UART_LSR_FR                _BIT(3)
/* Bit for masking parity error status */
#define UART_LSR_PE                _BIT(2)
/* Bit for masking RX FIFO overrun error status */
#define UART_LSR_OE                _BIT(1)
/* Bit for masking RX FIFO empty status */
#define UART_LSR_RDR               _BIT(0)

/*
 * UART modem_status register definitions
 */
/* Bit for masking data carrier detect state */
#define UART_MDMS_DCD              _BIT(7)
/* Bit for masking ring indicator state */
#define UART_MDMS_RI               _BIT(6)
/* Bit for masking data set ready state */
#define UART_MDMS_DSR              _BIT(5)
/* Bit for masking clear to send state */
#define UART_MDMS_CTS              _BIT(4)
/* Bit for detecting state change on DCD */
#define UART_MDMS_DCD_CHG          _BIT(3)
/* Bit for detecting state change on RI */
#define UART_MDMS_RI_CHG           _BIT(2)
/* Bit for detecting state change on DSR */
#define UART_MDMS_DSR_CHG          _BIT(1)
/* Bit for detecting state change on CTS */
#define UART_MDMS_CTS_CHG          _BIT(0)

/*
 * UART rxlev register definitions
 */
/* Macro for masking off the receive FIFO level */
#define UART_RXLEV(n)              ((n) & 0x7F)

/*
 * UART ctrl register definitions
*/
/* UART3 modem control pin enable bit */
#define UART_U3_MD_CTRL_EN         _BIT(11)
/* IRRX6 inversion enable bit */
#define UART_IRRX6_INV_EN          _BIT(10)
/* IRRX6 RX mask while TX enabled enable bit */
#define UART_HDPX_EN               _BIT(9)
/* UART6 IRA modulator bypass bit */
#define UART_UART6_IRDAMOD_BYPASS  _BIT(5)
/* IRTX6 inversion enable bit */
#define RT_IRTX6_INV_EN            _BIT(4)
/* IRRX6 inversion enable bit */
#define RT_IRTX6_INV_MIR_EN        _BIT(3)
/* IR RX length, 3/16th pulse length with a 115Kbps clock */
#define RT_RX_IRPULSE_3_16_115K    _BIT(2)
/* IR TX length, 3/16th pulse length with a 115Kbps clock */
#define RT_TX_IRPULSE_3_16_115K    _BIT(1)
/* UART5 mirror route to the USB D+ and D- pins bit */
#define UART_U5_ROUTE_TO_USB       _BIT(0)

/*
 * UART clkmode register definitions
 */
/* Macro return the UART clock enabled field, shifted */
#define UART_ENABLED_CLOCKS(n)     (((n) >> 16) & 0x7F)
/* Macro returning a selected enabled UART clock bit */
#define UART_ENABLED_CLOCK(n, u)   (((n) >> (16 + (u))) & 0x1)
/* Bit that indicates if any UARTS are being clocked */
#define UART_ENABLED_CLKS_ANY      _BIT(14)
/* Defnies for setting a IARTs clock mode */
#define UART_CLKMODE_OFF           0x0    /* Clocks are off */
#define UART_CLKMODE_ON            0x1    /* Clocks are on */
#define UART_CLKMODE_AUTO          0x2    /* Clocks are automatic */
/* Clock mode mask for a UART, for UARTs 6 to 3 only */
#define UART_CLKMODE_MASK(u)      (0x3 << ((((u) - 3) * 2) + 4))
/* Macro for loading a UARTs clock mode, for UARTs 6 to 3 only */
#define UART_CLKMODE_LOAD(m, u)   ((m) << ((((u) - 3) * 2) + 4))

/*
 * Macros pointing to UART base and control registers
 */
#define UART3 ((UART_REGS_T *)(UART3_BASE))
#define UART4 ((UART_REGS_T *)(UART4_BASE))
#define UART5 ((UART_REGS_T *)(UART5_BASE))
#define UART6 ((UART_REGS_T *)(UART6_BASE))
#define UARTCNTL ((UART_CNTL_REGS_T *) (UART_CTRL_BASE))












/**********************************************************************
* Clock and Power control register structures
**********************************************************************/

/*
 * Clock and Power control module register structure
 */
typedef struct
{
  volatile unsigned int reserved1 [5];
  volatile unsigned int clkpwr_bootmap;
  volatile unsigned int clkpwr_p01_er;
  volatile unsigned int clkpwr_usbclk_pdiv;
  volatile unsigned int clkpwr_int_er;
  volatile unsigned int clkpwr_int_rs;
  volatile unsigned int clkpwr_int_sr;
  volatile unsigned int clkpwr_int_ap;
  volatile unsigned int clkpwr_pin_er;
  volatile unsigned int clkpwr_pin_rs;
  volatile unsigned int clkpwr_pin_sr;
  volatile unsigned int clkpwr_pin_ap;
  volatile unsigned int clkpwr_hclk_div;
  volatile unsigned int clkpwr_pwr_ctrl;
  volatile unsigned int clkpwr_pll397_ctrl;
  volatile unsigned int clkpwr_main_osc_ctrl;
  volatile unsigned int clkpwr_sysclk_ctrl;
  volatile unsigned int clkpwr_lcdclk_ctrl;
  volatile unsigned int clkpwr_hclkpll_ctrl;
  volatile unsigned int reserved2;
  volatile unsigned int clkpwr_adc_clk_ctrl_1;
  volatile unsigned int clkpwr_usb_ctrl;
  volatile unsigned int clkpwr_sdramclk_ctrl;
  volatile unsigned int clkpwr_ddr_lap_nom;
  volatile unsigned int clkpwr_ddr_lap_count;
  volatile unsigned int clkpwr_ddr_cal_delay;
  volatile unsigned int clkpwr_ssp_blk_ctrl;
  volatile unsigned int clkpwr_i2s_clk_ctrl;
  volatile unsigned int clkpwr_ms_ctrl;
  volatile unsigned int reserved4 [3];
  volatile unsigned int clkpwr_macclk_ctrl;
  volatile unsigned int reserved5 [4];
  volatile unsigned int clkpwr_test_clk_sel;
  volatile unsigned int clkpwr_sw_int;
  volatile unsigned int clkpwr_i2c_clk_ctrl;
  volatile unsigned int clkpwr_key_clk_ctrl;
  volatile unsigned int clkpwr_adc_clk_ctrl;
  volatile unsigned int clkpwr_pwm_clk_ctrl;
  volatile unsigned int clkpwr_timer_clk_ctrl;
  volatile unsigned int clkpwr_timers_pwms_clk_ctrl_1;
  volatile unsigned int clkpwr_spi_clk_ctrl;
  volatile unsigned int clkpwr_nand_clk_ctrl;
  volatile unsigned int reserved7;
  volatile unsigned int clkpwr_uart3_clk_ctrl;
  volatile unsigned int clkpwr_uart4_clk_ctrl;
  volatile unsigned int clkpwr_uart5_clk_ctrl;
  volatile unsigned int clkpwr_uart6_clk_ctrl;
  volatile unsigned int clkpwr_irda_clk_ctrl;
  volatile unsigned int clkpwr_uart_clk_ctrl;
  volatile unsigned int clkpwr_dmaclk_ctrl;
  volatile unsigned int clkpwr_autoclock;
} CLKPWR_REGS_T;


/*
 * clkpwr_hclk_div register definitions
 */
/* HCLK Divider DDRAM clock stop (used for SDRAM only) */
#define CLKPWR_HCLKDIV_DDRCLK_STOP (0x0 << 7)
/* HCLK Divider DDRAM clock is the same speed as the ARM */
#define CLKPWR_HCLKDIV_DDRCLK_NORM (0x1 << 7)
/* HCLK Divider DDRAM clock is half the speed as the ARM */
#define CLKPWR_HCLKDIV_DDRCLK_HALF (0x2 << 7)
/* HCLK Divider PERIPH_CLK divider, for a value of n, the divider is
   (1+n), maximum value of n is 32 */
#define CLKPWR_HCLKDIV_PCLK_DIV(n) (((n) & 0x1F) << 2)
/* HCLK Divider, for a value of n, the divider is (2^n), maximum
   value of n is 2 for a divider of 4 */
#define CLKPWR_HCLKDIV_DIV_2POW(n) ((n) & 0x3)

/*
 * clkpwr_pwr_ctrl register definitions
 */
/* Force HCLK and ARMCLK to run from PERIPH_CLK to save power */
#define CLKPWR_CTRL_FORCE_PCLK      _BIT(10)
/* SDRAM self refresh request */
#define CLKPWR_SDRAM_SELF_RFSH      _BIT(9)
/* Update SDRAM self refresh request */
#define CLKPWR_UPD_SDRAM_SELF_RFSH  _BIT(8)
/* Enable auto exit SDRAM self refresh */
#define CLKPWR_AUTO_SDRAM_SELF_RFSH _BIT(7)
/* Highcore pin level (when CLKPWR_HIGHCORE_GPIO_EN is set) */
#define CLKPWR_HIGHCORE_STATE_BIT   _BIT(5)
/* SYSCLKEN pin level (when CLKPWR_SYSCLKEN_GPIO_EN is set) */
#define CLKPWR_SYSCLKEN_STATE_BIT   _BIT(4)
/* Enable SYSCLKEN pin as a GPIO bit */
#define CLKPWR_SYSCLKEN_GPIO_EN     _BIT(3)
/* Selects direct run mode (0) or run mode (1) */
#define CLKPWR_SELECT_RUN_MODE      _BIT(2)
/* Enable Highcore pin as a GPIO bit */
#define CLKPWR_HIGHCORE_GPIO_EN     _BIT(1)
/* Enable Highcore pin as a GPIO bit */
#define CLKPWR_STOP_MODE_CTRL       _BIT(0)

/*
 * clkpwr_sysclk_ctrl register definitions
 */
/* Number used by the clock switching circuitry to decide how long a
   bad phase must be present before clock switching is triggered */
#define CLKPWR_SYSCTRL_BP_TRIG(n)   (((n) & 0x3FF) << 2)
/* Mask for bad phase bits */
#define CLKPWR_SYSCTRL_BP_MASK      (0x3FF << 2)
/* (1) = Use main oscillator, (1) = use PLL397 oscillator */
#define CLKPWR_SYSCTRL_USEPLL397    _BIT(1)
/* Read only status mask bit of the select oscillator, (0) = main
   oscillator, (1) = PLL397 oscillator */
#define CLKPWR_SYSCTRL_SYSCLKMUX    _BIT(0)

/*
 * clkpwr_hclkpll_ctrl register definitions
 */
/* Bit to start (1) or stop (0) the main HCLK PLL */
#define CLKPWR_HCLKPLL_POWER_UP    _BIT(16)
/* Main HCLK PLL CCO bypass control (0) = CCO clock to post divider,
   (1) = Bypass CCO and route PLL clock to post divider */
#define CLKPWR_HCLKPLL_CCO_BYPASS  _BIT(15)
/* Main HCLK PLL post divider bypass control (0) = use post divider,
   (1) = Bypass post divider */
#define CLKPWR_HCLKPLL_POSTDIV_BYPASS _BIT(14)
/* Main HCLK PLL feedback divider path control, (0) = feedback
   divider clocked by CCO, (1) = feedback divider clocked by FCLKOUT */
#define CLKPWR_HCLKPLL_FDBK_SEL_FCLK _BIT(13)
/* Main HCLK PLL post divider setting, for a value of n, the divider
   is 2^n, maximum value of n is 3 */
#define CLKPWR_HCLKPLL_POSTDIV_2POW(n) (((n) & 0x3) << 11)
/* Main HCLK PLL pre divider setting, for a value of n, the divider
   is (1+n), maximum value of n is 3 */
#define CLKPWR_HCLKPLL_PREDIV_PLUS1(n) (((n) & 0x3) << 9)
/* Main HCLK PLL feedback setting, for a value of n, the feedback
   is (1+n), maximum value of n is 255 */
#define CLKPWR_HCLKPLL_PLLM(n)     (((n) & 0xFF) << 1)
/* Read only status mask bit of the PLL lock state, (0) = PLL is not
   locked, (1) = PLL is locked */
#define CLKPWR_HCLKPLL_PLL_STS     _BIT(0)

/*
 * clkpwr_sdramclk_ctrl register definitions
 */
/* SDRAM RAM_CLK fast slew rate control selection bit */
#define CLKPWR_SDRCLK_FASTSLEW_CLK _BIT(22)
/* SDRAM grouping fast slew rate control selection bit */
#define CLKPWR_SDRCLK_FASTSLEW     _BIT(21)
/* SDRAM data fast slew rate control selection bit */
#define CLKPWR_SDRCLK_FASTSLEW_DAT _BIT(20)
/* SDRAM/DDR controller reset bit */
#define CLKPWR_SDRCLK_SW_DDR_RESET _BIT(19)
/* Select HCLK delay calibration value, n = 0 to 31 at .25nS per tick */
#define CLKPWR_SDRCLK_HCLK_DLY(n)  (((n) & 0x1F) << 14)
/* SDRAM/DDR delay circuitry address status bit */
#define CLKPWR_SDRCLK_DLY_ADDR_STS _BIT(13)
/* Sensitivity factor for DDR SDRAM cal, n = 0 to 7 */
#define CLKPWR_SDRCLK_SENS_FACT(n) (((n) & 0x7) << 10)
/* Use calibrated settings for DDR SDRAM bit */
#define CLKPWR_SDRCLK_USE_CAL      _BIT(9)
/* Perform a DDR delay calibration bit */
#define CLKPWR_SDRCLK_DO_CAL       _BIT(8)
/* Enable auto DDR cal on RTC tick bit */
#define CLKPWR_SDRCLK_CAL_ON_RTC   _BIT(7)
/* Select DQS input delay value, n = 0 to 31 at .25nS per tick */
#define CLKPWR_SDRCLK_DQS_DLY(n)   (((n) & 0x1F) << 2)
/* Use DDR (1) or SDRAM (0) bit */
#define CLKPWR_SDRCLK_USE_DDR      _BIT(1)
/* SDRAM/DDR clock disable bit */
#define CLKPWR_SDRCLK_CLK_DIS      _BIT(0)

/**********************************************************************
* clkpwr_timers_pwms_clk_ctrl_1 register definitions
**********************************************************************/
/* Timer 3 clock enable, (0) = disable, (1) = enable */
#define CLKPWR_TMRPWMCLK_TIMER3_EN 0x20
/* Timer 2 clock enable, (0) = disable, (1) = enable */
#define CLKPWR_TMRPWMCLK_TIMER2_EN 0x10
/* Timer 1 clock enable, (0) = disable, (1) = enable */
#define CLKPWR_TMRPWMCLK_TIMER1_EN 0x08
/* Timer 0 clock enable, (0) = disable, (1) = enable */
#define CLKPWR_TMRPWMCLK_TIMER0_EN 0x04
/* PWM 4 clock enable, (0) = disable, (1) = enable */
#define CLKPWR_TMRPWMCLK_PWM4_EN   0x02
/* PWM 3 clock enable, (0) = disable, (1) = enable */
#define CLKPWR_TMRPWMCLK_PWM3_EN   0x01

/*
 * clkpwr_uart3_clk_ctrl, clkpwr_uart4_clk_ctrl, clkpwr_uart5_clk_ctrl
 * and clkpwr_uart6_clk_ctrl register definitions
 */
/* Macro for loading UART 'Y' divider value */
#define CLKPWR_UART_Y_DIV(y)       ((y) & 0xFF)
/* Macro for loading UART 'X' divider value */
#define CLKPWR_UART_X_DIV(x)       (((x) & 0xFF) << 8)
/* Bit for using HCLK as the UART X/Y divider input, or PERIPH_CLK */
#define CLKPWR_UART_USE_HCLK       _BIT(16)

/*
 * clkpwr_uart_clk_ctrl register definitions
 */
/* UART6 clock disable (0) / enable (1) bit */
#define CLKPWR_UARTCLKCTRL_UART6_EN _BIT(3)
/* UART5 clock disable (0) / enable (1) bit */
#define CLKPWR_UARTCLKCTRL_UART5_EN _BIT(2)
/* UART4 clock disable (0) / enable (1) bit */
#define CLKPWR_UARTCLKCTRL_UART4_EN _BIT(1)
/* UART3 clock disable (0) / enable (1) bit */
#define CLKPWR_UARTCLKCTRL_UART3_EN _BIT(0)

/**********************************************************************
* clkpwr_ssp_blk_ctrl register definitions
**********************************************************************/
/* SSP1 RX DMA selection, (0) = SSP1RX not connected/SPI2 connected,
   (1) = SSP1RX connected/SPI2 not connected */
#define CLKPWR_SSPCTRL_DMA_SSP1RX  _BIT(5)
/* SSP1 TX DMA selection, (0) = SSP1TX not connected/SPI1 connected,
   (1) = SSP1TX connected/SPI1 not connected */
#define CLKPWR_SSPCTRL_DMA_SSP1TX  _BIT(4)
/* SSP0 RX DMA selection, (0) = SSP1RX not connected/SPI2 connected,
   (1) = SSP1RX connected/SPI3 not connected */
#define CLKPWR_SSPCTRL_DMA_SSP0RX  _BIT(3)
/* SSP0 TX DMA selection, (0) = SSP1TX not connected/SPI1 connected,
   (1) = SSP1TX connected/SPI4 not connected */
#define CLKPWR_SSPCTRL_DMA_SSP0TX  _BIT(2)
/* SSP0 clock disable (0) / enable (1) bit */
#define CLKPWR_SSPCTRL_SSPCLK1_EN  _BIT(1)
/* SSP0 clock disable (0) / enable (1) bit */
#define CLKPWR_SSPCTRL_SSPCLK0_EN  _BIT(0)


/**********************************************************************
* clkpwr_timer_clk_ctrl register definitions
**********************************************************************/
/* High speed timer clock enable, (0) = disable, (1) = enable */
#define CLKPWR_PWMCLK_HSTIMER_EN   0x2
/* Watchdog timer clock enable, (0) = disable, (1) = enable */
#define CLKPWR_PWMCLK_WDOG_EN      0x1


/**********************************************************************
* clkpwr_macclk_ctrl register definitions
**********************************************************************/
/* Disables ethernet MAC pins */
#define CLKPWR_MACCTRL_NO_ENET_PIS 0x00
/* Ethernet MAC pins setup for MII */
#define CLKPWR_MACCTRL_USE_MII_PINS 0x08
/* Ethernet MAC pins setup for RMII */
#define CLKPWR_MACCTRL_USE_RMII_PINS 0x18
/* Mask for MAC pins selection */
#define CLKPWR_MACCTRL_PINS_MSK    0x18
/* Ethernet MAC DMA clock disable (0) / enable (1) bit */
#define CLKPWR_MACCTRL_DMACLK_EN   _BIT(2)
/* Ethernet MAC MMIO clock disable (0) / enable (1) bit */
#define CLKPWR_MACCTRL_MMIOCLK_EN  _BIT(1)
/* Ethernet MAC host registers clock disable (0) / enable (1) bit */
#define CLKPWR_MACCTRL_HRCCLK_EN   _BIT(0)


/**********************************************************************
* clkpwr_timers_pwms_clk_ctrl_1 register definitions
**********************************************************************/
/* Timer 3 clock enable, (0) = disable, (1) = enable */
#define CLKPWR_TMRPWMCLK_TIMER3_EN 0x20
/* Timer 2 clock enable, (0) = disable, (1) = enable */
#define CLKPWR_TMRPWMCLK_TIMER2_EN 0x10
/* Timer 1 clock enable, (0) = disable, (1) = enable */
#define CLKPWR_TMRPWMCLK_TIMER1_EN 0x08
/* Timer 0 clock enable, (0) = disable, (1) = enable */
#define CLKPWR_TMRPWMCLK_TIMER0_EN 0x04
/* PWM 4 clock enable, (0) = disable, (1) = enable */
#define CLKPWR_TMRPWMCLK_PWM4_EN   0x02
/* PWM 3 clock enable, (0) = disable, (1) = enable */
#define CLKPWR_TMRPWMCLK_PWM3_EN   0x01


/*
 * Macro pointing to Clock and Power control registers
 */
#define CLKPWR ((CLKPWR_REGS_T *)(CLK_PM_BASE))

/*
 * Main system clocks
 */
typedef enum
{
  /* Main oscillator clock */
  CLKPWR_MAINOSC_CLK,
  /* RTC clock */
  CLKPWR_RTC_CLK,
  /* System clock (Main oscillator or PLL397) */
  CLKPWR_SYSCLK,
  /* ARM clock, either HCLK(PLL), SYSCLK, or PERIPH_CLK */
  CLKPWR_ARM_CLK,
  /* HCLK (HCLKPLL divided, SYSCLK, or PERIPH_CLK) */
  CLKPWR_HCLK,
  /* Peripheral clock (HCLKPLL divided or SYSCLK) */
  CLKPWR_PERIPH_CLK,
  /* USB HCLK SYS*/
  CLKPWR_USB_HCLK_SYS,
  /* USB PLL clock */
  CLKPWR_48M_CLK,
  /* DDR clock (HCLKPLL divided or SYSCLK) */
  CLKPWR_DDR_CLK,
  /* Sd card controller */
  CLKPWR_MSSD_CLK,
  CLKPWR_BASE_INVALID
} CLKPWR_BASE_CLOCK_T;

/*
 * Clock rate fetch function
 */
unsigned int sys_get_rate(CLKPWR_BASE_CLOCK_T clkid);





/**********************************************************************
* Timer/counter register structures
**********************************************************************/

/* Timer module register structures */
typedef struct
{
  volatile unsigned int ir;          /* Timer interrupt status reg */
  volatile unsigned int tcr;         /* Timer control register */
  volatile unsigned int tc;          /* Timer counter value reg */
  volatile unsigned int pr;          /* Timer prescale register */
  volatile unsigned int pc;          /* Timer prescale counter reg */
  volatile unsigned int mcr;         /* Timer Match control reg */
  volatile unsigned int mr[4];       /* Timer Match registers */
  volatile unsigned int ccr;         /* Timer Capture control reg */
  volatile unsigned int cr[4];       /* Timer Capture registers */
  volatile unsigned int emr;         /* Timer External match reg */
  volatile unsigned int rsvd2[12];   /* Reserved */
  volatile unsigned int ctcr;        /* Timer Count control reg */
} TIMER_CNTR_REGS_T;

/**********************************************************************
* ir register definitions
* Write a '1' to clear interrupt, reading a '1' indicates active int
**********************************************************************/
/* Macro for getting a timer match interrupt bit */
#define TIMER_CNTR_MTCH_BIT(n)     (1 << ((n) & 0x3))

/* Macro for getting a capture event interrupt bit */
#define TIMER_CNTR_CAPT_BIT(n)     (1 << (4 + ((n) & 0x3)))

/**********************************************************************
* tcr register definitions
**********************************************************************/
/* Timer/counter enable bit */
#define TIMER_CNTR_TCR_EN          0x1

/* Timer/counter reset bit */
#define TIMER_CNTR_TCR_RESET       0x2

/**********************************************************************
* mcr register definitions
**********************************************************************/
/* Bit location for interrupt on MRx match, n = 0 to 3 */
#define TIMER_CNTR_MCR_MTCH(n)     (0x1 << ((n) * 3))

/* Bit location for reset on MRx match, n = 0 to 3 */
#define TIMER_CNTR_MCR_RESET(n)    (0x1 << (((n) * 3) + 1))

/* Bit location for stop on MRx match, n = 0 to 3 */
#define TIMER_CNTR_MCR_STOP(n)     (0x1 << (((n) * 3) + 2))

/**********************************************************************
* ccr register definitions
**********************************************************************/
/* Bit location for CAP.n on CRx rising edge, n = 0 to 3 */
#define TIMER_CNTR_CCR_CAPNRE(n)   (0x1 << ((n) * 3))

/* Bit location for CAP.n on CRx falling edge, n = 0 to 3 */
#define TIMER_CNTR_CCR_CAPNFE(n)   (0x1 << (((n) * 3) + 1))

/* Bit location for CAP.n on CRx interrupt enable, n = 0 to 3 */
#define TIMER_CNTR_CCR_CAPNI(n)    (0x1 << (((n) * 3) + 2))

/**********************************************************************
* emr register definitions
**********************************************************************/
/* Bit location for output state change of MAT.n when external match
   happens, n = 0 to 3 */
#define TIMER_CNTR_EMR_DRIVE(n)    (1 << (n))

/* Macro for setting MAT.n soutput state */
#define TIMER_CNTR_EMR_DRIVE_SET(n, s) (((s) & 0x1) << (n))

/* Output state change of MAT.n when external match happens */
#define TIMER_CNTR_EMR_NOTHING     0x0
#define TIMER_CNTR_EMR_LOW         0x1
#define TIMER_CNTR_EMR_HIGH        0x2
#define TIMER_CNTR_EMR_TOGGLE      0x3

/* Macro for setting for the MAT.n change state bits */
#define TIMER_CNTR_EMR_EMC_SET(n, s) (((s) & 0x3) << (4 + ((n) * 2)))

/* Mask for the MAT.n change state bits */
#define TIMER_CNTR_EMR_EMC_MASK(n) (0x3 << (4 + ((n) * 2)))

/**********************************************************************
* ctcr register definitions
**********************************************************************/
/* Mask to get the Counter/timer mode bits */
#define TIMER_CNTR_CTCR_MODE_MASK  0x3

/* Mask to get the count input select bits */
#define TIMER_CNTR_CTCR_INPUT_MASK 0xC

/* Counter/timer modes */
#define TIMER_CNTR_CTCR_TIMER_MODE 0x0
#define TIMER_CNTR_CTCR_TCINC_MODE 0x1
#define TIMER_CNTR_CTCR_TCDEC_MODE 0x2
#define TIMER_CNTR_CTCR_TCBOTH_MODE 0x3

/* Count input selections */
#define TIMER_CNTR_CTCR_INPUT_CAP0 0x0
#define TIMER_CNTR_CTCR_INPUT_CAP1 0x1
#define TIMER_CNTR_CTCR_INPUT_CAP2 0x2
#define TIMER_CNTR_CTCR_INPUT_CAP3 0x3

/* Macro for setting the counter/timer mode */
#define TIMER_CNTR_SET_MODE(n)     ((n) & 0x3)

/* Macro for setting the count input select */
#define TIMER_CNTR_SET_INPUT(n)    (((n) & 0x3) << 2)

/* Macros pointing to timer registers */
#define TIMER_CNTR0 ((TIMER_CNTR_REGS_T *)(TIMER0_BASE))
#define TIMER_CNTR1 ((TIMER_CNTR_REGS_T *)(TIMER1_BASE))
#define TIMER_CNTR2 ((TIMER_CNTR_REGS_T *)(TIMER2_BASE))
#define TIMER_CNTR3 ((TIMER_CNTR_REGS_T *)(TIMER3_BASE))





/* WDT module register structures */
typedef struct
{
  volatile unsigned int wdtim_int;      /* WDT interrupt status register */
  volatile unsigned int wdtim_ctrl;     /* WDT control register */
  volatile unsigned int wdtim_counter;  /* WDT counter value register */
  volatile unsigned int wdtim_mctrl;    /* WDT match control register */
  volatile unsigned int wdtim_match0;   /* WDT match 0 register */
  volatile unsigned int wdtim_emr;      /* WDT external match control reg */
  volatile unsigned int wdtim_pulse;    /* WDT reset pulse length register */
  volatile unsigned int wdtim_res;      /* WDT reset source register */
} WDT_REGS_T;

/**********************************************************************
* wdtim_int register definitions
**********************************************************************/
/* Interrupt flag for MATCH 0 interrupt */
#define WDT_MATCH_INT           _BIT(0)

/**********************************************************************
* wdtim_ctrl register definitions
**********************************************************************/
#define WDT_COUNT_ENAB          _BIT(0) /* Timer Counter enable */
#define WDT_RESET_COUNT         _BIT(1) /* Timer Counter reset */
#define WDT_PAUSE_EN            _BIT(2) /* Timer Cntr stopped in debug*/

/**********************************************************************
* wdtim_mctrl register definitions
**********************************************************************/
#define WDT_MR0_INT             _BIT(0) /* Enable WDT int on MR0 */
#define WDT_RESET_COUNT0        _BIT(1) /* Enable WDT reset on MR0 */
#define WDT_STOP_COUNT0         _BIT(2) /* Enable WDT stop on MR0 */
#define WDT_M_RES1              _BIT(3) /* M_RES1 control */
#define WDT_M_RES2              _BIT(4) /* M_RES2 control */
#define WDT_RESFRC1             _BIT(5) /* RESFRC1 control */
#define WDT_RESFRC2             _BIT(6) /* RESFRC2 control */

/* Macro pointing to WDT registers */
#define WDT ((WDT_REGS_T *)(WDTIM_BASE))



/* SLC NAND controller module register structures */
typedef struct
{
  volatile unsigned int slc_data;      /* SLC NAND data reg */
  volatile unsigned int slc_addr;      /* SLC NAND address register */
  volatile unsigned int slc_cmd;       /* SLC NAND command reg */
  volatile unsigned int slc_stop;      /* SLC NAND stop register */
  volatile unsigned int slc_ctrl;      /* SLC NAND control reg */
  volatile unsigned int slc_cfg;       /* SLC NAND config register */
  volatile unsigned int slc_stat;      /* SLC NAND status register */
  volatile unsigned int slc_int_stat;  /* SLC NAND int status register */
  volatile unsigned int slc_ien;       /* SLC NAND int enable register */
  volatile unsigned int slc_isr;       /* SLC NAND int set register */
  volatile unsigned int slc_icr;       /* SLC NAND int clear register */
  volatile unsigned int slc_tac;       /* SLC NAND timing register */
  volatile unsigned int slc_tc;        /* SLC NAND transfer count reg */
  volatile unsigned int slc_ecc;       /* SLC NAND parity register */
  volatile unsigned int slc_dma_data;  /* SLC NAND DMA data register */
} SLCNAND_REGS_T;

/**********************************************************************
* slc_ctrl register definitions
**********************************************************************/
#define SLCCTRL_SW_RESET    _BIT(2) /* Reset the NAND controller bit */
#define SLCCTRL_ECC_CLEAR   _BIT(1) /* Reset ECC bit */
#define SLCCTRL_DMA_START   _BIT(0) /* Start DMA channel bit */

/**********************************************************************
* slc_cfg register definitions
**********************************************************************/
#define SLCCFG_CE_LOW       _BIT(5) /* Force CE low bit */
#define SLCCFG_DMA_ECC      _BIT(4) /* Enable DMA ECC bit */
#define SLCCFG_ECC_EN       _BIT(3) /* ECC enable bit */
#define SLCCFG_DMA_BURST    _BIT(2) /* DMA burst bit */
#define SLCCFG_DMA_DIR      _BIT(1) /* DMA write(0)/read(1) bit */
#define SLCCFG_WIDTH        _BIT(0) /* External device width, 0=8bit */

/**********************************************************************
* slc_stat register definitions
**********************************************************************/
#define SLCSTAT_DMA_FIFO    _BIT(2) /* DMA FIFO has data bit */
#define SLCSTAT_SLC_FIFO    _BIT(1) /* SLC FIFO has data bit */
#define SLCSTAT_NAND_READY  _BIT(0) /* NAND device is ready bit */

/**********************************************************************
* slc_int_stat, slc_ien, slc_isr, and slc_icr register definitions
**********************************************************************/
#define SLCSTAT_INT_TC      _BIT(1) /* Transfer count bit */
#define SLCSTAT_INT_RDY_EN  _BIT(0) /* Ready interrupt bit */

/**********************************************************************
* slc_tac register definitions
**********************************************************************/
/* Clock setting for RDY write sample wait time in 2*n clocks */
#define SLCTAC_WDR(n)       (((n) & 0xF) << 28)
/* Write pulse width in clocks cycles, 1 to 16 clocks */
#define SLCTAC_WWIDTH(n)    (((n) & 0xF) << 24)
/* Write hold time of control and data signals, 1 to 16 clocks */
#define SLCTAC_WHOLD(n)     (((n) & 0xF) << 20)
/* Write setup time of control and data signals, 1 to 16 clocks */
#define SLCTAC_WSETUP(n)    (((n) & 0xF) << 16)
/* Clock setting for RDY read sample wait time in 2*n clocks */
#define SLCTAC_RDR(n)       (((n) & 0xF) << 12)
/* Read pulse width in clocks cycles, 1 to 16 clocks */
#define SLCTAC_RWIDTH(n)    (((n) & 0xF) << 8)
/* Read hold time of control and data signals, 1 to 16 clocks */
#define SLCTAC_RHOLD(n)     (((n) & 0xF) << 4)
/* Read setup time of control and data signals, 1 to 16 clocks */
#define SLCTAC_RSETUP(n)    (((n) & 0xF) << 0)

/* Macro pointing to SLC NAND controller registers */
#define SLCNAND ((SLCNAND_REGS_T *)(SLC_BASE))











/**********************************************************************
* Ethernet MAC controller register structures
**********************************************************************/

/* Ethernet MAC controller module register structures */
typedef struct
{
  /* MAC registers */
  volatile unsigned long mac1;
  volatile unsigned long mac2;
  volatile unsigned long ipgt;
  volatile unsigned long ipgr;
  volatile unsigned long clrt;
  volatile unsigned long maxf;
  volatile unsigned long supp;
  volatile unsigned long test;
  volatile unsigned long mcfg;
  volatile unsigned long mcmd;
  volatile unsigned long madr;
  volatile unsigned long mwtd;
  volatile unsigned long mrdd;
  volatile unsigned long mind;
  volatile unsigned long reserved1 [2];
  volatile unsigned long sa [3];
  volatile unsigned long reserved2 [45];
  /* Control registers */
  volatile unsigned long command;
  volatile unsigned long status;
  volatile unsigned long rxdescriptor;
  volatile unsigned long rxstatus;
  volatile unsigned long rxdescriptornumber;
  volatile unsigned long rxproduceindex;
  volatile unsigned long rxconsumeindex;
  volatile unsigned long txdescriptor;
  volatile unsigned long txstatus;
  volatile unsigned long txdescriptornumber;
  volatile unsigned long txproduceindex;
  volatile unsigned long txconsumeindex;
  volatile unsigned long reserved3 [10];
  volatile unsigned long tsv0;
  volatile unsigned long tsv1;
  volatile unsigned long rsv;
  volatile unsigned long reserved4 [3];
  volatile unsigned long flowcontrolcounter;
  volatile unsigned long flowcontrolstatus;
  volatile unsigned long reserved5 [34];
  /* RX filter registers */
  volatile unsigned long rxfliterctrl;
  volatile unsigned long rxfilterwolstatus;
  volatile unsigned long rxfilterwolclear;
  volatile unsigned long reserved6;
  volatile unsigned long hashfilterL;
  volatile unsigned long hashfilterh;
  volatile unsigned long reserved7 [882];
  /* Module control registers */
  volatile unsigned long intstatus;
  volatile unsigned long intenable;
  volatile unsigned long intclear;
  volatile unsigned long Intset;
  volatile unsigned long reserved8;
  volatile unsigned long powerdown;
  volatile unsigned long reserved9;
} ETHERNET_REGS_T;

/* Structure of a TX/RX descriptor */
typedef struct
{
  volatile unsigned long packet;
  volatile unsigned long control;
} TXRX_DESC_T;

/* Structure of a RX status entry */
typedef struct
{
  volatile unsigned long statusinfo;
  volatile unsigned long statushashcrc;
} RX_STATUS_T;

/**********************************************************************
* mac1 register definitions
**********************************************************************/
/* Set this to allow receive frames to be received. Internally the
   MAC synchronize this control bit to the incoming receive stream */
#define MAC1_RECV_ENABLE               _BIT(0)
/* When enabled (set to 1), the MAC will pass all frames regardless
   of type (normal vs. Control). When disabled, the MAC does not pass
   valid Control frames */
#define MAC1_PASS_ALL_RX_FRAMES        _BIT(1)
/* When enabled (set to 1), the MAC acts upon received PAUSE Flow
   Control frames. When disabled, received PAUSE Flow Control frames
   are ignored */
#define MAC1_RX_FLOW_CONTROL           _BIT(2)
/* When enabled (set to 1), PAUSE Flow Control frames are allowed
   to be transmitted. When disabled, Flow Control frames are blocked */
#define MAC1_TX_FLOW_CONTROL           _BIT(3)
/* Setting this bit will cause the MAC Transmit interface to be
   looped back to the MAC Receive interface. Clearing this bit
   results in normal operation */
#define MAC1_LOOPBACK                  _BIT(4)
/* Setting this bit will put the Transmit Function logic in reset */
#define MAC1_RESET_TX                  _BIT(8)
/* Setting this bit resets the MAC Control Sublayer / Transmit logic.
   The MCS logic implements flow control */
#define MAC1_RESET_MCS_TX              _BIT(9)
/* Setting this bit will put the Ethernet receive logic in reset */
#define MAC1_RESET_RX                  _BIT(10)
/* Setting this bit resets the MAC Control Sublayer / Receive logic.
   The MCS logic implements flow control */
#define MAC1_RESET_MCS_RX              _BIT(11)
/* Setting this bit will cause a reset to the random number generator
   within the Transmit Function */
#define MAC1_SIMULATION_RESET          _BIT(14)
/* Setting this bit will put all modules within the MAC in reset
   except the Host Interface */
#define MAC1_SOFT_RESET                _BIT(15)

/**********************************************************************
* mac2 register definitions
**********************************************************************/
/* When enabled (set to 1), the MAC operates in Full-Duplex mode.
   When disabled the MAC operates in Half-Duplex mode */
#define MAC2_FULL_DUPLEX               _BIT(0)
/* When enabled (set to 1), both transmit and receive frame lengths
   are compared to the Length/Type field. If the Length/Type field
   represents a length then the check is performed. Mismatches are
   reported in the StatusInfo word for each received frame */
#define MAC2_FRAME_LENGTH_CHECKING     _BIT(1)
/* When enabled (set to 1), frames of any length are transmitted
   and received */
#define MAC2_HUGH_LENGTH_CHECKING      _BIT(2)
/* This bit determines the number of bytes, if any, of proprietary
   header information that exist on the front of IEEE 802.3 frames.
   When 1, four bytes of header (ignored by the CRC function) are
   added. When 0, there is no proprietary header */
#define MAC2_DELAYED_CRC               _BIT(3)
/* Set this bit to append a CRC to every frame whether padding was
   required or not. Must be set if PAD/CRC ENABLE is set. Clear this
   bit if frames presented to the MAC contain a CRC */
#define MAC2_CRC_ENABLE                _BIT(4)
/* Set this bit to have the MAC pad all short frames. Clear this bit
   if frames presented to the MAC have a valid length. This bit is used
   in conjunction with AUTO PAD ENABLE and VLAN PAD ENABLE */
#define MAC2_PAD_CRC_ENABLE            _BIT(5)
/* Set this bit to cause the MAC to pad all short frames to 64 bytes
   and append a valid CRC. Note: This bit is ignored if
   MAC2_PAD_CRC_ENABLE is cleared */
#define MAC2_VLAN_PAD_ENABLE           _BIT(6)
/* Set this bit to cause the MAC to automatically detect the type of
   frame, either tagged or un-tagged, by comparing the two octets
   following the source address with 0x8100 (VLAN Protocol ID) and
   pad accordingly. Table 14273 - Pad Operation provides a description
   of the pad function based on the configuration of this register.
   Note: This bit is ignored if PAD / CRC ENABLE is cleared */
#define MAC2_AUTO_DETECT_PAD_ENABLE    _BIT(7)
/* When enabled (set to 1), the MAC will verify the content of the
   preamble to ensure it contains 0x55 and is error-free. A packet
   with an incorrect preamble is discarded. When disabled, no preamble
   checking is performed */
#define MAC2_PURE_PREAMBLE_ENFORCEMENT _BIT(8)
/* When enabled (set to 1), the MAC only allows receive packets
   which contain preamble fields less than 12 bytes in length. When
   disabled, the MAC allows any length preamble as per the Standard */
#define MAC2_LONG_PREAMBLE_ENFORCEMENT _BIT(9)
/* When enabled (set to 1), the MAC will immediately retransmit
   following a collision rather than using the Binary Exponential
   Backoff algorithm as specified in the Standard */
#define MAC2_NO_BACKOFF                _BIT(12)
/* When enabled (set to 1), after the MAC incidentally causes a
   collision during back pressure, it will immediately retransmit
   without backoff, reducing the chance of further collisions and
   ensuring transmit packets get sent */
#define MAC2_BACK_PRESSURE             _BIT(13)
/* When enabled (set to 1) the MAC will defer to carrier indefinitely
   as per the Standard. When disabled, the MAC will abort when the
   excessive deferral limit is reached */
#define MAC2_EXCESS_DEFER              _BIT(14)

/**********************************************************************
* ipgt register definitions
**********************************************************************/
/* This is a programmable field representing the nibble time offset
   of the minimum possible period between the end of any transmitted
   packet to the beginning of the next. In Full-Duplex mode, the
   register value should be the desired period in nibble times minus 3.
   In Half-Duplex mode, the register value should be the desired
   period in nibble times minus 6. In Full-Duplex the recommended
   setting is 0x15 (21d), which represents the minimum IPG of 960 ns
   (in 100 Mbps mode) or 9.6 ?s (in 10 Mbps mode). In Half-Duplex the
   recommended setting is 0x12 (18d), which also represents the minimum
   IPG of 960 ns (in 100 Mbps mode) or 9.6 ?s (in 10 Mbps mode) */
#define IPGT_LOAD(n)                   ((n) & 0x7F)

/**********************************************************************
* ipgr register definitions
**********************************************************************/
/* This is a programmable field representing the Non-Back-to-Back
   Inter-Packet-Gap. The recommended value is 0x12 (18d), which
   represents the minimum IPG of 960 ns (in 100 Mbps mode) or 9.6 ?s
   (in 10 Mbps mode) */
#define IPGR_LOAD_PART2(n)             ((n) & 0x7F)
/* This is a programmable field representing the optional carrierSense
   window referenced in IEEE 802.3/4.2.3.2.1 'Carrier Deference'. If
   carrier is detected during the timing of IPGR1, the MAC defers to
   carrier. If, however, carrier becomes active after IPGR1, the MAC
   continues timing IPGR2 and transmits, knowingly causing a collision,
   thus ensuring fair access to medium. Its range of values is 0x0 to
   IPGR2. The recommended value is 0xC (12d) */
#define IPGR_LOAD_PART1(n)             (((n) & 0x7F) << 8)

/**********************************************************************
* clrt register definitions
**********************************************************************/
/* This is a programmable field specifying the number of
   retransmission attempts following a collision before aborting the
   packet due to excessive collisions. The Standard specifies the
   attemptLimit to be 0xF (15d). See IEEE 802.3/4.2.3.2.5. */
#define CLRT_LOAD_RETRY_MAX(n)         ((n) & 0xF)
/* This is a programmable field representing the slot time or
   collision window during which collisions occur in properly
   configured networks. The default value of 0x37 (55d) represents a
   56 byte window following the preamble and SFD. */
#define CLRT_LOAD_COLLISION_WINDOW(n)  (((n) & 0x3F) << 8)

/**********************************************************************
* maxf register definitions
**********************************************************************/
/* This field resets to the value 0x0600, which represents a maximum
   receive frame of 1536 octets. An untagged maximum size Ethernet
   frame is 1518 octets. A tagged frame adds four octets for a total
   of 1522 octets. If a shorter maximum length restriction is desired,
   program this 16 bit field. */
#define MAXF_LOAD_MAX_FRAME_LEN(n)     ((n) & 0xFFFF)

/**********************************************************************
* supp register definitions
**********************************************************************/
/* This bit configures the Reduced MII logic for the current operating
   speed. When set, 100 Mbps mode is selected. When cleared, 10 Mbps
   mode is selected */
#define SUPP_SPEED                     _BIT(8)
/* Reset Reduced MII Logic */
#define SUPP_RESET_RMII                _BIT(11)

/**********************************************************************
* test register definitions
**********************************************************************/
/* This bit reduces the effective PAUSE quanta from 64 byte-times to
   1 byte-time. */
#define TEST_SHORTCUT_PAUSE_QUANTA     _BIT(0)
/* This bit causes the MAC Control sublayer to inhibit transmissions,
   just as if a PAUSE Receive Control frame with a nonzero pause time
   parameter was received. */
#define TEST_PAUSE                     _BIT(1)
/* Setting this bit will cause the MAC to assert backpressure on the
   link. Backpressure causes preamble to be transmitted, raising
   carrier sense. A transmit packet from the system will be sent
   during backpressure. */
#define TEST_BACKPRESSURE              _BIT(2)

/**********************************************************************
* mcfg register definitions
**********************************************************************/
/* Set this bit to cause the MII Management hardware to perform read
   cycles across a range of PHYs. When set, the MII Management
   hardware will perform read cycles from address 1 through the value
   set in PHY ADDRESS[4:0]. Clear this bit to allow continuous reads
   of the same PHY. */
#define MCFG_SCAN_INCREMENT            _BIT(0)
/* Set this bit to cause the MII Management hardware to perform
   read/write cycles without the 32 bit preamble field. Clear this bit
   to cause normal cycles to be performed. Some PHYs support
   suppressed preamble. */
#define MCFG_SUPPRESS_PREAMBLE         _BIT(1)
/* This field is used by the clock divide logic in creating the MII
   Management Clock (MDC) which IEEE 802.3u defines to be no faster
   than 2.5 MHz. Some PHYs support clock rates up to 12.5 MHz,
   however. Refer to Table 14280 below for the definition of values
   for this field. */
#define MCFG_CLOCK_SELECT(n)           (((n) & 0x7) << 2)
/* MCFG_CLOCK_SELECT macro load values */
#define MCFG_CLOCK_HOST_DIV_4          0
#define MCFG_CLOCK_HOST_DIV_6          2
#define MCFG_CLOCK_HOST_DIV_8          3
#define MCFG_CLOCK_HOST_DIV_10         4
#define MCFG_CLOCK_HOST_DIV_14         5
#define MCFG_CLOCK_HOST_DIV_20         6
#define MCFG_CLOCK_HOST_DIV_28         7
/* This bit resets the MII Management hardware */
#define MCFG_RESET_MII_MGMT            _BIT(15)

/**********************************************************************
* mcmd register definitions
**********************************************************************/
/* This bit causes the MII Management hardware to perform a single
   Read cycle. The Read data is returned in Register MRDD (MII Mgmt
   Read Data). */
#define MCMD_READ                      _BIT(0)
/* This bit causes the MII Management hardware to perform Read cycles
   continuously. This is useful for monitoring Link Fail for example */
#define MCMD_SCAN                      _BIT(1)

/**********************************************************************
* madr register definitions
**********************************************************************/
/* This field represents the 5 bit Register Address field of Mgmt
   cycles. Up to 32 registers can be accessed. */
#define MADR_REGISTER_ADDRESS(n)       ((n) & 0x1F)
/* This field represents the 5 bit PHY Address field of Mgmt
   cycles. Up to 31 PHYs can be addressed (0 is reserved). */
#define MADR_PHY_0ADDRESS(n)           (((n) & 0x1F) << 8)

/**********************************************************************
* mwtd register definitions
**********************************************************************/
/* When written, an MII Mgmt write cycle is performed using the 16 bit
   data and the pre-configured PHY and Register addresses from the
   MII Mgmt Address register (MADR). */
#define MWDT_WRITE(n)                  ((n) & 0xFFFF)

/**********************************************************************
* mrdd register definitions
**********************************************************************/
/* Read mask for MUU read */
#define MRDD_READ_MASK                 0xFFFF

/**********************************************************************
* mind register definitions
**********************************************************************/
/* When 1 is returned - indicates MII Mgmt is currently performing
   an MII Mgmt Read or Write cycle. */
#define MIND_BUSY                      _BIT(0)
/* When 1 is returned - indicates a scan operation (continuous MII
   Mgmt Read cycles) is in progress. */
#define MIND_SCANNING                  _BIT(1)
/* When 1 is returned - indicates MII Mgmt Read cycle has not
   completed and the Read Data is not yet valid. */
#define MIND_NOT_VALID                 _BIT(2)
/* When 1 is returned - indicates that an MII Mgmt link fail has
   occurred.*/
#define MIND_MII_LINK_FAIL             _BIT(3)

/**********************************************************************
* command register definitions
**********************************************************************/
/* Enable receive */
#define COMMAND_RXENABLE               _BIT(0)
/* Enable transmit */
#define COMMAND_TXENABLE               _BIT(1)
/* When a 1 is written, all datapaths and the host registers are
   reset. The MAC needs to be reset separately. */
#define COMMAND_REG_RESET              _BIT(3)
/* When a 1 is written, the transmit datapath is reset. */
#define COMMAND_TXRESET                _BIT(4)
/* When a 1 is written, the receive datapath is reset. */
#define COMMAND_RXRESET                _BIT(5)
/* When set to 1, passes runt frames smaller than 64 bytes to
   memory unless they have a CRC error. If 0 runt frames are
   filtered out. */
#define COMMAND_PASSRUNTFRAME          _BIT(6)
/* When set to 1, disables receive filtering i.e. all frames
   received are written to memory. */
#define COMMAND_PASSRXFILTER           _BIT(7)
/* Enable IEEE 802.3 / clause 31 flow control sending pause
   frames in full duplex and continuous preamble in half duplex. */
#define COMMAND_TXFLOWCONTROL          _BIT(8)
/* When set to 1, RMII mode is selected; if 0, MII mode is
   selected. */
#define COMMAND_RMII                   _BIT(9)
/* When set to 1, indicates full duplex operation. */
#define COMMAND_FULLDUPLEX             _BIT(10)

/**********************************************************************
* status register definitions
**********************************************************************/
/* If 1, the receive channel is active. If 0, the receive channel is
   inactive. */
#define STATUS_RXACTIVE                _BIT(0)
/* If 1, the transmit channel is active. If 0, the transmit channel is
   inactive. */
#define STATUS_TXACTIVE                _BIT(1)

/**********************************************************************
* tsv0 register definitions
**********************************************************************/
/* The attached CRC in the packet did not match the internally
   generated CRC. */
#define TSV0_CRC_ERROR                 _BIT(0)
/* Indicates the frame length field does not match the actual
   number of data items and is not a type field. */
#define TSV0_LENGTH_CHECK_ERROR        _BIT(1)
/* Indicates that frame type/length field was larger tha 1500 bytes. */
#define TSV0_LENGTH_OUT_OF_RANGE       _BIT(2)
/* Transmission of packet was completed. */
#define TSV0_DONE                      _BIT(3)
/* Packets destination was a multicast address. */
#define TSV0_MULTICAST                 _BIT(4)
/* Packets destination was a broadcast address. */
#define TSV0_BROADCAST                 _BIT(5)
/* Packet was deferred for at least one attempt, but less than
   an excessive defer. */
#define TSV0_PACKET_DEFER              _BIT(6)
/* Packet was deferred in excess of 6071 nibble times in
   100 Mbps or 24287 bit times in 10 Mbps mode. */
#define TSV0_ESCESSIVE_DEFER           _BIT(7)
/* Packet was aborted due to exceeding of maximum allowed
   number of collisions. */
#define TSV0_ESCESSIVE_COLLISION       _BIT(8)
/* Collision occurred beyond collision window, 512 bit times. */
#define TSV0_LATE_COLLISION            _BIT(9)
/* Byte count in frame was greater than can be represented
   in the transmit byte count field in TSV1. */
#define TSV0_GIANT                     _BIT(10)
/* Host side caused buffer underrun. */
#define TSV0_UNDERRUN                  _BIT(11)
/* Macro: The total number of bytes transferred including
   collided attempts. */
#define TSV0_TOTAL_BYTES(n)            (((n) >> 12) & 0xFFFF)
/* The frame was a control frame. */
#define TSV0_CONTROL_FRAME             _BIT(28)
/* The frame was a control frame with a valid PAUSE opcode. */
#define TSV0_PAUSE                     _BIT(29)
/* Carrier-sense method backpressure was previously applied. */
#define TSV0_BACKPRESSURE              _BIT(30)
/* Frames length/type field contained 0x8100 which is the
   VLAN protocol identifier. */
#define TSV0_VLAN                      _BIT(31)

/**********************************************************************
* tsv1 register definitions
**********************************************************************/
/* Macro: The total number of bytes in the frame, not counting the
   collided bytes. */
#define TSV1_TRANSMIT_BYTE_COUNT(n)    ((n) & 0xFFFF)
/* Macro: Number of collisions the current packet incurred during
   transmission attempts. The maximum number of collisions
   (16) cannot be represented. */
#define TSV1_COLLISION_COUNT(n)        (((n) >> 16) & 0xF)

/**********************************************************************
* rsv register definitions
**********************************************************************/
/* Macro: Indicates length of received frame. */
#define RSV_RECEIVED_BYTE_COUNT(n)     ((n) & 0xFFFF)
/* Indicates that a packet was dropped. */
#define RSV_RXDV_EVENT_IGNORED         _BIT(16)
/* Indicates that the last receive event seen was not long
   enough to be a valid packet. */
#define RSV_RXDV_EVENT_PREVIOUSLY_SEEN _BIT(17)
/* Indicates that at some time since the last receive statistics,
   a carrier event was detected. */
#define RSV_CARRIER_EVNT_PREVIOUS_SEEN _BIT(18)
/* Indicates that MII data does not represent a valid receive
   code. */
#define RSV_RECEIVE_CODE_VIOLATION     _BIT(19)
/* The attached CRC in the packet did not match the internally
   generated CRC. */
#define RSV_CRC_ERROR                  _BIT(20)
/* Indicates the frame length field does not match the actual
   number of data items and is not a type field. */
#define RSV_LENGTH_CHECK_ERROR         _BIT(21)
/* Indicates that frame type/length field was larger than 1518 bytes */
#define RSV_LENGTH_OUT_OF_RANGE        _BIT(22)
/* The packet had valid CRC and no symbol errors. */
#define RSV_RECEIVE_OK                 _BIT(23)
/* The packet destination was a multicast address. */
#define RSV_MULTICAST                  _BIT(24)
/* The packet destination was a boardcase address. */
#define RSV_BROADCAST                  _BIT(25)
/* Indicates that after the end of packet another 1-7 bits were
   received. A single nibble, called dribble nibble, is formed
   but not sent out. */
#define RSV_DRIBBLE_NIBBLE             _BIT(26)
/* The frame was a control frame. */
#define RSV_CONTROL_FRAME              _BIT(27)
/* The frame was a control frame with a valid PAUSE opcode. */
#define RSV_PAUSE                      _BIT(28)
/* The current frame was recognized as a Control Frame but
   contains an unknown opcode. */
#define RSV_UNSUPPORTED_OPCODE         _BIT(29)
/* Frames length/type field contained 0x8100 which is the
   VLAN protocol identifier. */
#define RSV_VLAN                       _BIT(30)

/**********************************************************************
* flowcontrolcounter register definitions
**********************************************************************/
/* Macro: In full duplex mode the MirrorCounter specifies the number
   of cycles before re-issuing the Pause control frame. */
#define FCCR_MIRRORCOUNTER(n)          ((n) & 0xFFFF)
/* Macro: In full-duplex mode the PauseTimer specifies the value
   that is inserted into the pause timer field of a pause flow
   control frame. In half duplex mode the PauseTimer
   specifies the number of backpressure cycles. */
#define FCCR_PAUSETIMER(n)             (((n) >> 16) & 0xFFFF)

/**********************************************************************
* flowcontrolstatus register definitions
**********************************************************************/
/* Macro: In full duplex mode this register represents the current
   value of the datapaths mirror counter which counts up to
   the value specified by the MirrorCounter field in the
   FlowControlCounter register. In half duplex mode the
   register counts until it reaches the value of the PauseTimer
   bits in the FlowControlCounter register. */
#define FCCR_MIRRORCOUNTERCURRENT(n)   ((n) & 0xFFFF)

/**********************************************************************
* rxfliterctrl, rxfilterwolstatus, and rxfilterwolclear shared
* register definitions
**********************************************************************/
/* Unicast frame control */
#define RXFLTRW_ACCEPTUNICAST          _BIT(0)
/* Broadcase frame control. */
#define RXFLTRW_ACCEPTUBROADCAST       _BIT(1)
/* Multicast frame control */
#define RXFLTRW_ACCEPTUMULTICAST       _BIT(2)
/* Imperfect unicast frame control */
#define RXFLTRW_ACCEPTUNICASTHASH      _BIT(3)
/* Imperfect multicast frame control */
#define RXFLTRW_ACCEPTUMULTICASTHASH   _BIT(4)
/* Perfect frame control */
#define RXFLTRW_ACCEPTPERFECT          _BIT(5)

/**********************************************************************
* rxfliterctrl register definitions
**********************************************************************/
/* When set to 1, the result of the magic packet filter will
   generate a WoL interrupt when there is a match. */
#define RXFLTRWSTS_MAGICPACKETENWOL    _BIT(12)
/* When set to 1, the result of the perfect address
   matching filter and the imperfect hash filter will
   generate a WoL interrupt when there is a match. */
#define RXFLTRWSTS_RXFILTERENWOL       _BIT(13)

/**********************************************************************
* rxfilterwolstatus register definitions
**********************************************************************/
/* When the value is 1, the receive filter caused WoL. */
#define RXFLTRWSTS_RXFILTERWOL         _BIT(7)
/* When the value is 1, the magic packet filter caused WoL. */
#define RXFLTRWSTS_MAGICPACKETWOL      _BIT(8)

/**********************************************************************
* rxfilterwolclear register definitions
**********************************************************************/
/* When a 1 is written to one of these bits (7 and/or 8),
   the corresponding status bit in the rxfilterwolstatus
   register is cleared. */
#define RXFLTRWCLR_RXFILTERWOL         RXFLTRWSTS_RXFILTERWOL
#define RXFLTRWCLR_MAGICPACKETWOL      RXFLTRWSTS_MAGICPACKETWOL

/**********************************************************************
* intstatus, intenable, intclear, and Intset shared register
* definitions
**********************************************************************/
/* Interrupt trigger on receive buffer overrun or descriptor underrun
   situations. */
#define MACINT_RXOVERRUNINTEN          _BIT(0)
/* Enable for interrupt trigger on receive errors. */
#define MACINT_RXERRORONINT            _BIT(1)
/* Enable for interrupt triggered when all receive descriptors have
   been processed i.e. on the transition to the situation where
   ProduceIndex == ConsumeIndex. */
#define MACINT_RXFINISHEDINTEN         _BIT(2)
/* Enable for interrupt triggered when a receive descriptor has
   been processed while the Interrupt bit in the Control field of the
   descriptor was set. */
#define MACINT_RXDONEINTEN             _BIT(3)
/* Enable for interrupt trigger on transmit buffer or descriptor
   underrun situations. */
#define MACINT_TXUNDERRUNINTEN         _BIT(4)
/* Enable for interrupt trigger on transmit errors. */
#define MACINT_TXERRORINTEN            _BIT(5)
/* Enable for interrupt triggered when all transmit descriptors
   have been processed i.e. on the transition to the situation
   where ProduceIndex == ConsumeIndex. */
#define MACINT_TXFINISHEDINTEN         _BIT(6)
/* Enable for interrupt triggered when a descriptor has been
   transmitted while the Interrupt bit in the Control field of the
   descriptor was set. */
#define MACINT_TXDONEINTEN             _BIT(7)
/* Enable for interrupt triggered by the SoftInt bit in the IntStatus
   register, caused by software writing a 1 to the SoftIntSet bit in
   the IntSet register. */
#define MACINT_SOFTINTEN               _BIT(12)
/* Enable for interrupt triggered by a Wakeup event detected by
   the receive filter. */
#define MACINT_WAKEUPINTEN             _BIT(13)

/**********************************************************************
* powerdown register definitions
**********************************************************************/
/* If true, all AHB accesses will return a read/write error,
   except accesses to the PowerDown register. */
#define POWERDOWN_MACAHB               _BIT(31)

/* Macro pointing to ethernet MAC controller registers */
#define ENETMAC ((ETHERNET_REGS_T *)(ETHERNET_BASE))










/***********************************************************************
 * SSP Module Register Structure
 **********************************************************************/

/* SSP Module Register Structure */
typedef struct
{
  volatile unsigned long cr0;     /* SSP control register 0 */
  volatile unsigned long cr1;     /* SSP control register 1 */
  volatile unsigned long data;    /* SSP data register */
  volatile unsigned long sr;      /* SSP status register */
  volatile unsigned long cpsr;    /* SSP clock prescale register */
  volatile unsigned long imsc;    /* SSP interrupt mask register */
  volatile unsigned long ris;     /* SSP raw interrupt status register */
  volatile unsigned long mis;     /* SSP masked interrupt status register */
  volatile unsigned long icr;     /* SSP interrupt clear register */
  volatile unsigned long dmacr;   /* SSP DMA enable register */
} SSP_REGS_T;

/***********************************************************************
 * cr0 register definitions
 **********************************************************************/
/* SSP data size load macro, must be 4 bits to 16 bits */
#define SSP_CR0_DSS(n)   _SBF(0, (((n) - 1) & 0xF)) // Data Size Select
/* SSP control 0 Motorola SPI mode */
#define SSP_CR0_FRF_SPI  0x00000000
/* SSP control 0 TI synchronous serial mode */
#define SSP_CR0_FRF_TI   0x00000010
/* SSP control 0 National Microwire mode */
#define SSP_CR0_FRF_NS   0x00000020
/* SSP control 0 protocol mask */
#define SSP_CR0_PRT_MSK  0x00000030
/* SPI clock polarity bit (used in SPI mode only), (1) = maintains the
   bus clock high between frames, (0) = low */
#define SSP_CR0_CPOL(n)  _SBF(6, ((n) & 0x01))
/* SPI clock out phase bit (used in SPI mode only), (1) = captures data
   on the second clock transition of the frame, (0) = first */
#define SSP_CR0_CPHA(n)  _SBF(7, ((n) & 0x01))
/* SSP serial clock rate value load macro, divider rate is
   PERIPH_CLK / (cpsr * (SCR + 1)) */
#define SSP_CR0_SCR(n)   _SBF(8, ((n) & 0xFF))

/***********************************************************************
 * cr1 register definitions
 **********************************************************************/
/* SSP control 1 loopback mode enable bit */
#define SSP_CR1_LBM         _BIT(0)
/* SSP control 1 enable bit */
#define SSP_CR1_SSE(n)      _SBF(1, ((n) & 0x01))
#define SSP_CR1_SSP_ENABLE  _BIT(1)
#define SSP_CR1_SSP_DISABLE 0
/* SSP control 1 master/slave bit, (1) = master, (0) = slave */
#define SSP_CR1_MS       _BIT(2)
#define SSP_CR1_MASTER   0
#define SSP_CR1_SLAVE    _BIT(2)
/* SSP control 1 slave out disable bit, disables transmit line in slave
   mode */
#define SSP_CR1_SOD      _BIT(3)

/***********************************************************************
 * data register definitions
 **********************************************************************/
/* SSP data load macro */
#define SSP_DATAMASK(n)   ((n) & 0xFFFF)

/***********************************************************************
 * SSP status register (sr) definitions
 **********************************************************************/
/* SSP status TX FIFO Empty bit */
#define SSP_SR_TFE      _BIT(0)
/* SSP status TX FIFO not full bit */
#define SSP_SR_TNF      _BIT(1)
/* SSP status RX FIFO not empty bit */
#define SSP_SR_RNE      _BIT(2)
/* SSP status RX FIFO full bit */
#define SSP_SR_RFF      _BIT(3)
/* SSP status SSP Busy bit */
#define SSP_SR_BSY      _BIT(4)

/***********************************************************************
 * SSP clock prescaler register (cpsr) definitions
 **********************************************************************/
/* SSP clock prescaler load macro */
#define SSP_CPSR_CPDVSR(n) _SBF(0, (n) & 0xFE)

/***********************************************************************
 * SSP interrupt registers (imsc, ris, mis, icr) definitions
 **********************************************************************/
/* SSP interrupt bit for RX FIFO overflow */
#define SSP_IMSC_RORIM   _BIT(0)
#define SSP_RIS_RORRIS   _BIT(0)
#define SSP_MIS_RORMIS   _BIT(0)
#define SSP_ICR_RORIC    _BIT(0)
/* SSP interrupt bit for RX FIFO not empty and has a data timeout */
#define SSP_IMSC_RTIM    _BIT(1)
#define SSP_RIS_RTRIS    _BIT(1)
#define SSP_MIS_RTMIS    _BIT(1)
#define SSP_ICR_RTIC     _BIT(1)
/* SSP interrupt bit for RX FIFO half full */
#define SSP_IMSC_RXIM    _BIT(2)
#define SSP_RIS_RXRIS    _BIT(2)
#define SSP_MIS_RXMIS    _BIT(2)
/* SSP interrupt bit for TX FIFO half empty */
#define SSP_IMSC_TXIM    _BIT(3)
#define SSP_RIS_TXRIS    _BIT(3)
#define SSP_MIS_TXMIS    _BIT(3)

/***********************************************************************
 * SSP DMA enable register (dmacr) definitions
 **********************************************************************/
/* SSP bit for enabling RX DMA */
#define SSP_DMA_RXDMAEN  _BIT(0)
/* SSP bit for enabling TX DMA */
#define SSP_DMA_TXDMAEN  _BIT(1)

/* Macros pointing to SSP registers */
#define SSP0  ((SSP_REGS_T *)(SSP0_BASE))



/***********************************************************************
* GPIO Module Register Structure
**********************************************************************/

/* GPIO Module Register Structure */
typedef struct
{
  volatile unsigned long p3_inp_state;   /* Input pin state register */
  volatile unsigned long p3_outp_set;    /* Output pin set register */
  volatile unsigned long p3_outp_clr;    /* Output pin clear register */
  volatile unsigned long p3_outp_state;  /* Output pin state register */
  volatile unsigned long p2_dir_set;     /* GPIO direction set register */
  volatile unsigned long p2_dir_clr;     /* GPIO direction clear register */
  volatile unsigned long p2_dir_state;   /* GPIO direction state register */
  volatile unsigned long p2_inp_state; /* SDRAM-Input pin state register*/
  volatile unsigned long p2_outp_set;  /* SDRAM-Output pin set register */
  volatile unsigned long p2_outp_clr;  /* SDRAM-Output pin clear register*/
  volatile unsigned long p2_mux_set;     /* PIO mux control set register*/
  volatile unsigned long p2_mux_clr;     /* PIO mux control clear register*/
  volatile unsigned long p2_mux_state;   /* PIO mux state register */
  volatile unsigned long reserved1 [3];
  volatile unsigned long p0_inp_state;    /* P0 GPIOs pin read register */
  volatile unsigned long p0_outp_set;    /* P0 GPIOs output set register */
  volatile unsigned long p0_outp_clr;    /* P0 GPIOs output clear register */
  volatile unsigned long p0_outp_state;  /* P0 GPIOs output state register */
  volatile unsigned long p0_dir_set;     /* P0 GPIOs direction set reg */
  volatile unsigned long p0_dir_clr;     /* P0 GPIOs direction clear reg */
  volatile unsigned long p0_dir_state;   /* P0 GPIOs direction state reg */
  volatile unsigned long reserved2;
  volatile unsigned long p1_inp_state;    /* P1 GPIOs pin read register */
  volatile unsigned long p1_outp_set;    /* P1 GPIOs output set register */
  volatile unsigned long p1_outp_clr;    /* P1 GPIOs output clear register */
  volatile unsigned long p1_outp_state;  /* P1 GPIOs output state register */
  volatile unsigned long p1_dir_set;     /* P1 GPIOs direction set reg */
  volatile unsigned long p1_dir_clr;     /* P1 GPIOs direction clear reg */
  volatile unsigned long p1_dir_state;   /* P1 GPIOs direction state reg */
  volatile unsigned long reserved3;
  volatile unsigned long reserved4 [32];
  volatile unsigned long p_mux_set;     /* PIO mux2 control set register*/
  volatile unsigned long p_mux_clr;     /* PIO mux2 control clear register*/
  volatile unsigned long p_mux_state;   /* PIO mux2 state register */
  volatile unsigned long reserved5;
  volatile unsigned long p3_mux_set;     /* PIO mux3 control set register*/
  volatile unsigned long p3_mux_clr;     /* PIO mux3 control clear register*/
  volatile unsigned long p3_mux_state;   /* PIO mux3 state register */
  volatile unsigned long reserved6;
  volatile unsigned long p0_mux_set;       /* P0 mux control set register*/
  volatile unsigned long p0_mux_clr;       /* P0 mux control clear register*/
  volatile unsigned long p0_mux_state;     /* P0 mux state register */
  volatile unsigned long reserved7;
  volatile unsigned long p1_mux_set;       /* P1 mux control set register*/
  volatile unsigned long p1_mux_clr;       /* P1 mux control clear register*/
  volatile unsigned long p1_mux_state;     /* P1 mux state register */
} GPIO_REGS_T;

/* For direction registers, a '1' is an output */
#define GPIO_DIR_OUT          0x1

/***********************************************************************
* Input Pin State Register defines
**********************************************************************/
/* Input state of GPI_pin. Where pin = 0-9 */
#define INP_STATE_GPI_00	  _BIT(0)
#define INP_STATE_GPI_01	  _BIT(1)
#define INP_STATE_GPI_02	  _BIT(2)
#define INP_STATE_GPI_03	  _BIT(3)
#define INP_STATE_GPI_04	  _BIT(4)
#define INP_STATE_GPI_05	  _BIT(5)
#define INP_STATE_GPI_06	  _BIT(6)
#define INP_STATE_GPI_07	  _BIT(7)
#define INP_STATE_GPI_08	  _BIT(8)
#define INP_STATE_GPI_09	  _BIT(9)
#define INP_STATE_GPIO_00	  _BIT(10)
#define INP_STATE_GPIO_01	  _BIT(11)
#define INP_STATE_GPIO_02	  _BIT(12)
#define INP_STATE_GPIO_03	  _BIT(13)
#define INP_STATE_GPIO_04	  _BIT(14)
#define INP_STATE_U1_RX		  _BIT(15)
#define INP_STATE_U2_HCTS	  _BIT(16)
#define INP_STATE_U2_RX		  _BIT(17)
#define INP_STATE_U3_RX		  _BIT(18)
#define INP_STATE_GPI_19_U4RX _BIT(19)
#define INP_STATE_U5_RX		  _BIT(20)
#define INP_STATE_U6_IRRX	  _BIT(21)
#define INP_STATE_U7_HCTS	  _BIT(22)
#define INP_STATE_U7_RX		  _BIT(23)
#define INP_STATE_GPIO_05	  _BIT(24)
#define INP_STATE_SPI1_DATIN  _BIT(25)
#define INP_STATE_SPI2_DATIN  _BIT(27)
#define INP_STATE_GPI_28_U3RI _BIT(28)

/***********************************************************************
* p3_outp_set, p3_outp_clr, and p3_outp_state register defines
**********************************************************************/
/* Following macro is used to determine bit position for GPO pin in
*  P3_OUTP_SET, P3_OUTP_CLR & P3_OUTP_STATE registers.
*  Where pin = {0-23}
*/
#define OUTP_STATE_GPO(pin)	  _BIT((pin))

/* Following macro is used to determine bit position for GPIO pin in
*  PIO_OUTP_SET, P3_OUTP_CLR & P3_OUTP_STATE registers.
*  Where pin = {0-5}
*/
#define OUTP_STATE_GPIO(pin)  _BIT(((pin) + 25))

/***********************************************************************
* GPIO Direction Register defines
**********************************************************************/
/* Following macro is used to determine bit position for GPIO pin in
*  P2_DIR_SET, P2_DIR_STATE & P2_DIR_CLR registers.
*  Where pin = {0-5}
*/
#define PIO_DIR_GPIO(pin)	 _BIT(((pin) + 25))

/* Following macro is used to determine bit position for RAM_D pin in
*  P2_DIR_SET, P2_DIR_CLR, P2_DIR_STATE, P2_INP_STATE,
*  P2_OUTP_SET, & P2_OUTP_CLR.
*  Where pin = {19-31}
*/
#define PIO_SDRAM_DIR_PIN(pin) _BIT(((pin) - 19))

/* Macro for GPIO direction muxed with the high 16 bits of the SDRAM
   data related bit locations (when configured as a GPIO) */
#define PIO_SDRAM_PIN_ALL    0x00001FFF

/***********************************************************************
* p_mux_set, p_mux_clr, p_mux_state register defines
**********************************************************************/
/* Muxed PIO#0 pin state defines */
#define P_I2STXSDA1_MAT31     _BIT(2)
#define P_I2STXCLK1_MAT30     _BIT(3)
#define P_I2STXWS1_CAP30      _BIT(4)
#define P_SPI2DATAIO_MOSI1    _BIT(5)
#define P_SPI2DATAIN_MISO1    _BIT(6)
#define P_SPI2CLK_SCK1        _BIT(8)
#define P_SPI1DATAIO_SSP0_MOSI _BIT(9)
#define P_SPI1DATAIN_SSP0_MISO _BIT(10)
#define P_SPI1CLK_SCK0        _BIT(12)
#define P_MAT21_PWM36         _BIT(13)
#define P_MAT20_PWM35         _BIT(14)
#define P_U7TX_MAT11          _BIT(15)
#define P_MAT03_PWM34         _BIT(17)
#define P_MAT02_PWM33         _BIT(18)
#define P_MAT01_PWM32         _BIT(19)
#define P_MAT00_PWM31         _BIT(20)

/***********************************************************************
* p0_mux_set, p0_mux_clr, p0_mux_state register defines
**********************************************************************/

/* Following macro is used to determine bit position for a P0 GPIO pin
   used with the p0_xxx registers for pins P0_0 to P0_7*/
#define OUTP_STATE_GPIO_P0(pin)	  _BIT((pin))

/* P0 pin mux defines (0 = GPIO, 1 = alternate function) */
#define P0_GPOP0_I2SRXCLK1		  _BIT(0)
#define P0_GPOP1_I2SRXWS1	      _BIT(1)
#define P0_GPOP2_I2SRXSDA0	      _BIT(2)
#define P0_GPOP3_I2SRXCLK0	      _BIT(3)
#define P0_GPOP4_I2SRXWS0	      _BIT(4)
#define P0_GPOP5_I2STXSDA0	      _BIT(5)
#define P0_GPOP6_I2STXCLK0	      _BIT(6)
#define P0_GPOP7_I2STXWS0	      _BIT(7)
#define P0_ALL						0xFF

/***********************************************************************
* p1_mux_set, p1_mux_clr, p1_mux_state register defines
**********************************************************************/

/* Following macro is used to determine bit position for a P1 GPIO pin
   used with the p1_xxx registers for pins P1_0 to P1_23*/
#define OUTP_STATE_GPIO_P1(pin)	  _BIT((pin))

/* Mask for all GPIO P1 bits */
#define P1_ALL                    0x00FFFFFF

/* Macro pointing to GPIO registers */
#define GPIO  ((GPIO_REGS_T *)(GPIO_BASE))

/***********************************************************************
* p2_mux_set, p2_mux_clr, p2_mux_state register defines
**********************************************************************/
/* Muxed PIO#2 pin state defines */
#define P2_GPIO05_SSEL0			_BIT(5)
#define P2_GPIO04_SSEL1			_BIT(4)
#define P2_SDRAMD19D31_GPIO		_BIT(3)
#define P2_GPO21_U4TX	        _BIT(2)
#define P2_GPIO03_KEYROW7		_BIT(1)
#define P2_GPIO02_KEYROW6		_BIT(0)

/***********************************************************************
* p3_mux_set, p3_mux_clr, p3_mux_state register defines
**********************************************************************/
/* Muxed PIO#3 pin states, first column is '0' state, second is '1' */
#define P3_GPO2_MAT10          _BIT(2)
#define P3_GPO6_PWM43          _BIT(6)
#define P3_GPO8_PWM42          _BIT(8)
#define P3_GPO9_PWM41          _BIT(9)
#define P3_GPO10_PWM36         _BIT(10)
#define P3_GPO12_PWM35         _BIT(12)
#define P3_GPO13_PWM34         _BIT(13)
#define P3_GPO15_PWM33         _BIT(15)
#define P3_GPO16_PWM32         _BIT(16)
#define P3_GPO18_PWM31         _BIT(18)


#endif /* __LPC3250_H */

