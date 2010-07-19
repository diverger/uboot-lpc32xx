/*
 * Copyright (C) 2008 by NXP Semiconductors
 * All rights reserved.
 *
 * @Author: Kevin Wells
 * @Descr: LPC3250 serial port functions
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <common.h>
#include <lpc3250.h>

DECLARE_GLOBAL_DATA_PTR;

/*
 * ABS function
 */
int serial_abs(int v1, int v2)
{
  if (v1 > v2)
  {
    return v1 - v2;
  }

  return v2 - v1;
}

/*
 * Find the best UART clock divider to get the desired port rate
 */
void serial_getdiv(u32 baudrate,
			unsigned int *xdiv,
			unsigned int *ydiv)
{
	unsigned int clkrate, savedclkrate, diff, basepclk;
	int idxx, idyy;

	/* Get the clock rate for the UART block */
	basepclk = sys_get_rate(CLKPWR_PERIPH_CLK) >> 4;

	/* Find the best divider */
	*xdiv = *ydiv = 0;
	savedclkrate = 0;
	diff = 0xFFFFFFFF;
	for (idxx = 1; idxx < 0xFF; idxx++)
	{
		for (idyy = idxx; idyy < 0xFF; idyy++)
		{
			clkrate = (basepclk * idxx) / idyy;
			if (serial_abs(clkrate, baudrate) < diff)
			{
				diff = serial_abs(clkrate, baudrate);
				savedclkrate = clkrate;
				*xdiv = idxx;
				*ydiv = idyy;
			}
		}
	}
}

void serial_setbrg (void)
{
	unsigned int xdiv, ydiv;

	/* Find closest baud rate for desired clock frequency */
	serial_getdiv(gd->baudrate, &xdiv, &ydiv);

	/* Set clock x/y divider for the UART */
	if (CFG_UART_SEL == UART3)
	{
		CLKPWR->clkpwr_uart3_clk_ctrl =
			CLKPWR_UART_X_DIV(xdiv) | CLKPWR_UART_Y_DIV(ydiv);
	}
	else if (CFG_UART_SEL == UART4)
	{
		CLKPWR->clkpwr_uart4_clk_ctrl =
			CLKPWR_UART_X_DIV(xdiv) | CLKPWR_UART_Y_DIV(ydiv);
	}
	else if (CFG_UART_SEL == UART5)
	{
		CLKPWR->clkpwr_uart5_clk_ctrl =
			CLKPWR_UART_X_DIV(xdiv) | CLKPWR_UART_Y_DIV(ydiv);
	}
	else if (CFG_UART_SEL == UART6)
	{
		CLKPWR->clkpwr_uart6_clk_ctrl =
			CLKPWR_UART_X_DIV(xdiv) | CLKPWR_UART_Y_DIV(ydiv);
	}
}

/*
 * Initialise the serial port with the given baudrate. The settings
 * are always 8 data bits, no parity, 1 stop bit, no start bits.
 *
 */
int serial_init (void)
{
	volatile unsigned int tmp32;
	int unum;
	UART_REGS_T *puregs = (UART_REGS_T *) CFG_UART_SEL;

	/* UART setup */

	/* Enable UART system clock */
	if (puregs == UART3)
	{
		CLKPWR->clkpwr_uart_clk_ctrl |= CLKPWR_UARTCLKCTRL_UART3_EN;
		unum = 3;
	}
	else if (puregs == UART4)
	{
		CLKPWR->clkpwr_uart_clk_ctrl |= CLKPWR_UARTCLKCTRL_UART4_EN;
		unum = 4;
	}
	else if (puregs == UART5)
	{
		CLKPWR->clkpwr_uart_clk_ctrl |= CLKPWR_UARTCLKCTRL_UART5_EN;
		unum = 5;
	}
	else if (puregs == UART6)
	{
		CLKPWR->clkpwr_uart_clk_ctrl |= CLKPWR_UARTCLKCTRL_UART6_EN;
		unum = 6;
	}

	/* Place UART in autoclock mode */
	tmp32 = UARTCNTL->clkmode & ~UART_CLKMODE_MASK(unum);
	UARTCNTL->clkmode = (tmp32 |
		UART_CLKMODE_LOAD(UART_CLKMODE_AUTO, (unum)));

	/* UART baud rate generator isn't used, so just set it to divider
	   by 1 */
	puregs->lcr |= UART_LCR_DIVLATCH_EN;
	puregs->dll_fifo = 1;
	puregs->dlm_ier = 0;
	puregs->lcr &= ~UART_LCR_DIVLATCH_EN;

	/* Setup default UART state for N81 with FIFO mode */
	puregs->lcr = UART_LCR_WLEN_8BITS;

	/* set baudrate */
	serial_setbrg();

	/* Clear FIFOs and set FIFO level */
	puregs->iir_fcr = (UART_FCR_RXFIFO_TL16 |
		UART_FCR_TXFIFO_TL0 | UART_FCR_FIFO_CTRL |
		UART_FCR_FIFO_EN | UART_FCR_TXFIFO_FLUSH |
		UART_FCR_RXFIFO_FLUSH);
	tmp32 = puregs->iir_fcr;
	tmp32 = puregs->lsr;

	/* Use automatic clocking */
//	tmp32 = UARTCNTL->clkmode & UART_CLKMODE_MASK(unum + 3);
//	UARTCNTL->clkmode = tmp32 | UART_CLKMODE_LOAD(
//        	UART_CLKMODE_AUTO, (unum + 3));  // TBD delete me

	return 0;
}

/*
 * Read a single byte from the serial port.
 */
int serial_getc (void)
{
	/* Wait for a character from the UART */
	while ((CFG_UART_SEL->lsr & UART_LSR_RDR) == 0);

	return (int) (CFG_UART_SEL->dll_fifo & 0xFF);
}

/*
 * Output a single byte to the serial port.
 */
void serial_putc (const char c)
{
	/* Wait for FIFO to become empty */
	while ((CFG_UART_SEL->lsr & UART_LSR_THRE) == 0);

	CFG_UART_SEL->dll_fifo = (u32) c;

	/* If \n, also do \r */
	if (c == '\n')
	{
		serial_putc ('\r');
	}
}

/*
 * Test whether a character is in the RX buffer
 */
int serial_tstc (void)
{
	/* Wait for a character from the UART */
	if ((CFG_UART_SEL->lsr & UART_LSR_RDR) == 0)
	{
		return 0;
	}

	return 1;
}

/*
 * Put a string to the UART
 */
void serial_puts (const char *s)
{
	while (*s)
	{
		serial_putc (*s++);
	}
}

