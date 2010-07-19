/*
 * Copyright (C) 2008 by NXP Semiconductors
 * All rights reserved.
 *
 * @Author: Kevin Wells
 * @Descr: LPC3250 clock and power query functions
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
#include <div64.h>

//DECLARE_GLOBAL_DATA_PTR;


/* CLK divider values for HCLK based on selected register value */
static unsigned int hclkdivs[4] =
{
  1, 2, 4, 4
};

/* Post divider values for PLLs based on selected register value */
static unsigned int pll_postdivs[4] =
{
  1, 2, 4, 8
};

/*
 * Structure used for setting up the HCLK PLL
 */
typedef struct
{
  /* (0) = analog off, (!0) = on */
  int analog_on;
  /* (0) = CCO clock sent to post divider, (!0) = PLL input clock sent
  to post div */
  int cco_bypass_b15;
  /* (0) = PLL out from post divider, (!0) = PLL out bypasses post
  divider */
  int direct_output_b14;
  /* (0) = use CCO clock, (!0) = use FCLKOUT */
  int fdbk_div_ctrl_b13;
  /* Must be 1, 2, 4, or 8 */
  int pll_p;
  /* Must be 1, 2, 3, or 4 */
  int pll_n;
  /* Feedback multiplier 1-256 */
  unsigned int pll_m;
} CLKPWR_HCLK_PLL_SETUP_T;

/*
 * Determine if PLL rate is valid
 */
static unsigned int clkpwr_check_pll_setup(unsigned int ifreq,
						CLKPWR_HCLK_PLL_SETUP_T *pllsetup)
{
  unsigned long long i64freq, p, m, n, fcco, fref,
		cfreq, div_cfreq, div_fcco, div_fref;
  int mode;

  /* PLL requirements */
  /* ifreq must be >= 1MHz and <= 20MHz */
  /* FCCO must be >= 156MHz and <= 320MHz */
  /* FREF must be >= 1MHz and <= 27MHz. */
  /* Assume the passed input data is not valid */

	fcco = fref = cfreq = 0;
  	div_cfreq = div_fcco = div_fref = 0;

  /* Work with 64-bit values to prevent overflow */
  i64freq = (unsigned long long) ifreq;
  m = (unsigned long long) pllsetup->pll_m;
  n = (unsigned long long) pllsetup->pll_n;
  p = (unsigned long long) pllsetup->pll_p;

  /* Get components of the PLL register */
  mode = (pllsetup->cco_bypass_b15 << 2) |
         (pllsetup->direct_output_b14 << 1) |
         pllsetup->fdbk_div_ctrl_b13;

  switch (mode)
  {
    case 0x0: /* Non-integer mode */
      cfreq = m * i64freq;
      div_cfreq = 2 * p * n;
      fcco = m * i64freq;
      div_fcco = n;
      fref = i64freq;
      div_fref = n;
      break;

    case 0x1: /* integer mode */
      cfreq = m * i64freq;
      div_cfreq = n;
      fcco = m * i64freq;
      div_fcco = 2 * p * n;
      fref = i64freq;
      div_fref = n;
      break;

    case 0x2:
    case 0x3: /* Direct mode */
      cfreq = m * i64freq;
      div_cfreq = n;
      fcco = m * i64freq;
      div_fcco = n;
      fref = i64freq;
      div_fref = n;
      break;

    case 0x4:
    case 0x5: /* Bypass mode */
      cfreq = i64freq;
      div_cfreq = 2 * p;
      fcco = 156000000;
      fref = 1000000;
      break;

    case 0x6:
    case 0x7: /* Direct bypass mode */
      cfreq = i64freq;
      fcco = 156000000;
      fref = 1000000;
      break;
  }

  if (div_cfreq != 0)
	  do_div(cfreq, div_cfreq);
  
  if (div_fcco != 0)
	  do_div(fcco, div_fcco);

  if (div_fref != 0)
	  do_div(fref, div_fref);

  if ((fcco < 156000000) || (fcco > 320000000))
  {
    /* not a valid range */
    cfreq = 0;
  }

  if ((fref < 1000000) || (fref > 27000000))
  {
    /* not a valid range */
    cfreq = 0;
  }

  return (int) cfreq;
}

/*
 * Get PLL values and compute PLL rate
 */
static unsigned int clkpwr_pll_rate_from_val(unsigned int osc_rate,
						unsigned int val)
{
  CLKPWR_HCLK_PLL_SETUP_T pllcfg;

  /* Get components of the PLL register */
  pllcfg.cco_bypass_b15 = 0;
  pllcfg.direct_output_b14 = 0;
  pllcfg.fdbk_div_ctrl_b13 = 0;
  if ((val & CLKPWR_HCLKPLL_CCO_BYPASS) != 0)
  {
    pllcfg.cco_bypass_b15 = 1;
  }
  if ((val & CLKPWR_HCLKPLL_POSTDIV_BYPASS) != 0)
  {
    pllcfg.direct_output_b14 = 1;
  }
  if ((val & CLKPWR_HCLKPLL_FDBK_SEL_FCLK) != 0)
  {
    pllcfg.fdbk_div_ctrl_b13 = 1;
  }
  pllcfg.pll_m = 1 + ((val >> 1) & 0xFF);
  pllcfg.pll_n = 1 + ((val >> 9) & 0x3);
  pllcfg.pll_p = pll_postdivs[((val >> 11) & 0x3)];

  return clkpwr_check_pll_setup(osc_rate, &pllcfg);
}

/*
 * Get PLL rate from USB or HCLK PLL
 */
static unsigned int clkpwr_pll_rate(unsigned int osc_rate,
				unsigned int *pPllreg)
{
  return clkpwr_pll_rate_from_val(osc_rate, *pPllreg);
}

/*
 * Clock rate fetch function
 */
unsigned int sys_get_rate(CLKPWR_BASE_CLOCK_T clkid)
{
	unsigned int sys_clk, ddr_clock, ddr_hclk_div, hclkpll_clk;
	unsigned int periph_clk, tmp, hclk1_clk, arm1_clk;
	unsigned int hclk_clk, arm_clk, clkrate;

	/* Is PLL397 oscillator being used? */
	if ((CLKPWR->clkpwr_sysclk_ctrl & CLKPWR_SYSCTRL_USEPLL397) != 0)
	{
		/* PLL397 is used */
		sys_clk = CLOCK_OSC_FREQ * 397;
	}
	else
	{
		sys_clk = MAIN_OSC_FREQ;
	}

	/* Compute HCLK DDR divider */
	ddr_hclk_div = 0;
	if ((CLKPWR->clkpwr_sdramclk_ctrl & CLKPWR_SDRCLK_USE_DDR) != 0)
	{
		/* DDR is being used */
		if ((CLKPWR->clkpwr_hclk_div & CLKPWR_HCLKDIV_DDRCLK_NORM) != 0)
		{
			ddr_hclk_div = 1;
		}
		else if ((CLKPWR->clkpwr_hclk_div &
			CLKPWR_HCLKDIV_DDRCLK_HALF) != 0)
		{
			ddr_hclk_div = 2;
		}
	}
	else
	{
		/* SDRAM is being used */
		tmp = CLKPWR->clkpwr_hclk_div & CLKPWR_HCLKDIV_DIV_2POW(0x3);
		ddr_hclk_div = hclkdivs[tmp] - 1;
	}

	/* Is the device in run mode? */
	if ((CLKPWR->clkpwr_pwr_ctrl & CLKPWR_SELECT_RUN_MODE) != 0)
	{
		/* In run mode */

		/* Compute HCLK PLL rate */
		hclkpll_clk = clkpwr_pll_rate(sys_clk,
			(unsigned int *) &CLKPWR->clkpwr_hclkpll_ctrl);

		/* Base DDR rate */
		ddr_clock = hclkpll_clk;

		/* Base peripheral clock rate */
		tmp = 1 + ((CLKPWR->clkpwr_hclk_div >> 2) & 0x1F);
		periph_clk = hclkpll_clk / tmp;

		/* Base HCLK rate (when not using peripheral clock */
		hclk1_clk = hclkpll_clk /
			hclkdivs[CLKPWR_HCLKDIV_DIV_2POW(CLKPWR->clkpwr_hclk_div)];

		/* Base ARM clock (when not using peripheral clock */
		arm1_clk = hclkpll_clk;
	}
	else
	{
		/* In direct-run mode */

		/* Base DDR rate */
		ddr_clock = sys_clk;

		/* Base peripheral clock rate */
		periph_clk = sys_clk;

		/* Base HCLK rate (when not using peripheral clock */
		hclk1_clk = sys_clk;

		/* Base ARM clock (when not using peripheral clock */
		arm1_clk = sys_clk;
	}

	/* Compute SDRAM/DDR clock */
	ddr_clock = ddr_clock / (ddr_hclk_div + 1);

	/* Compute HCLK and ARM clock rates */
	if ((CLKPWR->clkpwr_pwr_ctrl & CLKPWR_CTRL_FORCE_PCLK) != 0)
	{
		/* HCLK and ARM clock run from peripheral clock */
		hclk_clk = periph_clk;
		arm_clk = periph_clk;
	}
	else
	{
		/* Normal clock is used for HCLK and ARM clock */
		hclk_clk = hclk1_clk;
		arm_clk = arm1_clk;
	}

	/* Determine rates */
	switch (clkid)
	{
		case CLKPWR_MAINOSC_CLK:
			/* Main oscillator rate */
			clkrate = MAIN_OSC_FREQ;
			break;

		case CLKPWR_RTC_CLK:
			/* RTC oscillator rate */
			clkrate = CLOCK_OSC_FREQ;
			break;

		case CLKPWR_SYSCLK:
			/* System oscillator (main osc or PLL397) rate */
			clkrate = sys_clk;
			break;

		case CLKPWR_ARM_CLK:
			clkrate = arm_clk;
			break;

		case CLKPWR_HCLK:
			clkrate = hclk_clk;
			break;

		case CLKPWR_PERIPH_CLK:
			clkrate = periph_clk;
			break;

		case CLKPWR_USB_HCLK_SYS:
			clkrate = 0; // Not supported
			break;

		case CLKPWR_48M_CLK:
			clkrate = 0; // Not supported
			break;

		case CLKPWR_DDR_CLK:
			clkrate = ddr_clock;
			break;

		case CLKPWR_MSSD_CLK:
			clkrate = hclk_clk;
			break;

		default:
			clkrate = 0;
			break;
	}

	return clkrate;
}



