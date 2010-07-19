/***********************************************************************
 * $Id::$
 *
 * Project: Embedded Artists LPC3250 OEM Board definitions
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

#ifndef EA3250_PRV_H
#define EA3250_PRV_H


// Enable the following define to setup for RMII mode
#define USE_PHY_RMII

// PHY address (configured via PHY ADRx pins)
#define PHYDEF_PHYADDR           0x0001

//------------------------------------------------------------------------------
// PHY register and bitfield data
//------------------------------------------------------------------------------

// PHY Register indices
#define PHY_REG_BMCR             0x00        // Basic Mode Control Register
#define PHY_REG_BMSR             0x01        // Basic Mode Status Register
#define PHY_REG_IDR1             0x02        // PHY Identifier 1
#define PHY_REG_IDR2             0x03        // PHY Identifier 2
#define PHY_REG_ANAR             0x04        // Auto-Negotiation Advertisement
#define PHY_REG_ANLPAR           0x05        // Auto-Neg. Link Partner Abitily
#define PHY_REG_ANER             0x06        // Auto-Neg. Expansion Register
#define PHY_REG_ANNPTR           0x07        // Auto-Neg. Next Page TX

// BMCR register specific control bits
#define PHY_BMCR_RESET_BIT         0x8000    // Reset bit
#define PHY_BMCR_SPEED_BIT         0x2000    // 100M speed enable bit
#define PHY_BMCR_AUTON_BIT         0x1000    // Auto-negotiation rate bit

// BMSR register specific control bits
#define PHY_BMSR_LINKUP_STATUS     0x0004    // Link up status bit
#define PHY_BMSR_AUTON_COMPLETE    0x0020    // Auto-negotiation complete bit
#define PHY_BMSR_10M_HALF          0x0800    // 10MBase with half duplex support
#define PHY_BMSR_10M_FULL          0x1000    // 10MBase with full duplex support
#define PHY_BMSR_TX_HALF           0x2000    // TX with half duplex support
#define PHY_BMSR_TX_FULL           0x4000    // TX with full duplex support
#define PHY_BMSR_T4_ABLE           0x8000    // T4 able

// Maximum ethernet frame size, maximum RX and TX packets
#define ENET_MAXF_SIZE             1536
#define ENET_MAX_TX_PACKETS        16
#define ENET_MAX_RX_PACKETS        16



#endif /* EA3250_PRV_H */
