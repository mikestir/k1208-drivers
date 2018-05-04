/*
 *  SPI NET ENC28J60 device driver for K1208/Amiga 1200
 *
 *  Copyright (C) 2018 Mike Stirling
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef _ENC28J60_H
#define _ENC28J60_H

//#include <stdint.h>

#define ENC28J60_BANK0		(0 << 5)			/*!< Register in bank 0 */
#define ENC28J60_BANK1		(1 << 5)			/*!< Register in bank 1 */
#define ENC28J60_BANK2		(2 << 5)			/*!< Register in bank 2 */
#define ENC28J60_BANK3		(3 << 5)			/*!< Register in bank 3 */

#define ENC28J60_ADDRMASK	0x1f				/*!< Register address mask */
#define ENC28J60_BANKMASK	0x60				/*!< Register bank mask */

#define ENC28J60_MIIREG		(0 << 7)			/*!< Register is in the MII block */
#define ENC28J60_MACREG		(1 << 7)			/*!< Register is in the MAC block */

/* BANK 0 */

#define ERDPTL				(0x00 | ENC28J60_BANK0)
#define ERDPTH				(0x01 | ENC28J60_BANK0)
#define ERDPT				ERDPTL
#define EWRPTL				(0x02 | ENC28J60_BANK0)
#define EWRPTH				(0x03 | ENC28J60_BANK0)
#define EWRPT				EWRPT
#define ETXSTL				(0x04 | ENC28J60_BANK0)
#define ETXSTH				(0x05 | ENC28J60_BANK0)
#define ETXST				ETXSTL
#define ETXNDL				(0x06 | ENC28J60_BANK0)
#define ETXNDH				(0x07 | ENC28J60_BANK0)
#define ETXND				ETXNDL
#define ERXSTL				(0x08 | ENC28J60_BANK0)
#define ERXSTH				(0x09 | ENC28J60_BANK0)
#define ERXST				ERXSTL
#define ERXNDL				(0x0A | ENC28J60_BANK0)
#define ERXNDH				(0x0B | ENC28J60_BANK0)
#define ERXND				ERXNDL
#define ERXRDPTL			(0x0C | ENC28J60_BANK0)
#define ERXRDPTH			(0x0D | ENC28J60_BANK0)
#define ERXRDPT				ERXRDPTL
#define ERXWRPTL			(0x0E | ENC28J60_BANK0)
#define ERXWRPTH			(0x0F | ENC28J60_BANK0)
#define ERXWRPT				ERXWRPTL
#define EDMASTL				(0x10 | ENC28J60_BANK0)
#define EDMASTH				(0x11 | ENC28J60_BANK0)
#define EDMAST				EDMASTL
#define EDMANDL				(0x12 | ENC28J60_BANK0)
#define EDMANDH				(0x13 | ENC28J60_BANK0)
#define EDMAND				EDMANDL
#define EDMADSTL			(0x14 | ENC28J60_BANK0)
#define EDMADSTH			(0x15 | ENC28J60_BANK0)
#define EDMADST				EDMADSTL
#define EDMACSL				(0x16 | ENC28J60_BANK0)
#define EDMACSH				(0x17 | ENC28J60_BANK0)
#define EDMACS				EDMACS

/* BANK 1 */

#define EHT0				(0x00 | ENC28J60_BANK1)
#define EHT1				(0x01 | ENC28J60_BANK1)
#define EHT2				(0x02 | ENC28J60_BANK1)
#define EHT3				(0x03 | ENC28J60_BANK1)
#define EHT4				(0x04 | ENC28J60_BANK1)
#define EHT5				(0x05 | ENC28J60_BANK1)
#define EHT6				(0x06 | ENC28J60_BANK1)
#define EHT7				(0x07 | ENC28J60_BANK1)
#define EHT					EHT0
#define EPMM0				(0x08 | ENC28J60_BANK1)
#define EPMM1				(0x09 | ENC28J60_BANK1)
#define EPMM2				(0x0A | ENC28J60_BANK1)
#define EPMM3				(0x0B | ENC28J60_BANK1)
#define EPMM4				(0x0C | ENC28J60_BANK1)
#define EPMM5				(0x0D | ENC28J60_BANK1)
#define EPMM6				(0x0E | ENC28J60_BANK1)
#define EPMM7				(0x0F | ENC28J60_BANK1)
#define EPMM				EPMM0
#define EPMCSL				(0x10 | ENC28J60_BANK1)
#define EPMCSH				(0x11 | ENC28J60_BANK1)
#define EPMCS				EPMCSL
#define EPMOL				(0x14 | ENC28J60_BANK1)
#define EPMOH				(0x15 | ENC28J60_BANK1)
#define EPMO				EPMOL
#define ERXFCON				(0x18 | ENC28J60_BANK1)
#define EPKTCNT				(0x19 | ENC28J60_BANK1)

/* BANK 2 */

#define MACON1				(0x00 | ENC28J60_BANK2 | ENC28J60_MACREG)
#define MACON2				(0x01 | ENC28J60_BANK2 | ENC28J60_MACREG)
#define MACON3				(0x02 | ENC28J60_BANK2 | ENC28J60_MACREG)
#define MACON4				(0x03 | ENC28J60_BANK2 | ENC28J60_MACREG)
#define MABBIPG				(0x04 | ENC28J60_BANK2 | ENC28J60_MACREG)
#define MAIPGL				(0x06 | ENC28J60_BANK2 | ENC28J60_MACREG)
#define MAIPGH				(0x07 | ENC28J60_BANK2 | ENC28J60_MACREG)
#define MAIPG				MAIPGL
#define MACLCON1			(0x08 | ENC28J60_BANK2 | ENC28J60_MACREG)
#define MACLCON2			(0x09 | ENC28J60_BANK2 | ENC28J60_MACREG)
#define MAMXFLL				(0x0A | ENC28J60_BANK2 | ENC28J60_MACREG)
#define MAMXFLH				(0x0B | ENC28J60_BANK2 | ENC28J60_MACREG)
#define MAMXFL				MAMXFLL
#define MICMD				(0x12 | ENC28J60_BANK2 | ENC28J60_MACREG)
#define MIREGADR			(0x14 | ENC28J60_BANK2 | ENC28J60_MACREG)
#define MIWRL				(0x16 | ENC28J60_BANK2 | ENC28J60_MACREG)
#define MIWRH				(0x17 | ENC28J60_BANK2 | ENC28J60_MACREG)
#define MIWR				MIWRL
#define MIRDL				(0x18 | ENC28J60_BANK2 | ENC28J60_MACREG)
#define MIRDH				(0x19 | ENC28J60_BANK2 | ENC28J60_MACREG)
#define MIRD				MIRDL

/* BANK 3 */

#define MAADR5				(0x00 | ENC28J60_BANK3 | ENC28J60_MACREG)
#define MAADR6				(0x01 | ENC28J60_BANK3 | ENC28J60_MACREG)
#define MAADR3				(0x02 | ENC28J60_BANK3 | ENC28J60_MACREG)
#define MAADR4				(0x03 | ENC28J60_BANK3 | ENC28J60_MACREG)
#define MAADR1				(0x04 | ENC28J60_BANK3 | ENC28J60_MACREG)
#define MAADR2				(0x05 | ENC28J60_BANK3 | ENC28J60_MACREG)
#define EBSTSD				(0x06 | ENC28J60_BANK3)
#define EBSTCON				(0x07 | ENC28J60_BANK3)
#define EBSTCSL				(0x08 | ENC28J60_BANK3)
#define EBSTCSH				(0x09 | ENC28J60_BANK3)
#define EBSTCS				EBSTCSL
#define MISTAT				(0x0A | ENC28J60_BANK3 | ENC28J60_MACREG)
#define EREVID				(0x12 | ENC28J60_BANK3)
#define ECOCON				(0x15 | ENC28J60_BANK3)
#define EFLOCON				(0x17 | ENC28J60_BANK3)
#define EPAUSL				(0x18 | ENC28J60_BANK3)
#define EPAUSH				(0x19 | ENC28J60_BANK3)
#define EPAUS				EPAUSL

/* GLOBALS */

#define EIE					(0x1B)
#define EIR					(0x1C)
#define ESTAT				(0x1D)
#define ECON2				(0x1E)
#define ECON1				(0x1F)

// Start of global registers
#define ENC28J60_GLOBAL_START	EIE

/* PHY registers (accessed via MAC) */

#define PHCON1				(0x00)
#define PHSTAT1				(0x01)
#define PHID1				(0x02)
#define PHID2				(0x03)
#define PHCON2				(0x10)
#define PHSTAT2				(0x11)
#define PHIE				(0x12)
#define PHIR				(0x13)
#define PHLCON				(0x14)

/* ETH/MAC/MII Register bit fields */

// EIE
#define EIE_EXERIE			(1 << 0)
#define EIE_TXERIE			(1 << 1)
#define EIE_TXIE			(1 << 3)
#define EIE_LINKIE			(1 << 4)
#define EIE_DMAIE			(1 << 5)
#define EIE_PKTIE			(1 << 6)
#define EIE_INTIE			(1 << 7)

// EIR
#define EIR_RXERIF			(1 << 0)
#define EIR_TXERIF			(1 << 1)
#define EIR_TXIF			(1 << 3)
#define EIR_LINKIF			(1 << 4)
#define EIR_DMAIF			(1 << 5)
#define EIR_PKTIF			(1 << 6)
#define EIR_INTIF			(1 << 7)

// ESTAT
#define ESTAT_CLKRDY		(1 << 0)
#define ESTAT_TXABRT		(1 << 1)
#define ESTAT_RXBUSY		(1 << 2)
#define ESTAT_LATECOL		(1 << 4)
#define ESTAT_BUFER			(1 << 6)
#define ESTAT_INT			(1 << 7)

// ECON2
#define ECON2_VRPS			(1 << 3)
#define ECON2_PWRSV			(1 << 5)
#define ECON2_PKTDEC		(1 << 6)
#define ECON2_AUTOINC		(1 << 7)

// ECON1
#define ECON1_BSEL0			(1 << 0)
#define ECON1_BSEL1			(1 << 1)
#define ECON1_RXEN			(1 << 2)
#define ECON1_TXRTS			(1 << 3)
#define ECON1_CSUMEN		(1 << 4)
#define ECON1_DMAST			(1 << 5)
#define ECON1_RXRST			(1 << 6)
#define ECON1_TXRST			(1 << 7)

// ERXFCON
#define ERXFCON_BCEN		(1 << 0)
#define ERXFCON_MCEN		(1 << 1)
#define ERXFCON_HTEN		(1 << 2)
#define ERXFCON_MPEN		(1 << 3)
#define ERXFCON_PMEN		(1 << 4)
#define ERXFCON_CRCEN		(1 << 5)
#define ERXFCON_ANDOR		(1 << 6)
#define ERXFCON_UCEN		(1 << 7)

// MACON1
#define MACON1_MARXEN		(1 << 0)
#define MACON1_PASSALL		(1 << 1)
#define MACON1_RXPAUS		(1 << 2)
#define MACON1_TXPAUS		(1 << 3)

// MACON3
#define MACON3_FULDPX		(1 << 0)
#define MACON3_FRMLNEN		(1 << 1)
#define MACON3_HFRMEN		(1 << 2)
#define MACON3_PHDREN		(1 << 3)
#define MACON3_TXCRCEN		(1 << 4)
#define MACON3_PADCFG0		(1 << 5)
#define MACON3_PADCFG1		(1 << 6)
#define MACON3_PADCFG2		(1 << 7)

// MACON4
#define MACON4_NOBKOFF		(1 << 4)
#define MACON4_BPEN			(1 << 5)
#define MACON4_DEFER		(1 << 6)

// MICMD
#define MICMD_MIIRD			(1 << 0)
#define MICMD_MIISCAN		(1 << 1)

// EBSTCON
#define EBSTCON_BISTST		(1 << 0)
#define EBSTCON_TME			(1 << 1)
#define EBSTCON_TMSEL0		(1 << 2)
#define EBSTCON_TMSEL1		(1 << 3)
#define EBSTCON_PSEL		(1 << 4)
#define EBSTCON_PSV0		(1 << 5)
#define EBSTCON_PSV1		(1 << 6)
#define EBSTCON_PSV2		(1 << 7)

// MISTAT
#define MISTAT_BUSY			(1 << 0)
#define MISTAT_SCAN			(1 << 1)
#define MISTAT_NVALID		(1 << 2)

// ECOCON
#define ECOCON_COCON0		(1 << 0)
#define ECOCON_COCON1		(1 << 1)
#define ECOCON_COCON2		(1 << 2)

// EFLOCON
#define EFLOCON_FCEN0		(1 << 0)
#define EFLOCON_FCEN1		(1 << 1)
#define EFLOCON_FULDPXS		(1 << 2)

/* PHY register bit fields */

// PHCON1
#define PHCON1_PDPXMD		(1 << 8)
#define PHCON1_PPWRSV		(1 << 11)
#define PHCON1_PLOOPBK		(1 << 14)
#define PHCON1_PRST			(1 << 15)

// PHSTAT1
#define PHSTAT1_JBSTAT		(1 << 1)
#define PHSTAT1_LLSTAT		(1 << 2)
#define PHSTAT1_PHDPX		(1 << 11)
#define PHSTAT1_PFDPX		(1 << 12)

// PHCON2
#define PHCON2_HDLDIS		(1 << 8)
#define PHCON2_JABBER		(1 << 10)
#define PHCON2_TXDIS		(1 << 13)
#define PHCON2_FRCLNK		(1 << 14)

// PHSTAT2
#define PHSTAT2_PLRITY		(1 << 5)
#define PHSTAT2_DPXSTAT		(1 << 9)
#define PHSTAT2_LSTAT		(1 << 10)
#define PHSTAT2_COLSTAT		(1 << 11)
#define PHSTAT2_RXSTAT		(1 << 12)
#define PHSTAT2_TXSTAT		(1 << 13)

// PHIE
#define PHIE_PGEIE			(1 << 1)
#define PHIE_PLNKIE			(1 << 4)

// PHIF
#define PHIF_PGIF			(1 << 2)
#define PHIF_PLNKIF			(1 << 4)

// PHLCON
#define PHLCON_STRCH		(1 << 1)
#define PHLCON_LFRQ0		(1 << 2)
#define PHLCON_LFRQ1		(1 << 3)
#define PHLCON_LBCFG0		(1 << 4)
#define PHLCON_LBCFG1		(1 << 5)
#define PHLCON_LBCFG2		(1 << 6)
#define PHLCON_LBCFG3		(1 << 7)
#define PHLCON_LACFG0		(1 << 8)
#define PHLCON_LACFG1		(1 << 9)
#define PHLCON_LACFG2		(1 << 10)
#define PHLCON_LACFG3		(1 << 11)


// SPI operations
#define ENC28J60_SPI_RCR	(0x00)
#define ENC28J60_SPI_RBM	(0x3a)
#define ENC28J60_SPI_WCR	(0x40)
#define ENC28J60_SPI_WBM	(0x7a)
#define ENC28J60_SPI_BFS	(0x80)
#define ENC28J60_SPI_BFC	(0xa0)
#define ENC28J60_SPI_SRC	(0xff)

/* Buffer pointers */
/* NOTE: Errata 3 - place RX buffer first */

#define RXSTART_INIT		(0x0000)		/*!< Start address for receive buffers */
#define RXSTOP_INIT			(0x19ff)		/*!< End address for receive buffers */
#define TXSTART_INIT		(0x1a00)		/*!< Start address for transmit buffers */

#define MAX_FRAMELEN		1518

/* Transmit packet control bytes */

#define ENC28J60_TX_PHUGEEN		(1 << 3)
#define ENC28J60_TX_PPADEN		(1 << 2)
#define ENC28J60_TX_PCRCEN		(1 << 1)
#define ENC28J60_TX_POVERRIDE	(1 << 0)

/*! Receive packet status header */
typedef struct {
	uint16_t		next_packet;
	uint16_t		length;
	uint16_t		status;
} __attribute__ ((packed)) enc28j60_rx_status_t;

/* Receive status flags */

#define ENC28J60_RXSTATUS_DROP_EVENT	(1 << 0)
#define ENC28J60_RXSTATUS_CARRIER_EVENT	(1 << 2)
#define ENC28J60_RXSTATUS_CRC_ERROR		(1 << 4)
#define ENC28J60_RXSTATUS_LENGTH_ERROR	(1 << 5)
#define ENC28J60_RXSTATUS_LENGTH_RANGE	(1 << 6)
#define ENC28J60_RXSTATUS_OK			(1 << 7)
#define ENC28J60_RXSTATUS_MULTICAST		(1 << 8)
#define ENC28J60_RXSTATUS_BROADCAST		(1 << 9)
#define ENC28J60_RXSTATUS_DRIBBLE		(1 << 10)
#define ENC28J60_RXSTATUS_CTRL_FRAME	(1 << 11)
#define ENC28J60_RXSTATUS_PAUSE_CTRL	(1 << 12)
#define ENC28J60_RXSTATUS_UNKNOWN_OP	(1 << 13)
#define ENC28J60_RXSTATUS_VLAN_TYPE		(1 << 14)

/*! Transmit status vector */
typedef struct {
	uint16_t		length;
	uint16_t		status1;
	uint16_t		bytes_on_wire;
	uint8_t			status2;
} __attribute__((packed)) enc28j60_tx_status_t;

#define ENC28J60_TXSTATUS1_CRC_ERROR	(1 << 4)
#define ENC28J60_TXSTATUS1_LENGTH_ERROR	(1 << 5)
#define ENC28J60_TXSTATUS1_LENGTH_RANGE	(1 << 6)
#define ENC28J60_TXSTATUS1_OK			(1 << 7)
#define ENC28J60_TXSTATUS1_MULTICAST	(1 << 8)
#define ENC28J60_TXSTATUS1_BROADCAST	(1 << 9)
#define ENC28J60_TXSTATUS1_DEFER		(1 << 10)
#define ENC28J60_TXSTATUS1_EXCESS_DEFER	(1 << 11)
#define ENC28J60_TXSTATUS1_EXCESS_COL	(1 << 12)
#define ENC28J60_TXSTATUS1_LATE_COL		(1 << 13)
#define ENC28J60_TXSTATUS1_GIANT		(1 << 14)

#define ENC28J60_TXSTATUS2_CTRL_FRAME	(1 << 0)
#define ENC28J60_TXSTATUS2_PAUSE_CTRL	(1 << 1)
#define ENC28J60_TXSTATUS2_BACKPRESSURE	(1 << 2)
#define ENC28J60_TXSTATUS2_VLAN_TYPE	(1 << 3)

#endif
