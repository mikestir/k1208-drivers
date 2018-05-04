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

#include <string.h>

#include "common.h"
#include "nic.h"
#include "enc28j60.h"
#include "spi.h"
#include "timer.h"

#define ENC28J60_DEBUG_PACKETS				0
#define ENC28J60_DUMP_REGS					1
#define ENC28J60_FULL_DUPLEX				1
#define ENC28J60_ERRATA13_MAX_TX_TRIES		10
#define ENC28J60_LED_MODE					0x3742		/*!< LEDA = TX/RX activity, LEDB = Link status */

static uint8_t enc28j60_current_bank = 0;		/*!< Currently selected bank */
static uint8_t enc28j60_link_status = 0;			/*!< Current link status */
static uint16_t enc28j60_next_packet;		/*!< Start of next packet in the receive buffer */

static const uint8_t enc28j60_macaddr[NIC_MACADDR_SIZE] = { 0x6c,0x78,0x75,0x73,0xe6,0x10 };

// Declarations for local (private) functions

static int enc28j60_set_bank(uint8_t addr);
static int enc28j60_set_bits(uint8_t addr, uint8_t val);
static int enc28j60_clear_bits(uint8_t addr, uint8_t val);
static int enc28j60_write_reg(uint8_t addr, uint8_t val);
static int enc28j60_read_reg(uint8_t addr);
static int enc28j60_write_reg16(uint8_t addr, uint16_t val);
static int enc28j60_read_reg16(uint8_t addr);
static int enc28j60_write_phy(uint8_t addr, uint16_t val);
static int enc28j60_read_phy(uint8_t addr);
static int enc28j60_write_buf(const uint8_t *buf, unsigned int length);
static int enc28j60_read_buf(uint8_t *buf, unsigned int length);

/// Switch register banks if necessary
/// \param addr Address of register whose bank we need to be in.
/// \return -1 on error, otherwise 0
static int enc28j60_set_bank(uint8_t addr)
{
	uint8_t bank = (addr & ENC28J60_BANKMASK) >> 5;

	// Do we need to switch banks?
	if ( (addr & ENC28J60_ADDRMASK) < ENC28J60_GLOBAL_START && bank != enc28j60_current_bank ) {
		// Yes, clear the bank bits
		if (enc28j60_clear_bits(ECON1, ECON1_BSEL1 | ECON1_BSEL0) < 0) {
			return -1;
		}
		// Set up the new bank
		if (enc28j60_set_bits(ECON1, bank) < 0) {
			return -1;
		}

		// Update stored bank number
		enc28j60_current_bank = bank;
	}

	return 0;
}

/// Set bits in one of the ENC28J60s ETH registers.
/// \param addr Address of ETH register to access.
/// \param val Bitmask to be ORed with current register contents.
/// \return -1 on error (or if address not an ETH register), otherwise 0.
static int enc28j60_set_bits(uint8_t addr, uint8_t val)
{
	uint8_t buf[2];

	if (addr & ENC28J60_MACREG) {
		// Can't do bitfields on MAC or MII registers
		return -1;
	}

	// Make sure the bank is correct
	enc28j60_set_bank(addr);

	buf[0] = (addr & 0x1f) | ENC28J60_SPI_BFS;
	buf[1] = val;

	spi_select();
	spi_write(buf, 2);
	spi_deselect();
	return 0;
}

/// Clear bits in one of the ENC28J60s ETH registers.
/// \param addr Address of ETH register to access.
/// \param val Bitmask to be NOTANDed with current register contents.
/// \return -1 on error (or if address not an ETH register), otherwise 0.
static int enc28j60_clear_bits(uint8_t addr, uint8_t val)
{
	uint8_t buf[2];

	if (addr & ENC28J60_MACREG) {
		// Can't do bitfields on MAC or MII registers
		return -1;
	}

	// Make sure the bank is correct
	enc28j60_set_bank(addr);

	buf[0] = (addr & 0x1f) | ENC28J60_SPI_BFC;
	buf[1] = val;

	spi_select();
	spi_write(buf, 2);
	spi_deselect();
	return 0;
}

/// Write a single byte to one of the ENC28J60s registers.
/// \param addr Address of register to access.
/// \param val Value to write to the register.
/// \return -1 on error, otherwise 0.
static int enc28j60_write_reg(uint8_t addr, uint8_t val)
{
	uint8_t buf[3];

	enc28j60_set_bank(addr);

	// All ETH, MAC and MII registers can be written by sending
	// address byte then data byte
	buf[0] = (addr & 0x1f) | ENC28J60_SPI_WCR;
	buf[1] = val;

	spi_select();
	spi_write(buf, 2);
	spi_deselect();

	return 0;
}

/// Read a single byte from one of the ENC28J60 registers.
/// \param addr Address of the register to access.
/// \return The unsigned char value read from the register (cast to int), -1 on error.
static int enc28j60_read_reg(uint8_t addr)
{
	uint8_t txbuf[1], rxbuf[2];

	enc28j60_set_bank(addr);

	// ETH registers are written by sending address and reading
	// one byte of data.  MAC and MII require a 2 byte read; the first
	// byte is a dummy.
	txbuf[0] = (addr & 0x1f) | ENC28J60_SPI_RCR;

	// Transfer 2 bytes for ETH registers, 3 for MAC and MII
	spi_select();
	spi_write(txbuf, 1);
	spi_read(rxbuf, (addr & ENC28J60_MACREG) ? 2 : 1);
	spi_deselect();

	return (int)rxbuf[((addr & ENC28J60_MACREG) ? 1 : 0)];
}

static int enc28j60_write_reg16(uint8_t addr, uint16_t val)
{
	int status = 0;

	status |= enc28j60_write_reg(addr + 0, val & 0xff);
	status |= enc28j60_write_reg(addr + 1, val >> 8);
	return status;
}

static int enc28j60_read_reg16(uint8_t addr)
{
	int l, h;

	l = enc28j60_read_reg(addr + 0);
	h = enc28j60_read_reg(addr + 1);
	if (l < 0 || h < 0) {
		return -1;
	}
	return (l | (h << 8));
}

/// \brief Write to a PHY register
/// The ENC28J60 PHY module's registers can only be accessed indirectly through
/// the MAC.  This function provides write functionality.
/// \param addr Address of PHY register to access.
/// \param val Value to write to the register
/// \return -1 on error, otherwise 0.
static int enc28j60_write_phy(uint8_t addr, uint16_t val)
{
	int status = 0;

	/// \todo Check for busy
	/// \bug Timeout?
	while (enc28j60_read_reg(MISTAT) & MISTAT_BUSY) {}

	status |= enc28j60_write_reg(MIREGADR, addr & 0x1f);
	status |= enc28j60_write_reg16(MIWRL, val);
	return status;
}

/// \brief Read from a PHY register
/// The ENC28J60 PHY module's registers can only be accessed indirectly through
/// the MAC.  This function provides read functionality.
/// \param addr Address of the PHY register to access.
/// \return The unsigned short value read from the register (cast to int), -1 on error.
static int enc28j60_read_phy(uint8_t addr)
{
	int status = 0;

	/// \todo Check for busy
	/// \bug Timeout?
	while (enc28j60_read_reg(MISTAT) & MISTAT_BUSY) {}

	// Set up the transfer.  Can't do bitfield ops on MAC or MII
	// registers so just do a write to set the MIIRD bit.
	status |= enc28j60_write_reg(MIREGADR, addr & 0x1f);
	status |= enc28j60_write_reg(MICMD, MICMD_MIIRD);

	/// \todo Poll busy bit??? Wait 10.24 us
	/// Doing it this way probably takes ages
	while (enc28j60_read_reg(MISTAT) & MISTAT_BUSY) {}

	// Stop reading
	status |= enc28j60_write_reg(MICMD, 0);
	if (status < 0) {
		return -1;
	}

	// Read the register contents
	return enc28j60_read_reg16(MIRDL);
}

/// \brief Transfer packet data to the NIC
/// Using DMA, transfer a buffer to the NIC.
/// \param buf Pointer to the buffer from which to transfer the data.
/// \param length Number of bytes to transfer.
/// \return -1 on error, otherwise 0.
static int enc28j60_write_buf(const uint8_t *buf, unsigned int length)
{
	spi_select();
	spi_write((const uint8_t[]){ ENC28J60_SPI_WBM }, 1);
	spi_write(buf, length);
	spi_deselect();
	return 0;
}

/// \brief Transfer packet data from the NIC
/// Using DMA, transfer a buffer from the NIC.
/// \param buf Pointer to the buffer in which to place received bytes.
/// \param length Number of bytes to transfer.
/// \return -1 on error, otherwise 0.
static int enc28j60_read_buf(uint8_t *buf, unsigned int length)
{
	spi_select();
	spi_write((const uint8_t[]){ ENC28J60_SPI_RBM }, 1);
	spi_read(buf, length);
	spi_deselect();
	return 0;
}

#if ENC28J60_DUMP_REGS
static void enc28j60_dump_regs(void)
{
	uint8_t reg;

	INFO("ECON1  = 0x%02X\n", enc28j60_read_reg(ECON1));

	// Dump registers
	for (reg = 0; reg < 0x80; reg++) {
		// MAC/MII
		unsigned char addr = reg;
		if ( ( (reg >= 0x40) && (reg <= 0x5a) ) || ( (reg >= 0x60) && (reg <= 0x65) ) || (reg == 0x6a) ) {
			addr |= ENC28J60_MACREG;
		}

		// Don't read 0x1A in any bank!
		if ((reg & 0x1f) != 0x1a) {
			INFO("R 0x%02X = 0x%02X\n", reg, enc28j60_read_reg(addr));
		}
	}

	// Dump PHY registers
	INFO("PHCON1  = 0x%04X\n", enc28j60_read_phy(PHCON1));
	INFO("PHSTAT1 = 0x%04X\n", enc28j60_read_phy(PHSTAT1));
	INFO("PHID1   = 0x%04X\n", enc28j60_read_phy(PHID1));
	INFO("PHID2   = 0x%04X\n", enc28j60_read_phy(PHID2));
	INFO("PHCON2  = 0x%04X\n", enc28j60_read_phy(PHCON2));
	INFO("PHSTAT2 = 0x%04X\n", enc28j60_read_phy(PHSTAT2));
	INFO("PHIE    = 0x%04X\n", enc28j60_read_phy(PHIE));
	INFO("PHIR    = 0x%04X\n", enc28j60_read_phy(PHIR));
	INFO("PHLCON  = 0x%04X\n", enc28j60_read_phy(PHLCON));
}
#endif

/*!
 * NOTE: Silicon errata 11 (receive hardware may corrupt the circular receive buffer
 * when an even value is programmed into ERXRDPTx
 */
static uint16_t enc28j60_rxrdpt_fix(uint16_t next_packet_ptr)
{
	uint16_t rxrdpt;

	if ((next_packet_ptr - 1) < RXSTART_INIT || (next_packet_ptr - 1) > RXSTOP_INIT) {
		rxrdpt = RXSTOP_INIT;
	} else {
		rxrdpt = next_packet_ptr - 1;
	}
	return rxrdpt;
}

/// Detect and initialise the Ethernet hardware.
int nic_init(void)
{
	TRACE("enc28j60_init()\n");

	// Perform a software reset
	spi_select();
	spi_write((const uint8_t[]){ ENC28J60_SPI_SRC }, 1);
	spi_deselect();
	timer_delay(TIMER_MILLIS(10));

	/* Default status */
	enc28j60_current_bank = 0;
	enc28j60_link_status = 0;

	// Initialise
	// Bank 0

	// Program buffer limits
	// ERXWRPT updated with value written to ERXST
	enc28j60_next_packet = RXSTART_INIT;
	enc28j60_write_reg16(ERXSTL, RXSTART_INIT);
	enc28j60_write_reg16(ERXNDL, RXSTOP_INIT);
	enc28j60_write_reg16(ERXRDPTL, RXSTART_INIT);
	enc28j60_write_reg16(ETXSTL, TXSTART_INIT);

	// Bank 2

	// Initialise MAC for half-duplex operation
	enc28j60_write_reg(MACON1, MACON1_MARXEN | MACON1_TXPAUS | MACON1_RXPAUS);
	// Some code seems to write 0 to this undocumented register
	enc28j60_write_reg(MACON2, 0x00);
#if ENC28J60_FULL_DUPLEX
	// Pad to 60 bytes and append CRC.  Set half-duplex mode
	enc28j60_write_reg(MACON3, MACON3_PADCFG0 | MACON3_TXCRCEN | MACON3_FRMLNEN | MACON3_FULDPX);
	// Back-to-back inter-packet gap.  0x12 for half-duplex, 0x15 for full-duplex
	enc28j60_write_reg(MABBIPG, 0x15);
#else
	// Pad to 60 bytes and append CRC.  Set half-duplex mode
	enc28j60_write_reg(MACON3, MACON3_PADCFG0 | MACON3_TXCRCEN | MACON3_FRMLNEN);
	// For half-duplex DEFER should be set for 802.3 compliance
	enc28j60_write_reg(MACON4, MACON4_DEFER);
	// Back-to-back inter-packet gap.  0x12 for half-duplex, 0x15 for full-duplex
	enc28j60_write_reg(MABBIPG, 0x12);
#endif
	// Non-back-to-back inter-packet gap
	enc28j60_write_reg16(MAIPGL, 0x0c12);
	// Set maximum frame length
	enc28j60_write_reg16(MAMXFLL, MAX_FRAMELEN);

	// Bank 3 (MAC address)
	enc28j60_write_reg(MAADR1, enc28j60_macaddr[0]);
	enc28j60_write_reg(MAADR2, enc28j60_macaddr[1]);
	enc28j60_write_reg(MAADR3, enc28j60_macaddr[2]);
	enc28j60_write_reg(MAADR4, enc28j60_macaddr[3]);
	enc28j60_write_reg(MAADR5, enc28j60_macaddr[4]);
	enc28j60_write_reg(MAADR6, enc28j60_macaddr[5]);

	// Set up LEDs - Bits 11-8 are the orange LED, bits 7-4 are the green one
	enc28j60_write_phy(PHLCON, ENC28J60_LED_MODE);
	// Configure PHY
#if ENC28J60_FULL_DUPLEX
	enc28j60_write_phy(PHCON1, PHCON1_PDPXMD);
	enc28j60_write_phy(PHCON2, 0);
#else
	enc28j60_write_phy(PHCON1, 0);
	enc28j60_write_phy(PHCON2, PHCON2_HDLDIS);
#endif

	// Enable receive and link status interrupts
	enc28j60_write_phy(PHIE, PHIE_PGEIE | PHIE_PLNKIE);
	enc28j60_clear_bits(EIR, EIR_DMAIF | EIR_LINKIF | EIR_TXIF | EIR_TXERIF | EIR_RXERIF | EIR_PKTIF);
	enc28j60_set_bits(EIE, EIE_INTIE /*| EIE_LINKIE */| EIE_PKTIE);

	// Enable packet reception
	enc28j60_set_bits(ECON2, ECON2_AUTOINC | ECON2_VRPS);
	enc28j60_set_bits(ECON1, ECON1_RXEN);

#if ENC28J60_DUMP_REGS
	enc28j60_dump_regs();
#endif

	return 0;
}

/// Returns number of packets available for reading
int nic_poll(void)
{
	// Poll for a packet
	return enc28j60_read_reg(EPKTCNT);
}

/// Receives next packet, if one is available
/// \param buf Pointer to a buffer in which to place the received data.
/// \param length Size of the buffer, in bytes.
/// \return Number of bytes actually read.  -1 on error (no packet available).
int nic_recv(uint8_t *buf, unsigned int length)
{
	int status = 0;
	enc28j60_rx_status_t rxstatus;

	if (nic_poll() <= 0) {
		/* No packet */
		return -1;
	}

	// Set the read pointer to where the packet should be
	status |= enc28j60_write_reg16(ERDPTL, enc28j60_next_packet);

	// Read the header so we can determine the packet length
	status |= enc28j60_read_buf((uint8_t*)&rxstatus, sizeof(rxstatus));

	// 68k is big endian!
	rxstatus.next_packet = SWAP16(rxstatus.next_packet);
	rxstatus.length = SWAP16(rxstatus.length) - 4; /* Also remove 4 byte CRC */
	rxstatus.status = SWAP16(rxstatus.status);

	if (rxstatus.status & ENC28J60_RXSTATUS_OK) {
		// Transfer the received packet into the buffer, truncating if the
		// buffer is too small
		if (rxstatus.length < length) {
			length = rxstatus.length;
		}
		status |= enc28j60_read_buf(buf, length);
	} else {
		// Packet is bad
		// FIXME: Error counters
		status = -1;
	}

	// Update next packet pointer
	enc28j60_next_packet = rxstatus.next_packet;

	// Update the receive pointer to free the memory taken by this packet
	status |= enc28j60_write_reg16(ERXRDPTL, enc28j60_rxrdpt_fix(enc28j60_next_packet));

	// Decrement EPKTCNT to acknowledge the packet
	status |= enc28j60_set_bits(ECON2, ECON2_PKTDEC);

#if ENC28J60_DEBUG_PACKETS
	{
		int n;
		fprintf(stderr, "RX %d bytes (status = %02X) %s\n", length, rxstatus.status, (status < 0) ? "ERROR" : "OK");
		for (n = 0; n < length; n++) {
			fprintf(stderr, "%02X ", buf[n]);
			if ((n & 15) == 15) {
				fprintf(stderr, "\n");
			}
		}
		fprintf(stderr, "\n");
	}
#endif

	return (status < 0) ? -1 : length;
}

/// Writes a packet to the transmit buffer and starts transmission.
/// The NIC will add the CRC to the end and pad if necessary, but the caller
/// must ensure that the MAC and type fields are filled in.
/// Packet is copied using DMA so the function returns as soon as transmission
/// starts, but will block if the hardware is busy on entry.
/// \param buf Pointer to a buffer from which to get data for transmission.
/// \param length Number of bytes to transmit.
/// \return Number of bytes actually written.  -1 on error.
int nic_send(const uint8_t *buf, unsigned int length)
{
	uint16_t txend;
	int status = 0;
	enc28j60_tx_status_t txstatus;
	unsigned char dummy = 0;

#if ENC28J60_DEBUG_PACKETS
	{
		int n;
		fprintf(stderr, "TX %d bytes\n", length);
		for (n = 0; n < length; n++) {
			fprintf(stderr, "%02X ", buf[n]);
			if ((n & 15) == 15) {
				fprintf(stderr, "\n");
			}
		}
		fprintf(stderr,"\n");
	}
#endif

	// Set write pointer to start of tx buffer area
	status |= enc28j60_write_reg16(EWRPTL, TXSTART_INIT);
	// Set TXND to point to the location at the end of the packet (this is
	// the offset to the transmit status register)
	txend = TXSTART_INIT + length;
	status |= enc28j60_write_reg16(ETXNDL, txend);

	// Write the control byte (always 0)
	status |= enc28j60_write_buf(&dummy, 1);

	// Write the packet data
	status |= enc28j60_write_buf(buf, length);

	// Start transmission
	status |= enc28j60_set_bits(ECON1, ECON1_TXRST); /* errata 10 */
	status |= enc28j60_clear_bits(ECON1, ECON1_TXRST);
	status |= enc28j60_clear_bits(EIR, EIR_TXIF | EIR_TXERIF);
	status |= enc28j60_set_bits(ECON1, ECON1_TXRTS);
	if (status < 0) {
		return -1;
	}

	// Wait for completion
	do {
		status = enc28j60_read_reg(EIR);
		if (status < 0) {
			return -1;
		}
	} while (!(status & (EIR_TXIF | EIR_TXERIF)));
	status = enc28j60_clear_bits(ECON1, ECON1_TXRTS);

	// Read the TSV and determine actual length sent
	status |= enc28j60_write_reg16(ERDPTL, txend + 1);
	status |= enc28j60_read_buf((uint8_t*)&txstatus, sizeof(txstatus));

	// 68k is big-endian
	txstatus.length = SWAP16(txstatus.length) - 4; /* Remove 4 byte CRC */
	txstatus.status1 = SWAP16(txstatus.status1);
	txstatus.bytes_on_wire = SWAP16(txstatus.bytes_on_wire);

	// FIXME: Check for late collision, implement errata 13 workaround

	return (status < 0) ? -1 : txstatus.length;
}

void nic_get_mac_address(uint8_t *buf)
{
	memcpy(buf, enc28j60_macaddr, sizeof(enc28j60_macaddr));
}

