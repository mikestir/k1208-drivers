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

#include <exec/devices.h>
#include <exec/errors.h>
#include <exec/interrupts.h>

#include <dos/dos.h>
#include <dos/dostags.h>

#include <libraries/expansion.h>

#include <proto/exec.h>
#include <proto/expansion.h>
#include <proto/dos.h>

#include <hardware/custom.h>
#include <hardware/intbits.h>

#include "common.h"
#include "nic.h"
#include "spi.h"
#include "timer.h"

#define K1208_MFG		12345
#define K1208_PROD		0
#define K1208_IO_BASE	0xe90000

typedef struct {
	volatile uint32_t	*spi_base;
	struct Task			*task;
	int32_t				signal_mask;
} int_data_t;

extern void interrupt_handler(void);

static void dump_packet(const uint8_t *buf, unsigned int length)
{
	unsigned int n, m;
	const uint8_t *ptr;

	for (n = 0; n < length; n+=16) {
		ptr = &buf[n];
		for (m = 0; m < MIN(16, length - n); m++) {
			printf("%02x ", ptr[m]);
		}
		for ( ; m < 16; m++) {
			printf("   ");
		}
		for (m = 0; m < MIN(16, length - n); m++) {
			printf("%c", (ptr[m] > 31) ? ptr[m] : '.');
		}
		printf("\n");
	}
}

volatile uint32_t* find_board(void)
{
	struct ConfigDev *cd;
	volatile uint32_t *base = NULL;

	cd = NULL;
	while ((cd = FindConfigDev(cd, K1208_MFG, K1208_PROD)) != NULL) {
		if ((uint32_t)cd->cd_BoardAddr >= K1208_IO_BASE) {
			base = cd->cd_BoardAddr;
			break;
		}
	}

	return base;
}

int main(void)
{
	volatile uint32_t *board;
	int32_t signal;
	struct Interrupt *interrupt;
	int_data_t *int_data;

	/* Open libraries */
	SysBase = *(struct ExecBase**)4ul;
	ExpansionBase = (void*)OpenLibrary("expansion.library", 0);
	if (ExpansionBase == NULL) {
		ERROR("expansion.library not found\n");
		goto error;
	}
	DOSBase = (void*)OpenLibrary("dos.library", 37);
	if (DOSBase == NULL) {
		ERROR("dos.library not found\n");
		goto error;
	}

	/* Get base address of expansion board */
	board = find_board();
	if (board == NULL) {
		ERROR("Board not found\n");
		goto error;
	}
	INFO("Expansion board found at 0x%08x\n", (unsigned int)board);

	/* Allocate signal bit */
	signal = AllocSignal(-1);
	if (signal < 0) {
		ERROR("Failed to allocate signal bit\n");
		goto error;
	}
	INFO("Allocated signal %d\n", signal);
	interrupt = AllocMem(sizeof(struct Interrupt), MEMF_PUBLIC|MEMF_CLEAR);
	int_data = AllocMem(sizeof(int_data_t), MEMF_PUBLIC|MEMF_CLEAR);

	/* Register /INT2 handler */
	int_data->spi_base = (volatile uint32_t*)((uint32_t)board + 0x1000);  // offset to second device instance
	int_data->task = FindTask(NULL);
	int_data->signal_mask = 1ul << signal;
	interrupt->is_Data = int_data;
	interrupt->is_Code = interrupt_handler;
	AddIntServer(INTB_PORTS, interrupt);

	/* Initialise hardware */
	spi_init(int_data->spi_base);
	nic_init();

	while (1) {
		int len;
		ULONG sigs;
		static uint8_t rxbuf[1500];

		/* Wait for interrupt.  This can occur for events other than ethernet so we may
		 * not always receive a packet */
		spi_ext_int_enable();
		sigs = Wait(SIGBREAKF_CTRL_C | (1ul << signal));
		if (sigs & SIGBREAKF_CTRL_C) {
			break;
		}


		do {
			len = nic_recv(rxbuf, sizeof(rxbuf));
			if (len >= 0) {
				TRACE("RX %u bytes:\n", len);
				dump_packet(rxbuf, len);

				rxbuf[0] = rxbuf[6];
				rxbuf[1] = rxbuf[7];
				rxbuf[2] = rxbuf[8];
				rxbuf[3] = rxbuf[9];
				rxbuf[4] = rxbuf[10];
				rxbuf[5] = rxbuf[11];
				nic_get_mac_address(&rxbuf[6]);
				nic_send(rxbuf, len);
			}
		} while (len >= 0);
	}

	INFO("Cleaning up\n");
	RemIntServer(INTB_PORTS, interrupt);
	FreeMem(int_data, sizeof(int_data_t));
	FreeMem(interrupt, sizeof(struct Interrupt));
	FreeSignal(signal);

error:
	if (DOSBase) {
		CloseLibrary((struct Library*)DOSBase);
	}
	if (ExpansionBase) {
		CloseLibrary((struct Library*)ExpansionBase);
	}
	return 0;
}
