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

#include <exec/devices.h>
#include <exec/errors.h>
#include <exec/interrupts.h>

#include <dos/dos.h>
#include <dos/dostags.h>

#include <libraries/expansion.h>

#include <proto/exec.h>
#include <proto/expansion.h>
#include <proto/dos.h>
#include <proto/utility.h>
#include <proto/alib.h>

#include <hardware/custom.h>
#include <hardware/intbits.h>

#include <stabs.h>

#include "common.h"
#include "nic.h"
#include "spi.h"
#include "timer.h"
#include "sana2.h"

#define K1208_MFG		12345
#define K1208_PROD		0
#define K1208_IO_BASE	0xe90000

#define ETHERSPI_TASK_NAME		"spinet"
#define ETHERSPI_TASK_PRIO		10

/* These must be globals and the variable names are important (used by libnix device wrapper) */
const char DevName[] = "spinet.device";
const char DevIdString[] = "spinet 0.2 (4 May 2018)";

const UWORD DevVersion = 0;
const UWORD DevRevision = 2;

extern void interrupt_handler(void);

typedef BOOL (*etherspi_bmfunc_t)(register APTR dst __asm("a0"), register APTR src __asm("a1"), register LONG size __asm("d0"));

typedef struct {
	struct MinNode		node;
	etherspi_bmfunc_t	copyfrom;
	etherspi_bmfunc_t	copyto;
} etherspi_buffer_funcs_t;

typedef struct {
	/* Order is important here - some of these fields are used from interrupt.s */
	volatile ULONG		*spi_base;
	struct Process		*handler_task;
	ULONG				rx_signal_mask;
	ULONG				tx_signal_mask;
	ULONG				rx_signal;
	ULONG				tx_signal;

	struct Device		*device;
	struct Interrupt	interrupt;

	volatile struct List	read_list;
	volatile struct List	write_list;

	struct SignalSemaphore	read_list_sem;
	struct SignalSemaphore	write_list_sem;

	etherspi_buffer_funcs_t	*bf;
	unsigned char		frame[NIC_MTU + sizeof(nic_eth_hdr_t)];
} etherspi_ctx_t;


/*! Global device context allocated on device init */
static etherspi_ctx_t *ctx;
struct ExecBase *SysBase;

static volatile ULONG* device_find_board(void)
{
	struct ConfigDev *cd;
	volatile ULONG *base = NULL;

	cd = NULL;
	while ((cd = FindConfigDev(cd, K1208_MFG, K1208_PROD)) != NULL) {
		if ((ULONG)cd->cd_BoardAddr >= K1208_IO_BASE) {
			base = cd->cd_BoardAddr;
			break;
		}
	}

	return base;
}

static BOOL device_node_is_in_list(struct Node *node, struct List *list)
{
	struct Node *n;

	for (n = list->lh_Head; n; n = n->ln_Succ) {
		if (n == node) {
			return TRUE;
		}
	}
	return FALSE;
}

static void device_query(struct IOSana2Req *req)
{
	struct Sana2DeviceQuery *query;

	/* Answer device query message */
	query = req->ios2_StatData;
	query->DevQueryFormat = 0;
	query->DeviceLevel = 0;

	/* Some fields are not always present */
	if (query->SizeAvailable >= 18) {
		query->AddrFieldSize = NIC_MACADDR_SIZE * 8;
	}
	if (query->SizeAvailable >= 22) {
		query->MTU = NIC_MTU;
	}
	if (query->SizeAvailable >= 26) {
		query->BPS = NIC_BPS;
	}
	if (query->SizeAvailable >= 30) {
		query->HardwareType = S2WireType_Ethernet;
	}

	query->SizeSupplied = MIN(query->SizeAvailable, 30);
}

void __saveds device_task(void)
{
	while (1) {
		struct IOSana2Req *ios2;
		struct IORequest *ioreq;
		nic_eth_hdr_t *hdr = (nic_eth_hdr_t*)ctx->frame;
		etherspi_buffer_funcs_t *bf;
		int len;
		ULONG sigs;

		/* Enable external interrupt (ISR disables before raising signal to avoid a deadlock) */
		spi_ext_int_enable();

		/* Wait for signals from driver and ISR */
		sigs = Wait(ctx->rx_signal_mask | ctx->tx_signal_mask);
		if (sigs & ctx->tx_signal_mask) {
			/* Send packets from write queue */
			do {
				ObtainSemaphore(&ctx->write_list_sem);
				ios2 = (struct IOSana2Req*)RemHead((struct List*)&ctx->write_list);
				ioreq = (struct IORequest*)ios2;
				ReleaseSemaphore(&ctx->write_list_sem);

				if (ios2) {
					/* Assemble packet in buffer */
					bf = (etherspi_buffer_funcs_t*)ios2->ios2_BufferManagement;

					if (ioreq->io_Flags & SANA2IOF_RAW) {
						/* Verbatim */
						bf->copyfrom(ctx->frame, ios2->ios2_Data, ios2->ios2_DataLength);
					} else {
						/* Build header */
						hdr->type = ios2->ios2_PacketType;
						nic_get_mac_address(hdr->src);
						memcpy(hdr->dest, ios2->ios2_DstAddr, NIC_MACADDR_SIZE);
						bf->copyfrom(&hdr[1], ios2->ios2_Data, ios2->ios2_DataLength);
					}

					/* Write */
					Forbid();
					nic_send(ctx->frame, ios2->ios2_DataLength + ((ioreq->io_Flags & SANA2IOF_RAW) ? 0 : sizeof(nic_eth_hdr_t)));
					Permit();

					/* Reply to message */
					ioreq->io_Error = 0;
					ReplyMsg(&ioreq->io_Message);
				}
			} while (ios2);
		}

		if (sigs & ctx->rx_signal_mask) {
			do {
				Forbid();
				len = nic_recv(ctx->frame, NIC_MTU);
				Permit();

				if (len >= 0) {
					/* Packet received - search read list for read request of correct type */
					ObtainSemaphore(&ctx->read_list_sem);
					for (ios2 = (struct IOSana2Req*)ctx->read_list.lh_Head; ios2; ios2 = (struct IOSana2Req*)ios2->ios2_Req.io_Message.mn_Node.ln_Succ) {
						if (ios2->ios2_PacketType == hdr->type) {
							Remove((struct Node*)ios2);
							break;
						}
					}
					ReleaseSemaphore(&ctx->read_list_sem);
					ioreq = (struct IORequest*)ios2;

					if (ios2) {
						int n;

						/* Copy packet to io request buffer */
						bf = (etherspi_buffer_funcs_t*)ios2->ios2_BufferManagement;
						if (ioreq->io_Flags & SANA2IOF_RAW) {
							/* Verbatim */
							ios2->ios2_DataLength = len;
							bf->copyto(ios2->ios2_Data, ctx->frame, ios2->ios2_DataLength);
							ioreq->io_Flags = SANA2IOF_RAW;
						} else {
							/* Skip header */
							ios2->ios2_DataLength = len - sizeof(nic_eth_hdr_t);
							bf->copyto(ios2->ios2_Data, &hdr[1], ios2->ios2_DataLength);
							ioreq->io_Flags = 0;
						}

						/* Extract ethernet header data */
						memcpy(ios2->ios2_SrcAddr, hdr->src, NIC_MACADDR_SIZE);
						memcpy(ios2->ios2_DstAddr, hdr->dest, NIC_MACADDR_SIZE);
						ioreq->io_Flags |= SANA2IOF_BCAST;
						for (n = 0; n < NIC_MACADDR_SIZE; n++) {
							if (hdr->dest[n] != 0xff) {
								ioreq->io_Flags &= ~SANA2IOF_BCAST;
								break;
							}
						}
						ios2->ios2_PacketType = hdr->type;
						ioreq->io_Error = 0;

						ReplyMsg(&ioreq->io_Message);
					} else {
						/* FIXME: No matching read request - place packet in orphan queue */
					}
				}
			} while (len >= 0);
		}
	}
}

int __UserDevInit(struct Device *device)
{
	volatile ULONG *board;

	/* Open libraries */
	SysBase = *(struct ExecBase**)4l;
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
	UtilityBase = (void*)OpenLibrary("utility.library", 37);
	if (UtilityBase == NULL) {
		ERROR("utility.library not found\n");
		goto error;
	}

	/* Look for board */
	board = device_find_board();
	if (board == NULL) {
		ERROR("Expansion board not found\n");
		goto error;
	}
	INFO("Expansion board found at 0x%08x\n", (unsigned int)board);

	/* Allocate driver context */
	ctx = AllocMem(sizeof(etherspi_ctx_t), MEMF_PUBLIC | MEMF_CLEAR);
	if (ctx == NULL) {
		ERROR("Memory allocation failed\n");
		goto error;
	}
	ctx->spi_base = (volatile ULONG*)((ULONG)board + 0x1000);
	ctx->device = device;

	/* Allocate signal bit */
	ctx->rx_signal = AllocSignal(-1);
	if (ctx->rx_signal < 0) {
		ERROR("Failed to allocate signal bit\n");
		goto error;
	}
	ctx->tx_signal = AllocSignal(-1);
	if (ctx->tx_signal < 0) {
		ERROR("Failed to allocate signal bit\n");
		goto error;
	}
	ctx->rx_signal_mask = (1ul << ctx->rx_signal);
	ctx->tx_signal_mask = (1ul << ctx->tx_signal);

	/* Initialise message lists and mutexes */
	NewList((struct List*)&ctx->read_list);
	NewList((struct List*)&ctx->write_list);
	InitSemaphore(&ctx->read_list_sem);
	InitSemaphore(&ctx->write_list_sem);

	/* Initialise hardware (offset to second SPI port) */
	spi_init(ctx->spi_base);
	nic_init();

	/* Start receiver task */
	ctx->handler_task = CreateNewProcTags(NP_Entry, (ULONG)device_task, NP_Name, (ULONG)ETHERSPI_TASK_NAME, NP_Priority, ETHERSPI_TASK_PRIO, TAG_DONE);

	/* Register /INT2 handler */
	ctx->interrupt.is_Data = ctx;
	ctx->interrupt.is_Code = interrupt_handler;
	AddIntServer(INTB_PORTS, &ctx->interrupt);

	/* Return success */
	return 1;

error:
	/* Clean up after failed open */
	if (UtilityBase) {
		CloseLibrary((struct Library*)UtilityBase);
	}
	if (DOSBase) {
		CloseLibrary((struct Library*)DOSBase);
	}
	if (ExpansionBase) {
		CloseLibrary((struct Library*)ExpansionBase);
	}
	return 0;
}

void __UserDevCleanup(void)
{
	if (ctx) {
		RemIntServer(INTB_PORTS, &ctx->interrupt);
		FreeSignal(ctx->rx_signal);
		FreeSignal(ctx->tx_signal);

		/* Free context memory */
		FreeMem(ctx, sizeof(etherspi_ctx_t));
		ctx = NULL;
	}

	/* Clean up libs */
	if (UtilityBase) {
		CloseLibrary((struct Library*)UtilityBase);
	}
	if (DOSBase) {
		CloseLibrary((struct Library*)DOSBase);
	}
	if (ExpansionBase) {
		CloseLibrary((struct Library*)ExpansionBase);
	}
}

int __UserDevOpen(struct IORequest *ioreq, ULONG unit, ULONG flags)
{
	struct IOSana2Req *ios2 = (struct IOSana2Req*)ioreq;
	etherspi_buffer_funcs_t *bf;
	int err = IOERR_OPENFAIL;

	/* Only unit 0 supported */
	/* FIXME: This doesn't allow for multiple opens */
	if (unit == 0) {
		/* Allocate buffer function pointers */
		bf = AllocVec(sizeof(etherspi_buffer_funcs_t), MEMF_CLEAR | MEMF_PUBLIC);
		if (bf) {
			bf->copyfrom = (etherspi_bmfunc_t)GetTagData(S2_CopyFromBuff, NULL, (struct TagItem*)ios2->ios2_BufferManagement);
			bf->copyto = (etherspi_bmfunc_t)GetTagData(S2_CopyToBuff, NULL, (struct TagItem*)ios2->ios2_BufferManagement);
			ctx->bf = bf;

			/* Success */
			ios2->ios2_BufferManagement = bf;
			ioreq->io_Error = 0;
			ioreq->io_Unit = (struct Unit*)unit;
			ioreq->io_Device = ctx->device;
			err = 0;
		}
	}

	if (err) {
		ioreq->io_Error = err;
		ioreq->io_Unit = NULL;
		ioreq->io_Device = NULL;
	}
	ioreq->io_Message.mn_Node.ln_Type = NT_REPLYMSG;

	return err;
}

int __UserDevClose(struct IORequest *ioreq)
{
	ioreq->io_Unit = (struct Unit*)-1;
	ioreq->io_Device = (struct Device*)-1;

	if (ctx->bf) {
		FreeVec(ctx->bf);
	}
	ctx->bf = NULL;
	return 0;
}

ADDTABL_1(__BeginIO,a1);

void __BeginIO(struct IORequest *ioreq)
{
	struct IOSana2Req *ios2 = (struct IOSana2Req*)ioreq;

	if (ctx == NULL || ioreq == NULL) {
		/* Driver not initialised */
		return;
	}

	ioreq->io_Error = S2ERR_NO_ERROR;
	ios2->ios2_WireError = S2WERR_GENERIC_ERROR;

	TRACE("ioreq=%lu\n", ioreq->io_Command);

	/* Do IO */
	switch (ioreq->io_Command) {
	case CMD_READ:
		if (ios2->ios2_BufferManagement == NULL) {
			ioreq->io_Error = S2ERR_BAD_ARGUMENT;
			ios2->ios2_WireError = S2WERR_BUFF_ERROR;
			break;
		}

		/* Enqueue read buffer (defer reply to task) */
		ObtainSemaphore(&ctx->read_list_sem);
		AddTail((struct List*)&ctx->read_list, (struct Node*)ios2);
		ReleaseSemaphore(&ctx->read_list_sem);
		ioreq->io_Flags &= ~SANA2IOF_QUICK;
		ios2 = NULL;
		break;

	case S2_BROADCAST:
		/* Update destination address for broadcast */
		memset(ios2->ios2_DstAddr, 0xff, NIC_MACADDR_SIZE);
		/* Fall through */
	case CMD_WRITE:
		if (ios2->ios2_DataLength > NIC_MTU) {
			ioreq->io_Error = S2ERR_MTU_EXCEEDED;
			break;
		}
		if (ios2->ios2_BufferManagement == NULL) {
			ioreq->io_Error = S2ERR_BAD_ARGUMENT;
			ios2->ios2_WireError = S2WERR_BUFF_ERROR;
			break;
		}

		/* Enqueue write buffer (defer reply to task) */
		ObtainSemaphore(&ctx->write_list_sem);
		AddTail((struct List*)&ctx->write_list, (struct Node*)ios2);
		ReleaseSemaphore(&ctx->write_list_sem);
		ioreq->io_Flags &= ~SANA2IOF_QUICK;
		ios2 = NULL;

		/* Signal handler task */
		Signal((struct Task*)ctx->handler_task, ctx->tx_signal_mask);
		break;

	case S2_ONLINE:
	case S2_OFFLINE:
	case S2_CONFIGINTERFACE:
		break;
	case S2_GETSTATIONADDRESS:
		nic_get_mac_address(ios2->ios2_SrcAddr);
		nic_get_mac_address(ios2->ios2_DstAddr);
		break;
	case S2_DEVICEQUERY:
		device_query(ios2);
		break;

	case S2_ONEVENT:
	case S2_TRACKTYPE:
	case S2_UNTRACKTYPE:
	case S2_GETTYPESTATS:
	case S2_READORPHAN:
	case S2_GETGLOBALSTATS:
	case S2_GETSPECIALSTATS:
		break;

	/* All other commands are treated as unsupported SANA2 commands */
	default:
		ioreq->io_Error = IOERR_NOCMD;
		ios2->ios2_WireError = S2WERR_GENERIC_ERROR;
	}

	if (ios2) {
		/* If request wasn't deferred to a task then reply now */
		if (ioreq->io_Flags & SANA2IOF_QUICK) {
			ioreq->io_Message.mn_Node.ln_Type = NT_MESSAGE;
		} else {
			ReplyMsg(&ioreq->io_Message);
		}
	}
}

ADDTABL_1(__AbortIO,a1);

void __AbortIO(struct IORequest *ioreq)
{
	struct IOSana2Req *ios2 = (struct IOSana2Req *)ioreq;

	if (ioreq == NULL) {
		return;
	}

	/* Remove this IO request from any lists to which it is attached */
	ObtainSemaphore(&ctx->read_list_sem);
	if (device_node_is_in_list((struct Node*)ioreq, (struct List*)&ctx->read_list)) {
		Remove((struct Node*)ioreq);
	}
	ReleaseSemaphore(&ctx->read_list_sem);

	ObtainSemaphore(&ctx->write_list_sem);
	if (device_node_is_in_list((struct Node*)ioreq, (struct List*)&ctx->write_list)) {
		Remove((struct Node*)ioreq);
	}
	ReleaseSemaphore(&ctx->write_list_sem);

	/* Clean up */
	ioreq->io_Error = IOERR_ABORTED;
	ios2->ios2_WireError = 0;
	ReplyMsg((struct Message*)ioreq);
}

ADDTABL_END();



