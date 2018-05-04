/*
 *  SPI SD device driver for K1208/Amiga 1200
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

#include <dos/dos.h>
#include <dos/dostags.h>

#include <libraries/expansion.h>

#include <devices/trackdisk.h>
#include <devices/scsidisk.h>

#include <proto/exec.h>
#include <proto/dos.h>
#include <proto/utility.h>
#include <proto/alib.h>
#include <proto/disk.h>
#include <proto/expansion.h>

#include <stabs.h>

#include "common.h"
#include "sd.h"
#include "spi.h"

#define K1208_MFG		12345
#define K1208_PROD		0
#define K1208_IO_BASE	0xe90000

#define SDSPI_TASK_NAME		"spisd"
#define SDSPI_TASK_PRIO		5

/* These must be globals and the variable names are important */
const char DevName[] = "spisd.device";
const char DevIdString[] = "spisd 0.2 (4 May 2018)";

const UWORD DevVersion = 0;
const UWORD DevRevision = 2;

typedef struct {
	volatile ULONG		*spi_base;
	struct Process		*handler_task;
	ULONG				signal_mask;
	ULONG				signal;

	struct Device		*device;
	struct Unit			unit;

	volatile struct List	read_list;
	volatile struct List	write_list;

	struct SignalSemaphore	read_list_sem;
	struct SignalSemaphore	write_list_sem;
} device_ctx_t;

/*! Global device context allocated on device init */
static device_ctx_t *ctx;
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

static uint32_t device_get_geometry(struct IOStdReq *iostd)
{
	struct DriveGeometry *geom = (struct DriveGeometry*)iostd->io_Data;
	const sd_card_info_t *ci = sd_get_card_info();

	if (ci->type != sdCardType_None) {
		geom->dg_SectorSize = 1 << ci->block_size;
		geom->dg_TotalSectors = ci->capacity >> ci->block_size;
		geom->dg_Cylinders = geom->dg_TotalSectors;
		geom->dg_CylSectors = 1;
		geom->dg_Heads = 1;
		geom->dg_TrackSectors = 1;
		geom->dg_BufMemType = MEMF_PUBLIC;
		geom->dg_DeviceType = DG_DIRECT_ACCESS;
		geom->dg_Flags = DGF_REMOVABLE;
		return 0;
	} else {
		return TDERR_DiskChanged;
	}
}

void __saveds device_task(void)
{
	while (1) {
		struct IOStdReq *iostd;
		ULONG sigs;

		/* Wait for signals from driver */
		sigs = Wait(ctx->signal_mask);
		if (sigs) {
			/* Handle read requests */
			do {
				ObtainSemaphore(&ctx->read_list_sem);
				iostd = (struct IOStdReq*)RemHead((struct List*)&ctx->read_list);
				ReleaseSemaphore(&ctx->read_list_sem);

				if (iostd) {
					if (sd_read(iostd->io_Data, iostd->io_Offset >> 9, iostd->io_Length >> 9) == 0) {
						iostd->io_Actual = iostd->io_Length;
						iostd->io_Error = 0;
					} else {
						iostd->io_Actual = 0;
						iostd->io_Error = TDERR_NotSpecified;
					}

					/* Reply to message */
					ReplyMsg(&iostd->io_Message);
				}
			} while (iostd);

			/* Handle write requests */
			do {
				ObtainSemaphore(&ctx->write_list_sem);
				iostd = (struct IOStdReq*)RemHead((struct List*)&ctx->write_list);
				ReleaseSemaphore(&ctx->write_list_sem);

				if (iostd) {
					if (sd_write(iostd->io_Data, iostd->io_Offset >> 9, iostd->io_Length >> 9) == 0) {
						iostd->io_Actual = iostd->io_Length;
						iostd->io_Error = 0;
					} else {
						iostd->io_Actual = 0;
						iostd->io_Error = TDERR_NotSpecified;
					}

					/* Reply to message */
					ReplyMsg(&iostd->io_Message);
				}
			} while (iostd);
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
	ctx = AllocMem(sizeof(device_ctx_t), MEMF_PUBLIC | MEMF_CLEAR);
	if (ctx == NULL) {
		ERROR("Memory allocation failed\n");
		goto error;
	}
	ctx->spi_base = (volatile ULONG*)((ULONG)board);
	ctx->device = device;

	/* Allocate signal bit */
	ctx->signal = AllocSignal(-1);
	if (ctx->signal < 0) {
		ERROR("Failed to allocate signal bit\n");
		goto error;
	}
	ctx->signal_mask = (1ul << ctx->signal);

	/* Initialise message lists and mutexes */
	NewList((struct List*)&ctx->read_list);
	NewList((struct List*)&ctx->write_list);
	InitSemaphore(&ctx->read_list_sem);
	InitSemaphore(&ctx->write_list_sem);

	/* Initialise hardware */
	spi_init(ctx->spi_base);

	/* Start receiver task */
	ctx->handler_task = CreateNewProcTags(NP_Entry, (ULONG)device_task, NP_Name, (ULONG)SDSPI_TASK_NAME, NP_Priority, SDSPI_TASK_PRIO, TAG_DONE);

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
		FreeSignal(ctx->signal);

		/* Free context memory */
		FreeMem(ctx, sizeof(device_ctx_t));
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

int __UserDevOpen(struct IORequest *ioreq, uint32_t unit, uint32_t flags)
{
	struct IOStdReq *iostd = (struct IOStdReq*)ioreq;
	int err = IOERR_OPENFAIL;

	if (iostd && unit == 0) {
		if (sd_open() == 0) {
			/* Device is open */
			iostd->io_Unit = &ctx->unit;
			ctx->unit.unit_flags = UNITF_ACTIVE;
			ctx->unit.unit_OpenCnt = 1;
			err = 0;
		}
	}

	iostd->io_Error = err;
	return err;
}

int __UserDevClose(struct IOExtTD *iotd)
{
	return 0;
}

ADDTABL_1(__BeginIO,a1);

void __BeginIO(struct IORequest *ioreq)
{
	struct IOStdReq *iostd = (struct IOStdReq*)ioreq;

	if (ctx == NULL || ioreq == NULL) {
		/* Driver not initialised */
		return;
	}

	iostd->io_Error = 0;

	/* Do IO */
	switch (iostd->io_Command) {
	case CMD_RESET:
	case CMD_CLEAR:
	case CMD_UPDATE:
	case TD_MOTOR:
	case TD_REMOVE:
		/* NULL commands */
		iostd->io_Actual = 0;
		break;
	case TD_PROTSTATUS:
		/* Should return a non-zero value if the card is write protected */
		iostd->io_Actual = 0;
		break;
	case TD_CHANGESTATE:
		/* Should return a non-zero value if the card is invalid or not inserted */
		iostd->io_Actual = 0;
		break;
	case TD_CHANGENUM:
		/* This should increment each time a disk is inserted */
		iostd->io_Actual = 1;
		break;
	case TD_GETDRIVETYPE:
		iostd->io_Actual = DG_DIRECT_ACCESS;
		break;
	case TD_GETGEOMETRY:
		iostd->io_Actual = 0;
		iostd->io_Error = device_get_geometry(iostd);
		break;
#if 0
	case HD_SCSI_CMD:
		break;
	case NSCMD_DEVICEQUERY:
		break;
	case NSCMD_TD_READ64:
		break;
	case NSCMD_WRITE64:
		break;
#endif

	case TD_FORMAT:
	case CMD_WRITE:
		/* Enqueue write request (defer reply to task) */
		ObtainSemaphore(&ctx->write_list_sem);
		AddTail((struct List*)&ctx->write_list, (struct Node*)iostd);
		ReleaseSemaphore(&ctx->write_list_sem);
		iostd->io_Flags &= ~IOF_QUICK;
		iostd = NULL;

		/* Signal handler task */
		Signal((struct Task*)ctx->handler_task, ctx->signal_mask);
		break;
	case CMD_READ:
		/* Enqueue read request (defer reply to task) */
		ObtainSemaphore(&ctx->read_list_sem);
		AddTail((struct List*)&ctx->read_list, (struct Node*)iostd);
		ReleaseSemaphore(&ctx->read_list_sem);
		iostd->io_Flags &= ~IOF_QUICK;
		iostd = NULL;

		/* Signal handler task */
		Signal((struct Task*)ctx->handler_task, ctx->signal_mask);
		break;
	default:
		iostd->io_Error = IOERR_NOCMD;
	}

	if (iostd) {
		/* Reply to message now unless it was deferred to the task */
		if (iostd->io_Flags & IOF_QUICK) {
			iostd->io_Message.mn_Node.ln_Type = NT_MESSAGE;
		} else {
			ReplyMsg(&iostd->io_Message);
		}
	}
}

ADDTABL_1(__AbortIO,a1);

void __AbortIO(struct IORequest *ioreq)
{
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
	ReplyMsg((struct Message*)ioreq);
}

ADDTABL_END();
