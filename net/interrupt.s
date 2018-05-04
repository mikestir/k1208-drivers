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

	.global		_interrupt_handler

/*
 * Register usage for interrupt servers:
 *
 *  D0 == scratch
 *  D1 == scratch
 *  A0 == scratch
 *  A1 == is_Data which is RBFDATA structure (scratch)
 *  A5 == vector to our code (scratch)
 *  A6 == scratch
 *
 * Data is expected to contain the following data items in this order:
 *
 * 0 - base address of SPI module (4)
 * 4 - pointer to task structure for signal (4)
 * 8 - signal mask to send (4)
 */

_interrupt_handler:
	move.l		4,a6				/* SysBase */
	move.l		0(a1),a0			/* Pointer to SPI device */
	move.l		8(a1),d0			/* Signal number from data struct */
	move.l		4(a1),a1			/* Task handle */
	bclr.b		#6,0(a0)			/* Mask external interrupt */
	jsr			-0x144(a6)			/* Exec "Signal" */
	moveq		#0,d0				/* Set Z flag to indicate IRQ should be propagated further */
	rts
