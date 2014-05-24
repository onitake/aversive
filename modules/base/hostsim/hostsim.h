/*
 *  Copyright Droids Corporation
 *  Olivier Matz <zer0@droids-corp.org>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *  Revision : $Id: main.c,v 1.10 2009-11-08 17:24:33 zer0 Exp $
 *
 */

#include <stdint.h>

/* initialize hostsim uart framework */
int hostsim_uart_init(void);

/* exit hostsim framework (should be called when program exits) */
int hostsim_uart_exit(void);

/* replacement for wait_ms() */
void host_wait_ms(int ms);

/* allow irq */
void hostsim_sei(void);

/* lock irq */
void hostsim_cli(void);

/* lock emulated IRQ and return the previous state */
uint8_t hostsim_irq_save(void);

/* restore the state given as parameter  */
void hostsim_irq_restore(uint8_t flags);

/* return 1 if emulated IRQ are locked */
uint8_t hostsim_irq_locked(void);

/* Add a new timer: loaded at init and cannot be unloaded. The resolution is
 * specified later in hostsim_ittimer_enable(). If a value lower than the
 * resolution is given, the timer handler will be called several times from the
 * signal handler.  However it's not advised as some callbacks can be lost the
 * signal occurs when irq are locked.
 *
 * This function must be called before hostsim_ittimer_enable(). Once
 * hostsim_ittimer_enable() is called, no timer should be added. */
struct hostsim_ittimer *hostsim_ittimer_add(void (*handler)(void),
	unsigned period_ns);

/* enable loaded ittimers
 * 'timer_resolution_us' is the resolution of timer events that can be
 * loaded. The advised value is 100 (us). */
int hostsim_ittimer_enable(unsigned timer_resolution_us);
