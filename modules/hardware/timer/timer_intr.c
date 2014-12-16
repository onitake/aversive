/*  
 *  Copyright Droids Corporation, Microb Technology, Eirbot (2006)
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
 *  Revision : $Id: timer_intr.c,v 1.1.2.4 2009-01-30 20:18:36 zer0 Exp $
 *
 */

#include <stdint.h>
#include <string.h>

#include <aversive/timers.h>

#include <timer.h>
#include <timer_definitions.h>
#include <timer_intr.h>

#include <timer_config.h>

volatile timer_callback_t timer_callback_table[_VECTORS_SIZE/4];

/*************************/

#if defined TIMER0_ENABLED && defined TIMER0_OVF_vect
DEFINE_OV_INTR(TIMER0_OVF_vect)
#endif

#if defined TIMER0_ENABLED && defined TIMER0_COMP_vect
DEFINE_OC_INTR(TIMER0_COMP_vect)
#endif

#if defined TIMER0_ENABLED && defined TIMER0_COMPA_vect
DEFINE_OC_INTR(TIMER0_COMPA_vect)
#endif

#if defined TIMER0_ENABLED && defined TIMER0_COMPB_vect
DEFINE_OC_INTR(TIMER0_COMPB_vect)
#endif

/*************************/

#if defined TIMER1_ENABLED && defined TIMER1_OVF_vect
DEFINE_OV_INTR(TIMER1_OVF_vect)
#endif

#if defined TIMER1A_ENABLED && defined TIMER1_COMPA_vect
DEFINE_OC_INTR(TIMER1_COMPA_vect)
#endif

#if defined TIMER1B_ENABLED && defined TIMER1_COMPB_vect
DEFINE_OC_INTR(TIMER1_COMPB_vect)
#endif

#if defined TIMER1C_ENABLED && defined TIMER1_COMPC_vect
DEFINE_OC_INTR(TIMER1_COMPC_vect)
#endif

/*************************/

#if defined TIMER2_ENABLED && defined TIMER2_OVF_vect
DEFINE_OV_INTR(TIMER2_OVF_vect)
#endif

#if defined TIMER2_ENABLED && defined TIMER2_COMP_vect
DEFINE_OC_INTR(TIMER2_COMP_vect)
#endif

#if defined TIMER2_ENABLED && defined TIMER2_COMPA_vect
DEFINE_OC_INTR(TIMER2_COMPA_vect)
#endif

#if defined TIMER2_ENABLED && defined TIMER2_COMPB_vect
DEFINE_OC_INTR(TIMER2_COMPB_vect)
#endif

/*************************/

#if defined TIMER3_ENABLED && defined TIMER3_OVF_vect
DEFINE_OV_INTR(TIMER3_OVF_vect)
#endif

#if defined TIMER3A_ENABLED && defined TIMER3_COMPA_vect
DEFINE_OC_INTR(TIMER3_COMPA_vect)
#endif

#if defined TIMER3B_ENABLED && defined TIMER3_COMPB_vect
DEFINE_OC_INTR(TIMER3_COMPB_vect)
#endif

#if defined TIMER3C_ENABLED && defined TIMER3_COMPC_vect
DEFINE_OC_INTR(TIMER3_COMPC_vect)
#endif

/*************************/

#if defined TIMER4_ENABLED && defined TIMER4_OVF_vect
DEFINE_OV_INTR(TIMER4_OVF_vect)
#endif

#if defined TIMER4A_ENABLED && defined TIMER4_COMPA_vect
DEFINE_OC_INTR(TIMER4_COMPA_vect)
#endif

#if defined TIMER4B_ENABLED && defined TIMER4_COMPB_vect
DEFINE_OC_INTR(TIMER4_COMPB_vect)
#endif

#if defined TIMER4C_ENABLED && defined TIMER4_COMPC_vect
DEFINE_OC_INTR(TIMER4_COMPC_vect)
#endif

/*************************/

#if defined TIMER5_ENABLED && defined TIMER5_OVF_vect
DEFINE_OV_INTR(TIMER5_OVF_vect)
#endif

#if defined TIMER5A_ENABLED && defined TIMER5_COMPA_vect
DEFINE_OC_INTR(TIMER5_COMPA_vect)
#endif

#if defined TIMER5B_ENABLED && defined TIMER5_COMPB_vect
DEFINE_OC_INTR(TIMER5_COMPB_vect)
#endif

#if defined TIMER5C_ENABLED && defined TIMER5_COMPC_vect
DEFINE_OC_INTR(TIMER5_COMPC_vect)
#endif

/*************************/

void timer_intr_init(void)
{
	memset((void*)timer_callback_table, 0, sizeof(timer_callback_table));
}
