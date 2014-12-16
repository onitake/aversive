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
 *  Revision : $Id: timer4_register_OC_at_tics.c,v 1.1.2.1 2009-01-30 20:18:36 zer0 Exp $
 *
 */

#include <stdint.h>
#include <string.h>

#include <aversive/timers.h>

#include <timer.h>
#include <timer_definitions.h>
#include <timer_intr.h>
#include <timer_config.h>


#if defined TIMER4A_ENABLED && defined TIMER4_COMPA_vect
DEFINE_REGISTER_OC_INTR_AT_TICS(4A, TIMER4_COMPA_vect)
#endif

#if defined TIMER4B_ENABLED && defined TIMER4_COMPB_vect
DEFINE_REGISTER_OC_INTR_AT_TICS(4B, TIMER4_COMPB_vect)
#endif

#if defined TIMER4C_ENABLED && defined TIMER4_COMPC_vect
DEFINE_REGISTER_OC_INTR_AT_TICS(4C, TIMER4_COMPC_vect)
#endif

