/*  
 *  Copyright Droids Corporation (2008)
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
 *  Revision : $Id: main.h,v 1.1 2009-01-30 20:42:17 zer0 Exp $
 *
 */

// XXX values are not correct for this board
#define LED1_ON() 	sbi(PORTE, 2)
#define LED1_OFF() 	cbi(PORTE, 2)

#define LED2_ON() 	sbi(PORTE, 3)
#define LED2_OFF() 	cbi(PORTE, 3)

#define LED_PRIO           170
#define TIME_PRIO          160
#define CS_PRIO            150

