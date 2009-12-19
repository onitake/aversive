/*  
 *  Copyright Droids Corporation, Microb Technology, Eirbot (2005)
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
 *  Revision : $Id: kbd_config.h,v 1.2.4.1 2006-11-26 21:06:06 zer0 Exp $
 *
 */

#ifndef _KBD_CONFIG_H_
#define _KBD_CONFIG_H_ _SIMPLE

/* this config file is only for use with the kbd_simple module*/

//do not change for this line please
typedef uint8_t kbd_type;


#define KBD_FIFO_SIZE 4

#define KBD_COL1ROW1 '1'
#define KBD_COL1ROW2 '2'
#define KBD_COL1ROW3 '3'
#define KBD_COL1ROW4 '4'
#define KBD_COL2ROW1 '5'
#define KBD_COL2ROW2 '6'
#define KBD_COL2ROW3 '7'
#define KBD_COL2ROW4 '8'
#define KBD_COL3ROW1 '9'
#define KBD_COL3ROW2 '0'
#define KBD_COL3ROW3 '*'
#define KBD_COL3ROW4 '#'


#define KBD_PORT PORTA
#define KBD_DDR DDRA
#define KBD_PIN PINA

#define KBD_COL1_BIT 4
#define KBD_COL2_BIT 5
#define KBD_COL3_BIT 6

#define KBD_ROW1_BIT 0
/* Implicit, don't define it
#define KBD_ROW2_BIT (KBD_ROW1_BIT+1)
#define KBD_ROW3_BIT (KBD_ROW1_BIT+2)
#define KBD_ROW4_BIT (KBD_ROW1_BIT+3)
*/


#endif
