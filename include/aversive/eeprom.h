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
 *  Revision : $Id: eeprom.h,v 1.1.2.4 2007-11-21 21:54:38 zer0 Exp $
 *
 */

/**
 * This file is used for compatibility between host and avr : with
 * this we can emulate eeprom on a host.
 */

#ifndef _AVERSIVE_EEPROM_H_
#define _AVERSIVE_EEPROM_H_

#ifndef HOST_VERSION

#include <avr/eeprom.h>

#else

/* XXX */

#endif /* HOST_VERSION */
#endif /* _AVERSIVE_EEPROM_H_ */


