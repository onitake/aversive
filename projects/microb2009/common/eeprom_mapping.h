/*
 *  Copyright Droids Corporation (2007)
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
 *  Revision : $Id: eeprom_mapping.h,v 1.2 2009-05-27 20:04:06 zer0 Exp $
 *
 */

#ifndef _EEPROM_MAPPING_H_
#define _EEPROM_MAPPING_H_

#define EEPROM_MAGIC_MAINBOARD      1
#define EEPROM_MAGIC_MECHBOARD      2
#define EEPROM_MAGIC_SENSORBOARD    3

#define EEPROM_MAGIC_ADDRESS ((uint8_t *)0x100)

#define EEPROM_TIME_ADDRESS ((uint16_t *)0x110)

#endif
