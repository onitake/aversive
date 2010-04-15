/*
 *  Copyright Droids, Microb Technology (2010)
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
 *  Revision : $Id: strat.c,v 1.6 2009-11-08 17:24:33 zer0 Exp $
 *
 *  Olivier MATZ <zer0@droids-corp.org>
 */

#define CORN_NB 18

/* enum is better */
#define TYPE_WAYPOINT 0
#define TYPE_DANGEROUS 1
#define TYPE_WHITE_CORN 2
#define TYPE_BLACK_CORN 3
#define TYPE_OBSTACLE 4

extern uint8_t corn_table[CORN_NB];

int8_t ijcoord_to_corn_idx(int8_t i, int8_t j);
int8_t xycoord_to_corn_idx(int16_t *x, int16_t *y);
void init_corn_table(int8_t conf_side, int8_t conf_center);
