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

#ifndef _STRAT_AVOID_H_
#define _STRAT_AVOID_H_
#define MAX_CIRCUIT_WPLINE 15

struct wp_coord {
	uint8_t i:4;
	uint8_t j:3;
	uint8_t reserved:1;
};

struct wp_line {
	uint8_t line_num:4;
	uint8_t dir:4;
};

int8_t browse_circuits(uint8_t i, uint8_t j,
		       const struct wp_coord **selected_circuit,
		       int8_t *selected_face);

/* harvest on the best circuit */
uint8_t strat_harvest_circuit(void);

/* get the neighbour of the point at specified dir, return -1 if
 * there is no neighbor */
int8_t wp_get_neigh(uint8_t i, uint8_t j, uint8_t *ni, uint8_t *nj,
		 uint8_t dir);

/* count the number of non-black corns which are neighbors of
 * specified cob */
uint8_t corn_count_neigh(uint8_t i, uint8_t j);

/* return true if a waypoint belongs to a line */
uint8_t wp_belongs_to_line(uint8_t i, uint8_t j, uint8_t linenum, uint8_t dir);

uint8_t is_60deg(uint8_t dir1, uint8_t dir2);
uint8_t is_120deg(uint8_t dir1, uint8_t dir2);

void test_strat_avoid(void);

#endif
