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

#define LINE_UP     0
#define LINE_L_UP   1
#define LINE_L_DOWN 2
#define LINE_DOWN   3
#define LINE_R_DOWN 4
#define LINE_R_UP   5

struct line_2pts {
	point_t p1;
	point_t p2;
};

/* there is a corn near */
int8_t corn_is_near(uint8_t *corn_idx, uint8_t side,
		    int16_t *xspickle, int16_t *yspickle);

/* go from line num1,dir1 to line num2,dir2. Uses trjectory flags
 * specified as argument and return END_xxx condition */
uint8_t line2line(uint8_t num1, uint8_t dir1, uint8_t num2,
		  uint8_t dir2, uint8_t flags);
