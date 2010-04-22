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
#define LINE_DOWN   1
#define LINE_R_UP   2
#define LINE_L_DOWN 3
#define LINE_L_UP   4
#define LINE_R_DOWN 5

struct line_2pts {
	point_t p1;
	point_t p2;
};

void num2line(struct line_2pts *l, uint8_t dir, uint8_t num);

uint8_t line2line(uint8_t dir1, uint8_t num1,
		  uint8_t dir2, uint8_t num2);
