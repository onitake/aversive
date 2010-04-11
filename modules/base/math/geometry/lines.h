/*
 *  Copyright Droids Corporation (2009)
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
 *  Revision : $Id: f16.h,v 1.6.4.3 2008-05-10 15:06:26 zer0 Exp $
 *
 */

#ifndef _LINES_H_
#define _LINES_H_

typedef struct _line {
	double a;
	double b;
	double c;
} line_t;


void
pts2line(const point_t *p1, const point_t *p2, line_t *l);

void
proj_pt_line(const point_t *p, const line_t *l, point_t *p_out);

/*
 * return values:
 *  0 dont cross
 *  1 cross
 *  2 "parallel crossing"
 *
 *  p argument is the crossing point coordinates (dummy for 0 or 2
 *  result)
 */
uint8_t
intersect_line(const line_t *l1, const line_t *l2, point_t *p);

uint8_t
intersect_segment(const point_t *s1, const point_t *s2,
		  const point_t *t1, const point_t *t2,
		  point_t *p);

/* translate the line */
void
line_translate(line_t *l, vect_t *v);
#endif /* _LINES_H_ */
