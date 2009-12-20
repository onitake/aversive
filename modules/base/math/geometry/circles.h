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

#ifndef _CIRCLES_H_
#define _CIRCLES_H_

typedef struct _circle {
	double x;
	double y;
	double r;
} circle_t;

/* return true if a point is in the disc */
uint8_t pt_is_inside_circle(const point_t *p, circle_t *c);

/*
 * return values:
 *  0 dont cross
 *  1 one intersection point
 *  2 two intersection points
 *
 *  p1, p2 arguments are the crossing points coordinates. Both p1 and
 *  p2 are dummy for 0 result. When result is 1, p1 and p2 are set to
 *  the same value.
 */
uint8_t circle_intersect(const circle_t *c1, const circle_t *c2,
			 point_t *p1, point_t *p2);

#endif /* _CIRCLES_H_ */
