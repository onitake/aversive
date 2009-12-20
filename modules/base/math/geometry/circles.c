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

#include <math.h>

#include <aversive.h>

#include <vect_base.h>
#include <circles.h>

static inline float sq(float x)
{
	return x*x;
}

uint8_t pt_is_inside_circle(const point_t *p, circle_t *c)
{
	vect_t v;
	v.x = p->x - c->x;
	v.y = p->y - c->y;
	if ((v.x * v.x + v.y * v.y) < (c->r * c->r))
		return 1;
	return 0;
}

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
			 point_t *p1, point_t *p2)
{
	circle_t ca, cb;
	float a, b, c, d, e;
	uint8_t ret = 0;

	/* create circles with same radius, but centered on 0,0 : it
	 * will make process easier */
	ca.x = 0;
	ca.y = 0;
	ca.r = c1->r;
	cb.x = c2->x - c1->x;
	cb.y = c2->y - c1->y;
	cb.r = c2->r;

	/* inspired from
	   http://www.loria.fr/~roegel/notes/note0001.pdf */
	a = 2. * cb.x;
	b = 2. * cb.y;
	c = sq(cb.x) + sq(cb.y) - sq(cb.r) + sq(ca.r);
	d = sq(2. * a * c) -
		(4. * (sq(a) + sq(b)) * (sq(c) - sq(b) * sq(ca.r)) );

	/* no intersection */
	if (d < 0)
		return 0;

	if (b == 0) {
		/* special case */
		e = sq(cb.r) - sq((2. * c - sq(a)) / (2. * a));

		/* no intersection */
		if (e < 0)
			return 0;

		p1->x = (2. * a * c - sqrt(d)) / (2. * (sq(a) + sq(b)));
		p1->y = sqrt(e);
		p2->x = p1->x;
		p2->y = p1->y;
		ret = 1;
	}
	else {
		/* usual case */
		p1->x = (2. * a * c - sqrt(d)) / (2. * (sq(a) + sq(b)));
		p1->y = (c - a * p1->x) / b;
		p2->x = (2. * a * c + sqrt(d)) / (2. * (sq(a) + sq(b)));
		p2->y = (c - a * p2->x) / b;
		ret = 2;
	}

	/* retranslate */
	p1->x += c1->x;
	p1->y += c1->y;
	p2->x += c1->x;
	p2->y += c1->y;

	return ret;
}
