/*
 *  Copyright Droids Corporation (2010)
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
 *  Olivier MATZ <zer0@droids-corp.org>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <aversive.h>

#include <vect_base.h>
#include <lines.h>
#include <circles.h>

#define POS_ACCURACY 10.0 /* 1 cm accuracy max */
#ifndef HOST_VERSION
#define printf(args...) do {} while(0)
#endif

static int dprint = 0;
#define dprintf(args...) if (dprint) printf(args)

const point_t beacon0 = { 0, 1050 };
const point_t beacon1 = { 3000, 0 };
const point_t beacon2 = { 3000, 2100 };

/* Fill the 2 circles pointer given as parameter, each of those cross
 * both beacons b1 and b2. From any point of these circles (except b1
 * and b2), we see b1 and b2 with the angle of a_rad (which must be
 * positive). Return 0 on success.
 *
 *                              l
 *                <------------------------->
 *
 *              b1              O            b2
 *               +----------------------------+
 *             ,' \\            |            /|'\
 *            /    \ \          | ^        / |   `
 *           /       \ \   a___ | | d    /   |    `.
 *          /         \  \ /    | v    /     |     \
 *         |            \  \    |    /       |      |
 *         |             \   \  |  /        |       |
 *         |               \   \|/          |        |
 *         |                \   * C         |        |
 *         |                  \             |       .'
 *         |                   \           |        |
 *          |                    \         |       .'
 *           \                    \   a____|       /
 *            \                     \ /    |     ,'
 *             `                     \    |     /
 *              '.                     \  |   ,'
 *                '-.                   \ |_,'
 *                   '-._              _,*'
 *                       '`--......---'     R (the robot)
 *
 */
int8_t angle_to_circles(circle_t *c1, circle_t *c2,
			 const point_t *b1, const point_t *b2,
			 double a_rad)
{
	point_t O;
	vect_t v;
	float l, d;

	/* reject negative or too small angles */
	if (a_rad <= 0.01)
		return -1;

	/* get position of O */
	O.x = (b1->x + b2->x) / 2;
	O.y = (b1->y + b2->y) / 2;

	/* get the length l */
	v.x = b2->x - b1->x;
	v.y = b2->y - b1->y;
	l = vect_norm(&v);

	/* distance from O to the center of the circle */
	/* XXX div by 0 when pi */
	d = l / (2 * tan(a_rad));

	/* get the circle c1 */
	vect_rot_trigo(&v);
	vect_resize(&v, d);
	if (c1) {
		c1->x = O.x + v.x;
		c1->y = O.y + v.y;
		c1->r = norm(b1->x, b1->y, c1->x, c1->y);
	}

	/* get the circle c2 */
	if (c2) {
		c2->x = O.x - v.x;
		c2->y = O.y - v.y;
		c2->r = norm(b1->x, b1->y, c1->x, c1->y);
	}

	return 0;
}

/* get the position of the robot from the angle of the 3 beacons */
int8_t angles_to_posxy(point_t *pos, double a01, double a12, double a20)
{
	circle_t c01, c12, c20;
	point_t dummy_pt, p1, p2, p3;

	dprintf("a01 = %2.2f\n", a01);
	dprintf("a12 = %2.2f\n", a12);
	dprintf("a20 = %2.2f\n", a20);

	if (angle_to_circles(&c01, NULL, &beacon0, &beacon1, a01))
		return -1;
	dprintf("circle: x=%2.2f y=%2.2f r=%2.2f\n", c01.x, c01.y, c01.r);

	if (angle_to_circles(&c12, NULL, &beacon1, &beacon2, a12))
		return -1;
	dprintf("circle: x=%2.2f y=%2.2f r=%2.2f\n", c12.x, c12.y, c12.r);

	if (angle_to_circles(&c20, NULL, &beacon2, &beacon0, a20))
		return -1;
	dprintf("circle: x=%2.2f y=%2.2f r=%2.2f\n", c20.x, c20.y, c20.r);

	if (circle_intersect(&c01, &c12, &p1, &dummy_pt) == 0)
		return -1;
	if (circle_intersect(&c12, &c20, &p2, &dummy_pt) == 0)
		return -1;
	if (circle_intersect(&c20, &c01, &dummy_pt, &p3) == 0)
		return -1;

	dprintf("p1: x=%2.2f y=%2.2f\n", p1.x, p1.y);
	dprintf("p2: x=%2.2f y=%2.2f\n", p2.x, p2.y);
	dprintf("p3: x=%2.2f y=%2.2f\n", p3.x, p3.y);

	/* if (norm(p1.x, p1.y, p2.x, p2.y) > POS_ACCURACY || */
	/*     norm(p2.x, p2.y, p3.x, p3.y) > POS_ACCURACY || */
	/*     norm(p3.x, p3.y, p1.x, p1.y) > POS_ACCURACY) */
	/* 	return -1; */

	pos->x = (p1.x + p2.x + p3.x) / 3.0;
	pos->y = (p1.y + p2.y + p3.y) / 3.0;

	return 0;
}

/* get the angles of beacons from xy pos */
int8_t posxy_to_angles(point_t pos, double *a01, double *a12,
		       double *a20, int err_num, float err_val)
{
	double a0, a1, a2;

	a0 = atan2(beacon0.y-pos.y, beacon0.x-pos.x);
	a1 = atan2(beacon1.y-pos.y, beacon1.x-pos.x);
	a2 = atan2(beacon2.y-pos.y, beacon2.x-pos.x);

	if (err_num == 0 || err_num == 3)
		a0 += (err_val * M_PI/180.);
	if (err_num == 1 || err_num == 3)
		a1 += (err_val * M_PI/180.);
	if (err_num == 2 || err_num == 3)
		a2 += (err_val * M_PI/180.);

	*a01 = a1-a0;
	if (*a01 < 0)
		*a01 += M_PI*2;
	*a12 = a2-a1;
	if (*a12 < 0)
		*a12 += M_PI*2;
	*a20 = a0-a2;
	if (*a20 < 0)
		*a20 += M_PI*2;

	return 0;
}

int8_t process_move_error(double x, double y, double speed,
			  double period, double angle, double *err)
{
	double a01, a12, a20;
	point_t pos, tmp;
	double a0, a1, a2;
	vect_t u,v;
	point_t pos2, pos3;

	pos.x = x;
	pos.y = y;

	/* from start to destination */
	v.x = cos(angle) * speed * period;
	v.y = sin(angle) * speed * period;

	/* first process real pos */
	posxy_to_angles(pos, &a01, &a12, &a20, -1, 0);

	/* vector covered during measure of a0 and a1 */
	u.x = v.x * a01 / (2*M_PI);
	u.y = v.y * a01 / (2*M_PI);
	pos2.x = pos.x + u.x;
	pos2.y = pos.y + u.y;

	/* vector covered during measure of a1 and a2 */
	u.x = v.x * a12 / (2*M_PI);
	u.y = v.y * a12 / (2*M_PI);
	pos3.x = pos2.x + u.x;
	pos3.y = pos2.y + u.y;

	dprintf("p0: x=%2.2f y=%2.2f\n", pos.x, pos.y);
	dprintf("p1: x=%2.2f y=%2.2f\n", pos2.x, pos2.y);
	dprintf("p2: x=%2.2f y=%2.2f\n", pos3.x, pos3.y);

	a0 = atan2(beacon0.y-pos.y, beacon0.x-pos.x);
	a1 = atan2(beacon1.y-pos2.y, beacon1.x-pos2.x);
	a2 = atan2(beacon2.y-pos3.y, beacon2.x-pos3.x);

	a01 = a1-a0;
	if (a01 < 0)
		a01 += M_PI*2;
	a12 = a2-a1;
	if (a12 < 0)
		a12 += M_PI*2;
	a20 = a0-a2;
	if (a20 < 0)
		a20 += M_PI*2;

	if (angles_to_posxy(&tmp, a01, a12, a20))
		return -1;
	*err = pt_norm(&tmp, &pos);
	if (*err > 50.) /* saturate error to 5cm */
		*err = 50.;
	return 0;
}
