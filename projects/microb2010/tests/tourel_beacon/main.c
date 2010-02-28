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

#define AREA_X 3000
#define AREA_Y 2100

#define ANGLE_EPSILON 0.005

const point_t beacon0 = { 0, 1050 };
const point_t beacon1 = { 3000, 0 };
const point_t beacon2 = { 3000, 2100 };

static inline double abs_dbl(double x)
{
	if (x > 0)
		return x;
	else
		return -x;
}

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

	/* reject too small angles */
	if (a_rad <= 0.01 && a_rad >= -0.01)
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
		c1->r = xy_norm(b1->x, b1->y, c1->x, c1->y);
	}

	/* get the circle c2 */
	if (c2) {
		c2->x = O.x - v.x;
		c2->y = O.y - v.y;
		c2->r = xy_norm(b1->x, b1->y, c1->x, c1->y);
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

	/* if (xy_norm(p1.x, p1.y, p2.x, p2.y) > POS_ACCURACY || */
	/*     xy_norm(p2.x, p2.y, p3.x, p3.y) > POS_ACCURACY || */
	/*     xy_norm(p3.x, p3.y, p1.x, p1.y) > POS_ACCURACY) */
	/* 	return -1; */

	pos->x = (p1.x + p2.x + p3.x) / 3.0;
	pos->y = (p1.y + p2.y + p3.y) / 3.0;

	return 0;
}

static inline int8_t is_pt_in_area(point_t pt)
{
	if (pt.x < 0 || pt.y < 0)
		return 0;
	if (pt.x > AREA_X || pt.y > AREA_Y)
		return 0;
	return 1;
}

/* intersect the circle formed by one distance info with the circle
 * crossing the 2 beacons */
static int8_t
ad_to_posxy_one(point_t *pos,
		const point_t *b0, const point_t *b1, /* beacon position */
		const circle_t *cd, const circle_t *c01, /* circles to intersect */
		double a01) /* seen angle of beacons */
{
	double tmp_a01_p1, tmp_a01_p2;
	point_t p1, p2;
	uint8_t p1_ok=0, p2_ok=0;

	if (circle_intersect(c01, cd, &p1, &p2) == 0)
		return -1;

	dprintf("p1: x=%2.2f y=%2.2f\n", p1.x, p1.y);
	dprintf("p2: x=%2.2f y=%2.2f\n", p2.x, p2.y);

	dprintf("real a01: %2.2f\n", a01);

	tmp_a01_p1 = atan2(b1->y-p1.y, b1->x-p1.x) -
		atan2(b0->y-p1.y, b0->x-p1.x);
	dprintf("a01-1: %2.2f\n", tmp_a01_p1);

	tmp_a01_p2 = atan2(b1->y-p2.y, b1->x-p2.x) -
		atan2(b0->y-p2.y, b0->x-p2.x);
	dprintf("a01-2: %2.2f\n", tmp_a01_p2);

	/* in some conditions, we already know the position of the
	 * robot at this step */
	p1_ok = is_pt_in_area(p1) &&
		abs_dbl(tmp_a01_p1 - a01) < ANGLE_EPSILON;

	p2_ok = is_pt_in_area(p2) &&
		abs_dbl(tmp_a01_p2 - a01) < ANGLE_EPSILON;
	if (!p1_ok && p2_ok) {
		*pos = p2;
		return 0;
	}
	if (p1_ok && !p2_ok) {
		*pos = p1;
		return 0;
	}
	return -1;
}


/* get the position and angle of the robot from the angle of the 2
 * beacons, and the distance of 2 beacons */
static int8_t ad_to_posxya(point_t *pos, double *a, int algo,
			   const point_t *b0, const point_t *b1, /* beacon position */
			   double a0, double a1, /* seen angle of beacons */
			   double d0, double d1 /* distance to beacons */ )
{
	circle_t cd0, cd1, c01;
	double a01;

	dprintf("algo=%d a0=%2.2f a1=%2.2f d0=%2.2f d1=%2.2f\n",
		algo, a0, a1, d0, d1);

	/* the first 2 circles depends on distance between robot and
	 * beacons */
	cd0.x = b0->x;
	cd0.y = b0->y;
	cd0.r = d0;
	dprintf("circle: x=%2.2f y=%2.2f r=%2.2f\n", cd0.x, cd0.y, cd0.r);
	cd1.x = b1->x;
	cd1.y = b1->y;
	cd1.r = d1;
	dprintf("circle: x=%2.2f y=%2.2f r=%2.2f\n", cd1.x, cd1.y, cd1.r);

	/* the third circle depends on angle between b0 and b1 */
	a01 = a1-a0;
	if (angle_to_circles(&c01, NULL, b0, b1, a01))
		return -1;
	dprintf("circle: x=%2.2f y=%2.2f r=%2.2f\n", c01.x, c01.y, c01.r);

	switch (algo) {

		/* take the closer beacon first */
	case 0:
		if (d0 < d1) {
			if (ad_to_posxy_one(pos, b0, b1, &cd0, &c01, a01) == 0)
				return 0;
			if (ad_to_posxy_one(pos, b0, b1, &cd1, &c01, a01) == 0)
				return 0;
			return -1;
		}
		else {
			if (ad_to_posxy_one(pos, b0, b1, &cd1, &c01, a01) == 0)
				return 0;
			if (ad_to_posxy_one(pos, b0, b1, &cd0, &c01, a01) == 0)
				return 0;
			return -1;
		}
		break;
	case 1:
		/* b0 only */
		if (ad_to_posxy_one(pos, b0, b1, &cd0, &c01, a01) == 0)
			return 0;
		break;
	case 2:
		/* b0 only */
		if (ad_to_posxy_one(pos, b0, b1, &cd1, &c01, a01) == 0)
			return 0;
		break;
	default:
		break;
	}
	/* not found */
	return -1;
}

/* get the angles of beacons from xy pos */
int8_t posxy_to_abs_angles(point_t pos, double *a0, double *a1,
			   double *a2, int err_num, float err_val)
{
	*a0 = atan2(beacon0.y-pos.y, beacon0.x-pos.x);
	*a1 = atan2(beacon1.y-pos.y, beacon1.x-pos.x);
	*a2 = atan2(beacon2.y-pos.y, beacon2.x-pos.x);

	if (err_num == 0 || err_num == 3)
		*a0 += (err_val * M_PI/180.);
	if (err_num == 1 || err_num == 3)
		*a1 += (err_val * M_PI/180.);
	if (err_num == 2 || err_num == 3)
		*a2 += (err_val * M_PI/180.);

	return 0;
}

/* get the angles of beacons from xy pos */
int8_t posxy_to_angles(point_t pos, double *a01, double *a12,
		       double *a20, int err_num, float err_val)
{
	double a0, a1, a2;
	posxy_to_abs_angles(pos, &a0, &a1, &a2, err_num, err_val);

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

/* whole process is around 3ms on atmega128 at 16Mhz */
int main(int argc, char **argv)
{
	double a0, a1, a2;
	double a01, a12, a20;
	point_t pos, tmp;
	const char *mode = "nothing";

#ifdef HOST_VERSION
	if (argc < 2) {
		printf("bad args\n");
		return -1;
	}
	mode = argv[1];
#else
	mode = "angle2pos";
	argc = 5;
	a0 = -1.;
	a1 = 1.65;
	a2 = 3.03;
#endif

	/*
	 * angle2pos a0 a1 a2:
	 *   (angles in radian, -pi < a < pi)
	 */
	if (argc == 5 && strcmp(mode, "angle2pos") == 0) {
#ifdef HOST_VERSION
		dprint = 1;
		a0 = atof(argv[2]);
		a1 = atof(argv[3]);
		a2 = atof(argv[4]);
#endif

		/* */
		a01 = a1-a0;
		if (a01 < 0)
			a01 += 2*M_PI;
		a12 = a2-a1;
		if (a12 < 0)
			a12 += 2*M_PI;
		a20 = a0-a2;
		if (a20 < 0)
			a20 += 2*M_PI;

		if (angles_to_posxy(&pos, a01, a12, a20) < 0)
			return -1;
		printf("p0: x=%2.2f y=%2.2f\n", pos.x, pos.y);
		return 0;
	}

	/*
	 * ad2pos a0 a1 d0 d1:
	 *   (angles in radian, -pi < a < pi)
	 *   (distances in mm)
	 */
	if (argc == 6 && strcmp(mode, "ad2pos") == 0) {
		double a, d0, d1;

		dprint = 1;
		a0 = atof(argv[2]);
		a1 = atof(argv[3]);
		d0 = atof(argv[4]);
		d1 = atof(argv[5]);

		/* */
		if (ad_to_posxya(&pos, &a, 0, &beacon0, &beacon1,
				 a0, a1, d0, d1) < 0)
			return -1;
		printf("p0: x=%2.2f y=%2.2f a=%2.2f\n", pos.x, pos.y, a);
		return 0;
	}

	if (argc == 4 && strcmp(mode, "simple_error") == 0) {
		int x, y;
		int err_num;
		double err_val_deg;
		double err;

		err_num = atof(argv[2]); /* which beacon sees an error */
		err_val_deg = atof(argv[3]); /* how many degrees of error */

		for (x=0; x<300; x++) {
			for (y=0; y<210; y++) {
				pos.x = x*10;
				pos.y = y*10;
				posxy_to_angles(pos, &a01, &a12, &a20,
						err_num, err_val_deg);
				if (angles_to_posxy(&tmp, a01, a12, a20))
					continue;
				err = pt_norm(&tmp, &pos);
				if (err > 50.) /* saturate error to 5cm */
					err = 50.;
				printf("%d %d %2.2f\n", x, y, err);
			}
		}
		return 0;
	}

	/* da_error algo errpercent errdeg */
	if (argc == 5 && strcmp(mode, "da_error") == 0) {
		int x, y, algo;
		double err_val_deg;
		double err_val_percent;
		double err, d0, d1, a;

		algo = atoi(argv[2]);
		err_val_percent = atof(argv[3]); /* how many % of error for dist */
		err_val_deg = atof(argv[4]); /* how many degrees of error */

		for (x=0; x<300; x++) {
			for (y=0; y<210; y++) {

				pos.x = x*10;
				pos.y = y*10;
				posxy_to_abs_angles(pos, &a0, &a1, &a2,
						    0, err_val_deg);
				d0 = pt_norm(&pos, &beacon0);
				d0 += d0 * err_val_percent / 100.;
				d1 = pt_norm(&pos, &beacon1);
				d1 += d1 * err_val_percent / 100.;

				if (ad_to_posxya(&tmp, &a, algo, &beacon0, &beacon1,
						 a0, a1, d0, d1) < 0)
					continue;

				err = pt_norm(&tmp, &pos);
				if (err > 50.) /* saturate error to 5cm */
					err = 50.;
				printf("%d %d %2.2f\n", x, y, err);
			}
		}
		return 0;
	}

	if ((argc == 5 || argc == 7)
	    && strcmp(argv[1], "move_error") == 0) {
		int x, y;
		double angle, speed, period, err;

		speed = atof(argv[2]); /* speed in m/s ( = mm/ms) */
		period = atof(argv[3]); /* period of turret in ms */
		angle = atof(argv[4]); /* direction of moving */
		if (argc == 7) {
			dprint = 1;
			process_move_error(atof(argv[5]), atof(argv[6]),
					   speed, period, angle, &err);
			printf("%2.2f %2.2f %2.2f\n", atof(argv[5]),
			       atof(argv[6]), err);
			return 0;
		}

		for (x=0; x<300; x++) {
			for (y=0; y<210; y++) {
				pos.x = x*10;
				pos.y = y*10;
				if (process_move_error(pos.x, pos.y,
						       speed, period, angle,
						       &err) < 0)
					continue;
				printf("%d %d %2.2f\n", x, y, err);
			}
		}
		return 0;
	}

	printf("bad args\n");
	return -1;
}
