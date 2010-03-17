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

extern const point_t beacon0;
extern const point_t beacon1;
extern const point_t beacon2;

/* get the position of the robot from the angle of the 3 beacons */
int8_t angles_to_posxy(point_t *pos, double a01, double a12, double a20);

/* get the position and angle of the robot from the angle of the 2
 * beacons, and the distance of 2 beacons */
int8_t ad_to_posxya(point_t *pos, double *a, int algo,
		    const point_t *b0, const point_t *b1, /* beacon position */
		    double a0, double a1, /* seen angle of beacons */
		    double d0, double d1 /* distance to beacons */ );
