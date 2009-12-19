/*  
 *  Copyright Droids Corporation (2007)
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
 *  Revision : $Id: obstacle_avoidance.h,v 1.1.2.7 2009-05-02 10:00:35 zer0 Exp $
 *
 *  Main code and algorithm: Fabrice DESCLAUX <serpilliere@droids-corp.org>
 *  Integration in Aversive: Olivier MATZ <zer0@droids-corp.org>
 */

/*
 * The algorithm is based on the "visible point" algorithm.
 * There are 3 inputs:
 *   - the play ground (basically the table, here a rectangle)
 *   - the objects to avoid, represented by polygones 
 *   - start/stop points (A, B)
 *
 * The algorithm will first find every ray formed by 2 points that can
 * "see" each others. Basically, if a polygon is between two points,
 * they cannot see each others. A side of a polygon is composed by 2
 * points that can se each others.
 *
 * From all these rays, we can create a graph. We affect for each ray
 * a weight with its own length.
 *
 * The algorithm executes Dijkstra to find the shortest path to go
 * from A to B.
 */

/*
 * As we run on 4Ko ram uC, we have static structures arrays to store:
 *  - MAX_POLY => represent the maximum polygons to avoid in the area.
 *  - MAX_PTS => maximize the sum of every polygons vertices.
 *  - MAX_RAYS => maximum number of rays. 
 *  - MAX_CHKPOINTS => maximum accepted checkpoints in the resulting path.
 *  - PLAYGROUND XXX => dimensions of the playground.
 */

#ifndef _OBSTACLE_AVOIDANCE_H_
#define _OBSTACLE_AVOIDANCE_H_

#include <obstacle_avoidance_config.h>

struct obstacle_avoidance {
	poly_t polys[MAX_POLY];  /* tab of polygons (obstacles) */
	point_t points[MAX_PTS]; /* tab of points, referenced by polys */
	uint8_t valid[MAX_PTS];
	int32_t pweight[MAX_PTS];
	uint8_t p[MAX_PTS];
	uint8_t pt[MAX_PTS];


	
	uint8_t ray_n;
	uint8_t cur_poly_idx;
	uint8_t cur_pt_idx;

	uint16_t weight[MAX_RAYS];
	union {
		uint8_t rays[MAX_RAYS*2];
		point_t res[MAX_CHKPOINTS];
	} u;
};

/* To save memory space here is the moemory representation of
 *   polygons/points:
 *
 *   We have an array of points (oa_ext_point_t points):  
 *  _____ _____ _____ _____ _____ _____ _____ _____ _____
 * |     |     |     |     |     |     |     |     |     |
 * | p0  | p1  | p0  | p1  | p2  | p3  | p0  | p1  | p2  |
 * |_____|_____|_____|_____|_____|_____|_____|_____|_____|
 *
 *
 *  ^            ^                       ^
 *  |            |                       |
 *  -polygon 0   -polygon 1              -polygon 2
 *  -2 vertices  -4 vertices             -3 vertices
 *
 *
 * And each polygon is represented by the sub array starting with the
 * point represented by oa_ext_point_t * pts and composed of uint8_t l; 
 * (in the oa_poly_t structure)
 */

/* reset oa without cleaning points */
void oa_reset(void);


/** Init the oa structure */
void oa_init(void);

/** 
 * Set the start and destination point.
 */
void oa_start_end_points(int32_t st_x, int32_t st_y, int32_t en_x, int32_t en_y);

/**
 * Create a new obstacle polygon. Return NULL on error.
 */
poly_t *oa_new_poly(uint8_t size);


/**
 * Dump status
 */
void oa_dump(void);

/**
 * set a point of the polygon.
 */
void oa_poly_set_point(poly_t *pol, int32_t x, int32_t y, uint8_t i);


/**
 * process the path from start to end. Return 0 on sucess.
 */
int8_t oa_process(void);

/**
 * get the result after a call to oa_process()
 */
point_t * oa_get_path(void);

#endif /* _OBSTACLE_AVOIDANCE_H_ */
