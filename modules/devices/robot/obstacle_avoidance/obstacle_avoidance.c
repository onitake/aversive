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
 *  Revision : $Id: obstacle_avoidance.c,v 1.1.2.8 2009-05-02 10:00:35 zer0 Exp $
 *
 *  Main code and algorithm: Fabrice DESCLAUX <serpilliere@droids-corp.org>
 *  Integration in Aversive: Olivier MATZ <zer0@droids-corp.org>
 */

#include <aversive.h>
#include <aversive/error.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <vect_base.h>
#include <lines.h>
#include <polygon.h>

#include <obstacle_avoidance.h>

#define GET_PT(a) (&(a) - &(oa.points[0]))

static struct obstacle_avoidance oa;

static void __oa_start_end_points(int32_t st_x, int32_t st_y,
				  int32_t en_x, int32_t en_y);

/* reset oa without reseting points coord */
void oa_reset(void)
{
	DEBUG(E_OA, "%s()", __FUNCTION__);

	memset(oa.valid, 0, sizeof(oa.valid));
	memset(oa.pweight, 0, sizeof(oa.pweight));
	memset(oa.weight, 0, sizeof(oa.weight));
	memset(oa.p, 0, sizeof(oa.p));
	memset(oa.pt, 0, sizeof(oa.pt));

	
}

/** Init the oa structure. Note: In the algorithm, the first polygon
 * is a dummy one, and is used to represent the START and END points
 * (so it has 2 vertices) */
void oa_init(void)
{
	DEBUG(E_OA, "%s()", __FUNCTION__);
	memset(&oa, 0, sizeof(oa));
	
	/* set a default start and point, reserve the first poly and
	 * the first 2 points for it */
	oa.polys[0].pts = oa.points;
	oa.polys[0].l = 2;
	__oa_start_end_points(0, 0, 100, 100);
	oa.cur_pt_idx = 2;
	oa.cur_poly_idx = 1;
}

/** 
 * Set the start and destination point. Return 0 on sucess
 */
static void __oa_start_end_points(int32_t st_x, int32_t st_y,
				  int32_t en_x, int32_t en_y)
{
	/* we always use the first 2 points of the table for start and end */
	oa.points[0].x = en_x;
	oa.points[0].y = en_y;

        /* Each point processed by Dijkstra is marked as valid. If we
	 * have unreachable points (out of playground or points inside
	 * polygons) Disjkstra won't mark them as valid. At the end of
	 * the algorithm, if the destination point is not marked as
	 * valid, there's no valid path to reach it. */

	oa.valid[GET_PT(oa.points[0])] = 0;
	/* the real dest is the start point for the algorithm */
	oa.pweight[GET_PT(oa.points[0])] = 1;

	oa.points[1].x = st_x;
	oa.points[1].y = st_y;
	oa.valid[GET_PT(oa.points[1])] = 0;
	oa.pweight[GET_PT(oa.points[1])] = 0;
}

/** 
 * Set the start and destination point. Return 0 on sucess
 */
void oa_start_end_points(int32_t st_x, int32_t st_y,
			 int32_t en_x, int32_t en_y)
{
	DEBUG(E_OA, "%s() (%ld,%ld) (%ld,%ld)", __FUNCTION__,
	      st_x, st_y, en_x, en_y);

	__oa_start_end_points(st_x, st_y, en_x, en_y);
}


/**
 * Create a new obstacle polygon. Return NULL on error.
 */
poly_t *oa_new_poly(uint8_t size)
{
	DEBUG(E_OA, "%s(size=%d)", __FUNCTION__, size);

	if (oa.cur_pt_idx + size > MAX_PTS)
		return NULL;
	if (oa.cur_poly_idx + 1 > MAX_POLY)
		return NULL;

	oa.polys[oa.cur_poly_idx].l = size;
	oa.polys[oa.cur_poly_idx].pts = &oa.points[oa.cur_pt_idx];
	oa.cur_pt_idx += size;

	return &oa.polys[oa.cur_poly_idx++];
}

/**
 * Add a point to the polygon.
 */
void oa_poly_set_point(poly_t *pol, 
			 int32_t x, int32_t y, uint8_t i)
{
	DEBUG(E_OA, "%s() (%ld,%ld)", __FUNCTION__, x, y);
	
	pol->pts[i].x = x;
	pol->pts[i].y = y;
	oa.valid[GET_PT(pol->pts[i])] = 0;
	oa.pweight[GET_PT(pol->pts[i])] = 0;
}

point_t * oa_get_path(void)
{
	return oa.u.res;
}

void oa_dump(void)
{
	uint8_t i,j;
	poly_t *poly;
	point_t *pt;

	printf_P(PSTR("-- OA dump --\r\n"));
	printf_P(PSTR("nb_polys: %d\r\n"), oa.cur_poly_idx);
	printf_P(PSTR("nb_pts: %d\r\n"), oa.cur_pt_idx);
	for (i=0; i<oa.cur_poly_idx; i++) {
		poly = &oa.polys[i];
		printf_P(PSTR("poly #%d\r\n"), i);
		for (j=0; j<poly->l; j++) {
			pt = &poly->pts[j];
			printf_P(PSTR("  pt #%d (%2.2f,%2.2f)\r\n"), j, pt->x, pt->y);
		}
	}
}

/* Iterative Dijkstra algorithm: The valid filed is used to determine if:
 *   1: this point has been visited, his weight is correct.
 *   2: the point must be visited.
 *
 * The algorithm does: find a point that must be visited (2) update
 * his weight, mark it as (1) and update all his neightbours has 2.
 *
 * The algorithm ends when no (2) points are found
 *
 * A point with weight 0 is a point that has not been visited (so
 * weight is not affected yet); This explain why first point must have
 * a start weight different than 0.
 *
 * When the algo finds a shorter path to reach a point B from point A,
 * it will store in (p, pt) the parent point. This is important to
 * remenber and extract the solution path. */
void 
dijkstra(uint8_t start_p, uint8_t start)
{
	uint8_t i;
	int8_t add;
	int8_t finish = 0;
	/* weight == 0 means not visited */
	/* weight == 1 for start */
	
	/* find all point linked to start */

	oa.valid[GET_PT(oa.polys[start_p].pts[start])] = 2;

	while (!finish){
		finish = 1;

		for (start_p = 0;start_p<MAX_POLY;start_p++) {
			for (start = 0;start<oa.polys[start_p].l;start++) {
				if (oa.valid[GET_PT(oa.polys[start_p].pts[start])] != 2)
					continue;
				add = -2;		

			        /* For all points that must be
				 * visited, we look for rays that
				 * start or stop at this point.  As
				 * ray array is (poly_num, point_num)
				 * we wtep 2 by 2. */
				for (i=0 ; i<oa.ray_n ; i+=2) {

					/* If index is even in the
					 * aray, we have a start point
					 * ray, so connected point is
					 * i+2;
					 *
					 * If index is odd, we are in stop
					 * point and ray start point is at
					 * i-2 pos */
					add = -add;					
					
					if (start_p != oa.u.rays[i] || start != oa.u.rays[i+1])
						continue;

					if ((oa.pweight[GET_PT(oa.polys[oa.u.rays[i+add]].pts[oa.u.rays[i+add+1]])] != 0) &&
					    (oa.pweight[GET_PT(oa.polys[start_p].pts[start])]+oa.weight[i/4] >=
					     oa.pweight[GET_PT(oa.polys[oa.u.rays[i+add]].pts[oa.u.rays[i+add+1]])]))
						continue;
					
					oa.p[GET_PT(oa.polys[oa.u.rays[i+add]].pts[oa.u.rays[i+add+1]])] = start_p;
					oa.pt[GET_PT(oa.polys[oa.u.rays[i+add]].pts[oa.u.rays[i+add+1]])] = start;
					oa.valid[GET_PT(oa.polys[oa.u.rays[i+add]].pts[oa.u.rays[i+add+1]])]=2;
					oa.pweight[GET_PT(oa.polys[oa.u.rays[i+add]].pts[oa.u.rays[i+add+1]])] = 
						oa.pweight[GET_PT(oa.polys[start_p].pts[start])]+oa.weight[i/4];
							
					oa.valid[GET_PT(oa.polys[start_p].pts[start])] = 1;
					finish = 0;				
					DEBUG(E_OA, "%s() (%ld,%ld p=%ld) %ld (%ld,%ld p=%ld)\r\n", __FUNCTION__,
					      oa.polys[start_p].pts[start].x, oa.polys[start_p].pts[start].y,
					      oa.pweight[GET_PT(oa.polys[start_p].pts[start])],oa.weight[i/4],
					      
					      oa.polys[oa.u.rays[i+add]].pts[oa.u.rays[i+add+1]].x, 
					      oa.polys[oa.u.rays[i+add]].pts[oa.u.rays[i+add+1]].y,
					      oa.pweight[GET_PT(oa.polys[oa.u.rays[i+add]].pts[oa.u.rays[i+add+1]])]
					      );
				}
			}
		}
	}
}


/* display the path */
int8_t
get_path(poly_t *polys, uint8_t *rays)
{
	uint8_t p, pt, p1, pt1, i;

	p=0;
	pt=1;
	i=0;
	
	/* forget the first point */
	
	while (!(p==0 && pt==0)) {

		if (i>=MAX_CHKPOINTS)
			return -1;

		if (oa.valid[GET_PT(polys[p].pts[pt])]==0) {
			DEBUG(E_OA, "invalid path!");
			return -1;
		}

		p1 = oa.p[GET_PT(polys[p].pts[pt])];
		pt1 =  oa.pt[GET_PT(polys[p].pts[pt])];
		p = p1; pt = pt1;
		oa.u.res[i].x = polys[p].pts[pt].x;
		oa.u.res[i].y = polys[p].pts[pt].y;
		DEBUG(E_OA, "result[%d]: %d, %d", i, oa.u.res[i].x, oa.u.res[i].y);
		i++;
	}
	
	return i;
}

int8_t 
oa_process(void)
{
	uint8_t ret;
	uint8_t i;

	/* First we compute the visibility graph */
	ret = calc_rays(oa.polys, oa.cur_poly_idx, oa.u.rays);
	DEBUG(E_OA, "nb_rays = %d", ret);

	DEBUG(E_OA, "Ray list");
	for (i=0;i<ret;i+=4) {
		DEBUG(E_OA, "%d,%d -> %d,%d", oa.u.rays[i], oa.u.rays[i+1], 
		      oa.u.rays[i+2], oa.u.rays[i+3]);
	}
	
	/* Then we affect the rays lengths to their weights */
	calc_rays_weight(oa.polys, oa.cur_poly_idx, 
			 oa.u.rays, ret, oa.weight);
	
	DEBUG(E_OA, "Ray weights");
	for (i=0;i<ret;i+=4) {
		DEBUG(E_OA, "%d,%d -> %d,%d (%d)", 
		       (int)oa.polys[oa.u.rays[i]].pts[oa.u.rays[i+1]].x,
		       (int)oa.polys[oa.u.rays[i]].pts[oa.u.rays[i+1]].y,
		       (int)oa.polys[oa.u.rays[i+2]].pts[oa.u.rays[i+3]].x,
		       (int)oa.polys[oa.u.rays[i+2]].pts[oa.u.rays[i+3]].y,
		       oa.weight[i/4]);
	}
	
	/* We aplly dijkstra on the visibility graph from the start
	 * point (point 0 of the polygon 0) */
	oa.ray_n = ret;
	DEBUG(E_OA, "dijkstra ray_n = %d", ret);
	dijkstra(0, 0);

	/* As dijkstra sets the parent points in the resulting graph,
	 * we can backtrack the solution path. */
	return get_path(oa.polys, oa.u.rays);
}
