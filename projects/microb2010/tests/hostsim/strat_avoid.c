/*  
 *  Copyright Droids Corporation, Microb Technology (2009)
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
 *  Revision : $Id: strat_avoid.c,v 1.5 2009-11-08 17:24:33 zer0 Exp $
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <uart.h>
#include <time.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <trajectory_manager.h>
#include <vect_base.h>
#include <lines.h>
#include <polygon.h>
#include <obstacle_avoidance.h>
#include <blocking_detection_manager.h>
#include <robot_system.h>
#include <position_manager.h>

#include <rdline.h>
#include <parse.h>

#include "main.h"
#include "strat.h"
#include "strat_base.h"
#include "strat_utils.h"

#define EDGE_NUMBER 5
void set_rotated_pentagon(poly_t *pol, const point_t *robot_pt,
			  int16_t radius, int16_t x, int16_t y)
{

	double c_a, s_a;
	uint8_t i;
	double px1, py1, px2, py2;
	double a_rad;

	a_rad = atan2(y - robot_pt->y, x - robot_pt->x);

	/* generate pentagon  */
	c_a = cos(-2*M_PI/EDGE_NUMBER);
	s_a = sin(-2*M_PI/EDGE_NUMBER);

	/*
	px1 = radius;
	py1 = 0;
	*/
	px1 = radius * cos(a_rad + 2*M_PI/(2*EDGE_NUMBER));
	py1 = radius * sin(a_rad + 2*M_PI/(2*EDGE_NUMBER));


	for (i = 0; i < EDGE_NUMBER; i++){
		oa_poly_set_point(pol, x + px1, y + py1, i);
		
		px2 = px1*c_a + py1*s_a;
		py2 = -px1*s_a + py1*c_a;

		px1 = px2;
		py1 = py2;
	}
}

void set_rotated_poly(poly_t *pol, const point_t *robot_pt, 
		      int16_t w, int16_t l, int16_t x, int16_t y)
{
	double tmp_x, tmp_y;
	double a_rad;

	a_rad = atan2(y - robot_pt->y, x - robot_pt->x);

	DEBUG(E_USER_STRAT, "%s() x,y=%d,%d a_rad=%2.2f", 
	      __FUNCTION__, x, y, a_rad);

	/* point 1 */
	tmp_x = w;
	tmp_y = l;
	rotate(&tmp_x, &tmp_y, a_rad);
	tmp_x += x;
	tmp_y += y;
	oa_poly_set_point(pol, tmp_x, tmp_y, 0);
	
	/* point 2 */
	tmp_x = -w;
	tmp_y = l;
	rotate(&tmp_x, &tmp_y, a_rad);
	tmp_x += x;
	tmp_y += y;
	oa_poly_set_point(pol, tmp_x, tmp_y, 1);
	
	/* point 3 */
	tmp_x = -w;
	tmp_y = -l;
	rotate(&tmp_x, &tmp_y, a_rad);
	tmp_x += x;
	tmp_y += y;
	oa_poly_set_point(pol, tmp_x, tmp_y, 2);
	
	/* point 4 */
	tmp_x = w;
	tmp_y = -l;
	rotate(&tmp_x, &tmp_y, a_rad);
	tmp_x += x;
	tmp_y += y;
	oa_poly_set_point(pol, tmp_x, tmp_y, 3);
}

#define DISC_X CENTER_X
#define DISC_Y CENTER_Y

void set_central_disc_poly(poly_t *pol, const point_t *robot_pt)
{
	set_rotated_pentagon(pol, robot_pt, DISC_PENTA_DIAG,
			     DISC_X, DISC_Y);
}

#ifdef HOMOLOGATION
/* /!\ half size */
#define O_WIDTH  400
#define O_LENGTH 550
#else
/* /!\ half size */
#define O_WIDTH  360
#define O_LENGTH 500
#endif

void set_opponent_poly(poly_t *pol, const point_t *robot_pt, int16_t w, int16_t l)
{
	int16_t x, y;
	get_opponent_xy(&x, &y);
	DEBUG(E_USER_STRAT, "oponent at: %d %d", x, y);
	
	/* place poly even if invalid, because it's -100 */
	set_rotated_poly(pol, robot_pt, w, l, x, y);
}

/* don't care about polygons further than this distance for escape */
#define ESCAPE_POLY_THRES 1000

/* don't reduce opp if opp is too far */
#define REDUCE_POLY_THRES 600

/* has to be longer than any poly */
#define ESCAPE_VECT_LEN 3000

/*
 * Go in playground, loop until out of poly. The argument robot_pt is 
 * the pointer to the current position of the robot.
 * Return 0 if there was nothing to do.
 * Return 1 if we had to move. In this case, update the theorical 
 * position of the robot in robot_pt.
 */
static int8_t go_in_area(point_t *robot_pt)
{
	point_t poly_pts_area[4];
	poly_t poly_area;
	point_t disc_pt, dst_pt;

	disc_pt.x = DISC_X;
	disc_pt.y = DISC_Y;

	/* Go in playground */
	if (!is_in_boundingbox(robot_pt)){
		NOTICE(E_USER_STRAT, "not in playground %"PRIi32", %"PRIi32"",
		       robot_pt->x, robot_pt->y);

		poly_area.l = 4;
		poly_area.pts = poly_pts_area;
		poly_pts_area[0].x = strat_infos.area_bbox.x1;
		poly_pts_area[0].y = strat_infos.area_bbox.y1;

		poly_pts_area[1].x = strat_infos.area_bbox.x2;
		poly_pts_area[1].y = strat_infos.area_bbox.y1;

		poly_pts_area[2].x = strat_infos.area_bbox.x2;
		poly_pts_area[2].y = strat_infos.area_bbox.y2;

		poly_pts_area[3].x = strat_infos.area_bbox.x1;
		poly_pts_area[3].y = strat_infos.area_bbox.y2;

		is_crossing_poly(*robot_pt, disc_pt, &dst_pt, &poly_area);
		NOTICE(E_USER_STRAT, "pt dst %"PRIi32", %"PRIi32"", dst_pt.x, dst_pt.y);
		
		strat_goto_xy_force(dst_pt.x, dst_pt.y);

		robot_pt->x = dst_pt.x;
		robot_pt->y = dst_pt.y;

		NOTICE(E_USER_STRAT, "GOTO %"PRIi32",%"PRIi32"",
		       dst_pt.x, dst_pt.y);

		return 1;
	}

	return 0;
}


/*
 * Escape from polygons if needed.
 * robot_pt is the current position of the robot, it will be
 * updated.
 */
static int8_t escape_from_poly(point_t *robot_pt,
			       poly_t *pol_disc,
			       int16_t opp_x, int16_t opp_y, 
			       int16_t opp_w, int16_t opp_l, 
			       poly_t *pol_opp)
{
	uint8_t in_disc = 0, in_opp = 0;
	double escape_dx = 0, escape_dy = 0;
	double disc_dx = 0, disc_dy = 0;
	double opp_dx = 0, opp_dy = 0;
	double len;
	point_t opp_pt, disc_pt, dst_pt;
	point_t intersect_disc_pt, intersect_opp_pt;

	opp_pt.x = opp_x;
	opp_pt.y = opp_y;
	disc_pt.x = DISC_X;
	disc_pt.y = DISC_Y;

	/* escape from other poly if necessary */
	if (is_in_poly(robot_pt, pol_disc) == 1)
		in_disc = 1;
	if (is_in_poly(robot_pt, pol_opp) == 1)
		in_opp = 1;

	if (in_disc == 0 && in_opp == 0) {
		NOTICE(E_USER_STRAT, "no need to escape");
		return 0;
	}
	
	NOTICE(E_USER_STRAT, "in_disc=%d, in_opp=%d", in_disc, in_opp);
	
	/* process escape vector */

	if (distance_between(robot_pt->x, robot_pt->y, DISC_X, DISC_Y) < ESCAPE_POLY_THRES) {
		disc_dx = robot_pt->x - DISC_X;
		disc_dy = robot_pt->y - DISC_Y;
		NOTICE(E_USER_STRAT, " robot is near disc: vect=%2.2f,%2.2f",
		       disc_dx, disc_dy);
		len = norm(disc_dx, disc_dy);
		if (len != 0) {
			disc_dx /= len;
			disc_dy /= len;
		}
		else {
			disc_dx = 1.0;
			disc_dy = 0.0;
		}
		escape_dx += disc_dx;
		escape_dy += disc_dy;
	}

	if (distance_between(robot_pt->x, robot_pt->y, opp_x, opp_y) < ESCAPE_POLY_THRES) {
		opp_dx = robot_pt->x - opp_x;
		opp_dy = robot_pt->y - opp_y;
		NOTICE(E_USER_STRAT, " robot is near opp: vect=%2.2f,%2.2f",
		       opp_dx, opp_dy);
		len = norm(opp_dx, opp_dy);
		if (len != 0) {
			opp_dx /= len;
			opp_dy /= len;
		}
		else {
			opp_dx = 1.0;
			opp_dy = 0.0;
		}
		escape_dx += opp_dx;
		escape_dy += opp_dy;
	}

	/* normalize escape vector */
	len = norm(escape_dx, escape_dy);
	if (len != 0) {
		escape_dx /= len;
		escape_dy /= len;
	}
	else {
		if (pol_disc != NULL) {
			/* rotate 90° */
			escape_dx = disc_dy;
			escape_dy = disc_dx;
		}
		else if (pol_opp != NULL) {
			/* rotate 90° */
			escape_dx = opp_dy;
			escape_dy = opp_dx;
		}
		else { /* should not happen */
			opp_dx = 1.0;
			opp_dy = 0.0;
		}
	}

	NOTICE(E_USER_STRAT, " escape vect = %2.2f,%2.2f",
	       escape_dx, escape_dy);

	/* process the correct len of escape vector */

	dst_pt.x = robot_pt->x + escape_dx * ESCAPE_VECT_LEN;
	dst_pt.y = robot_pt->y + escape_dy * ESCAPE_VECT_LEN;

	NOTICE(E_USER_STRAT, "robot pt %"PRIi32" %"PRIi32,
	       robot_pt->x, robot_pt->y);
	NOTICE(E_USER_STRAT, "dst point %"PRIi32",%"PRIi32,
	       dst_pt.x, dst_pt.y);

	if (in_disc) {
		if (is_crossing_poly(*robot_pt, dst_pt, &intersect_disc_pt,
				     pol_disc) == 1) {
			/* we add 2 mm to be sure we are out of th polygon */
			dst_pt.x = intersect_disc_pt.x + escape_dx * 2;
			dst_pt.y = intersect_disc_pt.y + escape_dy * 2;
			if (is_point_in_poly(pol_opp, dst_pt.x, dst_pt.y) != 1) {

				if (!is_in_boundingbox(&dst_pt))
					return -1;
				
				NOTICE(E_USER_STRAT, "GOTO %"PRIi32",%"PRIi32"",
				       dst_pt.x, dst_pt.y);

				strat_goto_xy_force(dst_pt.x, dst_pt.y);

				robot_pt->x = dst_pt.x;
				robot_pt->y = dst_pt.y;

				return 0;
			}
		}
	}

	if (in_opp) {
		if (is_crossing_poly(*robot_pt, dst_pt, &intersect_opp_pt,
				     pol_opp) == 1) {
			/* we add 2 cm to be sure we are out of th polygon */
			dst_pt.x = intersect_opp_pt.x + escape_dx * 2;
			dst_pt.y = intersect_opp_pt.y + escape_dy * 2;

			if (is_point_in_poly(pol_disc, dst_pt.x, dst_pt.y) != 1) {

				if (!is_in_boundingbox(&dst_pt))
					return -1;
				
				NOTICE(E_USER_STRAT, "GOTO %"PRIi32",%"PRIi32"",
				       dst_pt.x, dst_pt.y);

				strat_goto_xy_force(dst_pt.x, dst_pt.y);

				robot_pt->x = dst_pt.x;
				robot_pt->y = dst_pt.y;

				return 0;
			}
		}
	}

	/* should not happen */
	return -1;
}


static int8_t __goto_and_avoid(int16_t x, int16_t y,
			       uint8_t flags_intermediate,
			       uint8_t flags_final,
			       uint8_t forward)
{
	int8_t len = -1, i;
	point_t *p;
	poly_t *pol_disc, *pol_opp;
	int8_t ret;
	int16_t opp_w, opp_l, opp_x, opp_y;
	point_t p_dst, robot_pt;

	DEBUG(E_USER_STRAT, "%s(%d,%d) flags_i=%x flags_f=%x forw=%d",
	      __FUNCTION__, x, y, flags_intermediate, flags_final, forward);

 retry:
	get_opponent_xy(&opp_x, &opp_y);
	opp_w = O_WIDTH;
	opp_l = O_LENGTH;

	robot_pt.x = position_get_x_s16(&mainboard.pos);
	robot_pt.y = position_get_y_s16(&mainboard.pos);
	
	oa_init();
	pol_disc = oa_new_poly(5);
	set_central_disc_poly(pol_disc, &robot_pt);
	pol_opp = oa_new_poly(4);
	set_opponent_poly(pol_opp, &robot_pt, O_WIDTH, O_LENGTH);

	/* If we are not in the limited area, try to go in it. */
	ret = go_in_area(&robot_pt);

	/* check that destination is valid */
	p_dst.x = x;
	p_dst.y = y;
	if (!is_in_boundingbox(&p_dst)) {
		NOTICE(E_USER_STRAT, " dst is not in playground");
		return END_ERROR;
	}
	if (is_point_in_poly(pol_disc, x, y)) {
		NOTICE(E_USER_STRAT, " dst is in disc");
		return END_ERROR;
	}
	if (is_point_in_poly(pol_opp, x, y)) {
		NOTICE(E_USER_STRAT, " dst is in opp");
		return END_ERROR;
	}

	/* now start to avoid */
	while (opp_w && opp_l) {

		/* robot_pt is not updated if it fails */
		ret = escape_from_poly(&robot_pt,
				       pol_disc, opp_x, opp_y, 
				       opp_w, opp_l, pol_opp);
		if (ret == 0) {
			oa_reset();
			oa_start_end_points(robot_pt.x, robot_pt.y, x, y);
			/* oa_dump(); */
	
			len = oa_process();
			if (len >= 0)
				break;
		}
		if (distance_between(robot_pt.x, robot_pt.y, opp_x, opp_y) < REDUCE_POLY_THRES ) {
			if (opp_w == 0)
				opp_l /= 2;
			opp_w /= 2;
		}
		else {
			NOTICE(E_USER_STRAT, "oa_process() returned %d", len);
			return END_ERROR;
		}

		NOTICE(E_USER_STRAT, "reducing opponent %d %d", opp_w, opp_l);
		set_opponent_poly(pol_opp, &robot_pt, opp_w, opp_l);
	}
	
	p = oa_get_path();
	for (i=0 ; i<len ; i++) {
		DEBUG(E_USER_STRAT, "With avoidance %d: x=%"PRIi32" y=%"PRIi32"", i, p->x, p->y);

		if (forward)
			trajectory_goto_forward_xy_abs(&mainboard.traj, p->x, p->y);
		else
			trajectory_goto_backward_xy_abs(&mainboard.traj, p->x, p->y);

		/* no END_NEAR for the last point */
		if (i == len - 1)
			ret = wait_traj_end(flags_final);
		else
			ret = wait_traj_end(flags_intermediate);

		if (ret == END_BLOCKING) {
			DEBUG(E_USER_STRAT, "Retry avoidance %s(%d,%d)",
			      __FUNCTION__, x, y);
			goto retry;
		}
		else if (ret == END_OBSTACLE) {
			/* brake and wait the speed to be slow */
			DEBUG(E_USER_STRAT, "Retry avoidance %s(%d,%d)",
			      __FUNCTION__, x, y);
			goto retry;
		}
		/* else if it is not END_TRAJ or END_NEAR, return */
		else if (!TRAJ_SUCCESS(ret)) {
			return ret;
		}
		p++;
	}
	
	return END_TRAJ;
}

/* go forward to a x,y point. use current speed for that */
uint8_t goto_and_avoid_forward(int16_t x, int16_t y, uint8_t flags_intermediate,
			       uint8_t flags_final)
{
	return __goto_and_avoid(x, y, flags_intermediate, flags_final, 1);
}

/* go backward to a x,y point. use current speed for that */
uint8_t goto_and_avoid_backward(int16_t x, int16_t y, uint8_t flags_intermediate,
		       uint8_t flags_final)
{
	return __goto_and_avoid(x, y, flags_intermediate, flags_final, 0);
}

/* go to a x,y point. prefer backward but go forward if the point is
 * near and in front of us */
uint8_t goto_and_avoid(int16_t x, int16_t y, uint8_t flags_intermediate,
			       uint8_t flags_final)
{
	double d,a;
	abs_xy_to_rel_da(x, y, &d, &a); 

	if (d < 300 && a < RAD(90) && a > RAD(-90))
		return __goto_and_avoid(x, y, flags_intermediate,
					flags_final, 1);
	else
		return __goto_and_avoid(x, y, flags_intermediate,
					flags_final, 0);
}
