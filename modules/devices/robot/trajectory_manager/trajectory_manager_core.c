/*
 *  Copyright Droids Corporation, Microb Technology, Eirbot (2005)
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
 *  Revision : $Id: trajectory_manager.c,v 1.4.4.17 2009-05-18 12:28:36 zer0 Exp $
 *
 */

/* Trajectory Manager v3 - zer0 - for Eurobot 2010 */

#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <aversive.h>
#include <aversive/error.h>
#include <scheduler.h>
#include <vect2.h>

#include <position_manager.h>
#include <robot_system.h>
#include <control_system_manager.h>
#include <quadramp.h>

#include <trajectory_manager.h>
#include "trajectory_manager_utils.h"
#include "trajectory_manager_core.h"

/************ SIMPLE TRAJS, NO EVENT */

#define UPDATE_D 1
#define UPDATE_A 2
#define RESET_D  4
#define RESET_A  8

static uint8_t evt_debug_cpt = 0;
#define EVT_DEBUG(args...) do {				\
		if (((evt_debug_cpt ++) & 0x07) == 0) { \
			DEBUG(args);			\
		}					\
	} while (0)


/**
 * update angle and/or distance
 * this function is not called directly by the user
 *   traj  : pointer to the trajectory structure
 *   d_mm  : distance in mm
 *   a_rad : angle in radian
 *   flags : what to update (UPDATE_A, UPDATE_D)
 */
void __trajectory_goto_d_a_rel(struct trajectory *traj, double d_mm,
			       double a_rad, uint8_t state, uint8_t flags)
{
	int32_t a_consign, d_consign;

	DEBUG(E_TRAJECTORY, "Goto DA/RS rel to d=%f a_rad=%f", d_mm, a_rad);
	delete_event(traj);
	traj->state = state;
	if (flags & UPDATE_A) {
		if (flags & RESET_A) {
			a_consign = 0;
		}
		else {
			a_consign = (int32_t)(a_rad * (traj->position->phys.distance_imp_per_mm) *
					      (traj->position->phys.track_mm) / 2);
		}
		a_consign +=  rs_get_angle(traj->robot);
		traj->target.pol.angle = a_consign;
		cs_set_consign(traj->csm_angle, a_consign);
	}
	if (flags & UPDATE_D) {
		if (flags & RESET_D) {
			d_consign = 0;
		}
		else {
			d_consign = (int32_t)((d_mm) * (traj->position->phys.distance_imp_per_mm));
		}
		d_consign += rs_get_distance(traj->robot);
		traj->target.pol.distance = d_consign;
		cs_set_consign(traj->csm_distance, d_consign);
	}
}

/** go straight forward (d is in mm) */
void trajectory_d_rel(struct trajectory *traj, double d_mm)
{
	__trajectory_goto_d_a_rel(traj, d_mm, 0, RUNNING_D,
				  UPDATE_D | UPDATE_A | RESET_A);
}

/** update distance consign without changing angle consign */
void trajectory_only_d_rel(struct trajectory *traj, double d_mm)
{
	__trajectory_goto_d_a_rel(traj, d_mm, 0, RUNNING_D, UPDATE_D);
}

/** turn by 'a' degrees */
void trajectory_a_rel(struct trajectory *traj, double a_deg_rel)
{
	__trajectory_goto_d_a_rel(traj, 0, RAD(a_deg_rel), RUNNING_A,
				  UPDATE_A | UPDATE_D | RESET_D);
}

/** turn by 'a' degrees */
void trajectory_a_abs(struct trajectory *traj, double a_deg_abs)
{
	double posa = position_get_a_rad_double(traj->position);
	double a;

	a = RAD(a_deg_abs) - posa;
	a = modulo_2pi(a);
	__trajectory_goto_d_a_rel(traj, 0, a, RUNNING_A,
				  UPDATE_A | UPDATE_D | RESET_D);
}

/** turn the robot until the point x,y is in front of us */
void trajectory_turnto_xy(struct trajectory *traj, double x_abs_mm, double y_abs_mm)
{
	double posx = position_get_x_double(traj->position);
	double posy = position_get_y_double(traj->position);
	double posa = position_get_a_rad_double(traj->position);

	DEBUG(E_TRAJECTORY, "Goto Turn To xy %f %f", x_abs_mm, y_abs_mm);
	__trajectory_goto_d_a_rel(traj, 0,
			simple_modulo_2pi(atan2(y_abs_mm - posy, x_abs_mm - posx) - posa),
				  RUNNING_A,
				  UPDATE_A | UPDATE_D | RESET_D);
}

/** turn the robot until the point x,y is behind us */
void trajectory_turnto_xy_behind(struct trajectory *traj, double x_abs_mm, double y_abs_mm)
{
	double posx = position_get_x_double(traj->position);
	double posy = position_get_y_double(traj->position);
	double posa = position_get_a_rad_double(traj->position);

	DEBUG(E_TRAJECTORY, "Goto Turn To xy %f %f", x_abs_mm, y_abs_mm);
	__trajectory_goto_d_a_rel(traj, 0,
			modulo_2pi(atan2(y_abs_mm - posy, x_abs_mm - posx) - posa + M_PI),
				  RUNNING_A,
				  UPDATE_A | UPDATE_D | RESET_D);
}

/** update angle consign without changing distance consign */
void trajectory_only_a_rel(struct trajectory *traj, double a_deg)
{
	__trajectory_goto_d_a_rel(traj, 0, RAD(a_deg), RUNNING_A,
				  UPDATE_A);
}

/** update angle consign without changing distance consign */
void trajectory_only_a_abs(struct trajectory *traj, double a_deg_abs)
{
	double posa = position_get_a_rad_double(traj->position);
	double a;

	a = RAD(a_deg_abs) - posa;
	a = modulo_2pi(a);
	__trajectory_goto_d_a_rel(traj, 0, a, RUNNING_A, UPDATE_A);
}

/** turn by 'a' degrees */
void trajectory_d_a_rel(struct trajectory *traj, double d_mm, double a_deg)
{
	__trajectory_goto_d_a_rel(traj, d_mm, RAD(a_deg),
				  RUNNING_AD, UPDATE_A | UPDATE_D);
}

/** set relative angle and distance consign to 0 */
void trajectory_stop(struct trajectory *traj)
{
	__trajectory_goto_d_a_rel(traj, 0, 0, READY,
				  UPDATE_A | UPDATE_D | RESET_D | RESET_A);
}

/** set relative angle and distance consign to 0, and break any
 * deceleration ramp in quadramp filter */
void trajectory_hardstop(struct trajectory *traj)
{
	struct quadramp_filter *q_d, *q_a;

	q_d = traj->csm_distance->consign_filter_params;
	q_a = traj->csm_angle->consign_filter_params;
	__trajectory_goto_d_a_rel(traj, 0, 0, READY,
				  UPDATE_A | UPDATE_D | RESET_D | RESET_A);

	q_d->previous_var = 0;
	q_d->previous_out = rs_get_distance(traj->robot);
	q_a->previous_var = 0;
	q_a->previous_out = rs_get_angle(traj->robot);
}


/************ GOTO XY, USE EVENTS */

/** goto a x,y point, using a trajectory event */
void trajectory_goto_xy_abs(struct trajectory *traj, double x, double y)
{
	DEBUG(E_TRAJECTORY, "Goto XY");
	delete_event(traj);
	traj->target.cart.x = x;
	traj->target.cart.y = y;
	traj->state = RUNNING_XY_START;
	trajectory_manager_event(traj);
	schedule_event(traj);
}

/** go forward to a x,y point, using a trajectory event */
void trajectory_goto_forward_xy_abs(struct trajectory *traj, double x, double y)
{
	DEBUG(E_TRAJECTORY, "Goto XY_F");
	delete_event(traj);
	traj->target.cart.x = x;
	traj->target.cart.y = y;
	traj->state = RUNNING_XY_F_START;
	trajectory_manager_event(traj);
	schedule_event(traj);
}

/** go backward to a x,y point, using a trajectory event */
void trajectory_goto_backward_xy_abs(struct trajectory *traj, double x, double y)
{
	DEBUG(E_TRAJECTORY, "Goto XY_B");
	delete_event(traj);
	traj->target.cart.x = x;
	traj->target.cart.y = y;
	traj->state = RUNNING_XY_B_START;
	trajectory_manager_event(traj);
	schedule_event(traj);
}

/** go forward to a d,a point, using a trajectory event */
void trajectory_goto_d_a_rel(struct trajectory *traj, double d, double a)
{
	vect2_pol p;
	double x = position_get_x_double(traj->position);
	double y = position_get_y_double(traj->position);

	DEBUG(E_TRAJECTORY, "Goto DA rel");

	delete_event(traj);
	p.r = d;
	p.theta = RAD(a) + position_get_a_rad_double(traj->position);
	vect2_pol2cart(&p, &traj->target.cart);
	traj->target.cart.x += x;
	traj->target.cart.y += y;

	traj->state = RUNNING_XY_START;
	trajectory_manager_event(traj);
	schedule_event(traj);
}

/** go forward to a x,y relative point, using a trajectory event */
void trajectory_goto_xy_rel(struct trajectory *traj, double x_rel_mm, double y_rel_mm)
{
	vect2_cart c;
	vect2_pol p;
	double x = position_get_x_double(traj->position);
	double y = position_get_y_double(traj->position);

	DEBUG(E_TRAJECTORY, "Goto XY rel");

	delete_event(traj);
	c.x = x_rel_mm;
	c.y = y_rel_mm;

	vect2_cart2pol(&c, &p);
	p.theta += position_get_a_rad_double(traj->position);;
	vect2_pol2cart(&p, &traj->target.cart);

	traj->target.cart.x += x;
	traj->target.cart.y += y;

	traj->state = RUNNING_XY_START;
	trajectory_manager_event(traj);
	schedule_event(traj);
}

/************ FUNCS FOR GETTING TRAJ STATE */

/** return true if the position consign is equal to the filtered
 * position consign (after quadramp filter), for angle and
 * distance. */
uint8_t trajectory_finished(struct trajectory *traj)
{
	return cs_get_consign(traj->csm_angle) == cs_get_filtered_consign(traj->csm_angle) &&
		cs_get_consign(traj->csm_distance) == cs_get_filtered_consign(traj->csm_distance) ;
}

/** return true if traj is nearly finished */
uint8_t trajectory_in_window(struct trajectory *traj, double d_win, double a_win_rad)
{
	switch(traj->state) {

	case RUNNING_XY_ANGLE_OK:
	case RUNNING_XY_F_ANGLE_OK:
	case RUNNING_XY_B_ANGLE_OK:
		/* if robot coordinates are near the x,y target */
		return is_robot_in_xy_window(traj, d_win);

	case RUNNING_A:
		return is_robot_in_angle_window(traj, a_win_rad);

	case RUNNING_D:
		return is_robot_in_dist_window(traj, d_win);

	case RUNNING_AD:
		return is_robot_in_dist_window(traj, d_win) &&
			is_robot_in_angle_window(traj, a_win_rad);

	case RUNNING_XY_START:
	case RUNNING_XY_F_START:
	case RUNNING_XY_B_START:
	case RUNNING_XY_ANGLE:
	case RUNNING_XY_F_ANGLE:
	case RUNNING_XY_B_ANGLE:
	default:
		return 0;
	}
}

/*********** *TRAJECTORY EVENT FUNC */

/** event called for xy trajectories */
void trajectory_manager_xy_event(struct trajectory *traj)
{
	double coef = 1.0;
	double x = position_get_x_double(traj->position);
	double y = position_get_y_double(traj->position);
	double a = position_get_a_rad_double(traj->position);
	int32_t d_consign=0, a_consign=0;

	/* These vectors contain target position of the robot in
	 * its own coordinates */
	vect2_cart v2cart_pos;
	vect2_pol v2pol_target;

	/* step 1 : process new commands to quadramps */

	switch (traj->state) {
	case RUNNING_XY_START:
	case RUNNING_XY_ANGLE:
	case RUNNING_XY_ANGLE_OK:
	case RUNNING_XY_F_START:
	case RUNNING_XY_F_ANGLE:
	case RUNNING_XY_F_ANGLE_OK:
	case RUNNING_XY_B_START:
	case RUNNING_XY_B_ANGLE:
	case RUNNING_XY_B_ANGLE_OK:

		/* process the command vector from current position to
		 * absolute target. */
		v2cart_pos.x = traj->target.cart.x - x;
		v2cart_pos.y = traj->target.cart.y - y;
		vect2_cart2pol(&v2cart_pos, &v2pol_target);
		v2pol_target.theta = simple_modulo_2pi(v2pol_target.theta - a);

		/* asked to go backwards */
		if (traj->state >= RUNNING_XY_B_START &&
		    traj->state <= RUNNING_XY_B_ANGLE_OK ) {
			v2pol_target.r = -v2pol_target.r;
			v2pol_target.theta = simple_modulo_2pi(v2pol_target.theta + M_PI);
		}

		/* if we don't need to go forward */
		if (traj->state >= RUNNING_XY_START &&
		    traj->state <= RUNNING_XY_ANGLE_OK ) {
			/* If the target is behind the robot, we need to go
			 * backwards. 0.52 instead of 0.5 because we prefer to
			 * go forward */
			if ((v2pol_target.theta > 0.52*M_PI) ||
			    (v2pol_target.theta < -0.52*M_PI ) ) {
				v2pol_target.r = -v2pol_target.r;
				v2pol_target.theta = simple_modulo_2pi(v2pol_target.theta + M_PI);
			}
		}

		/* XXX circle */

		/* If the robot is correctly oriented to start moving in distance */
		/* here limit dist speed depending on v2pol_target.theta */
		if (ABS(v2pol_target.theta) > traj->a_start_rad) // || ABS(v2pol_target.r) < traj->d_win)
			set_quadramp_speed(traj, 0, traj->a_speed);
		else {
			coef = (traj->a_start_rad - ABS(v2pol_target.theta)) / traj->a_start_rad;
			set_quadramp_speed(traj, traj->d_speed * coef, traj->a_speed);
		}

		d_consign = (int32_t)(v2pol_target.r * (traj->position->phys.distance_imp_per_mm));
		d_consign += rs_get_distance(traj->robot);

		/* angle consign */
		/* XXX here we specify 2.2 instead of 2.0 to avoid oscillations */
		a_consign = (int32_t)(v2pol_target.theta *
				      (traj->position->phys.distance_imp_per_mm) *
				      (traj->position->phys.track_mm) / 2.2);
		a_consign += rs_get_angle(traj->robot);

		break;

	default:
		/* hmmm quite odd, delete the event */
		DEBUG(E_TRAJECTORY, "GNI ???");
		delete_event(traj);
		traj->state = READY;
	}


	/* step 2 : update state, or delete event if we reached the
	 * destination */

	/* XXX if target is our pos !! */

	switch (traj->state) {
	case RUNNING_XY_START:
	case RUNNING_XY_F_START:
	case RUNNING_XY_B_START:
		/* START -> ANGLE */
		DEBUG(E_TRAJECTORY, "-> ANGLE");
		traj->state ++;
		break;

	case RUNNING_XY_ANGLE:
	case RUNNING_XY_F_ANGLE:
	case RUNNING_XY_B_ANGLE: {
		struct quadramp_filter *q_a;
		q_a = traj->csm_angle->consign_filter_params;
		/* if d_speed is not 0, we are in start_angle_win */
		if (get_quadramp_distance_speed(traj)) {
			if (is_robot_in_xy_window(traj, traj->d_win)) {
				delete_event(traj);
			}
			/* ANGLE -> ANGLE_OK */
			traj->state ++;
			DEBUG(E_TRAJECTORY, "-> ANGLE_OK");
		}
		break;
	}

	case RUNNING_XY_ANGLE_OK:
	case RUNNING_XY_F_ANGLE_OK:
	case RUNNING_XY_B_ANGLE_OK:
		/* If we reached the destination */
		if (is_robot_in_xy_window(traj, traj->d_win)) {
			delete_event(traj);
		}
	break;

	default:
		break;
	}

	/* step 3 : send the processed commands to cs */

	EVT_DEBUG(E_TRAJECTORY,"EVENT XY d_cur=%" PRIi32 ", d_consign=%" PRIi32 ", d_speed=%" PRIi32 ", "
	      "a_cur=%" PRIi32 ", a_consign=%" PRIi32 ", a_speed=%" PRIi32,
	      rs_get_distance(traj->robot), d_consign, get_quadramp_distance_speed(traj),
	      rs_get_angle(traj->robot), a_consign, get_quadramp_angle_speed(traj));

	cs_set_consign(traj->csm_angle, a_consign);
	cs_set_consign(traj->csm_distance, d_consign);
}

/* trajectory event for circles */
/* XXX static */
void trajectory_manager_circle_event(struct trajectory *traj)
{
	double coef = 1.0;
	double x = position_get_x_double(traj->position);
	double y = position_get_y_double(traj->position);
	double a = position_get_a_rad_double(traj->position);
	int32_t d_consign = 0, a_consign = 0;
	double angle_to_center_rad;

	/* These vectors contain target position of the robot in
	 * its own coordinates */
	vect2_cart v2cart_pos;
	vect2_pol v2pol_target;

	/* step 1 : process new commands to quadramps */

	/* process the command vector from current position to the
	 * center of the circle. */
	v2cart_pos.x = traj->target.circle.center.x - x;
	v2cart_pos.y = traj->target.circle.center.y - y;
	vect2_cart2pol(&v2cart_pos, &v2pol_target);
	v2pol_target.theta = simple_modulo_2pi(v2pol_target.theta - a);

	/* wanted direction of center of circle:
	 * if we are far, go in the center direction,
	 * if we are at radius, we want to see the center at 90°
	 * if we are nearer than radius, angle to center is > 90° */
	if (v2pol_target.r > traj->target.circle.radius) {
		angle_to_center_rad = traj->target.circle.radius / v2pol_target.r;
		angle_to_center_rad *= (M_PI / 2);
	}
	else {
		angle_to_center_rad = 1. - (v2pol_target.r /
					    (2 * traj->target.circle.radius));
		angle_to_center_rad *= M_PI;
	}

	/* XXX check flags */
	v2pol_target.theta -= angle_to_center_rad;

	/* If the robot is correctly oriented to start moving in distance */
	/* here limit dist speed depending on v2pol_target.theta */
	if (ABS(v2pol_target.theta) > traj->a_start_rad)
		set_quadramp_speed(traj, 0, traj->a_speed);
	else {
		coef = (traj->a_start_rad - ABS(v2pol_target.theta)) / traj->a_start_rad;
		set_quadramp_speed(traj, traj->d_speed * coef, traj->a_speed);
	}

	/* XXX check flags */
	d_consign = 40000 + rs_get_distance(traj->robot);

	/* angle consign */
	a_consign = (int32_t)(v2pol_target.theta *
			      (traj->position->phys.distance_imp_per_mm) *
			      (traj->position->phys.track_mm) / 2.0);
	a_consign += rs_get_angle(traj->robot);

	/* step 2 : update state, or delete event if we reached the
	 * destination */

	/* output angle -> delete event */
	if (a_consign >= traj->target.circle.dest_angle) {
		a_consign = traj->target.circle.dest_angle;
		delete_event(traj);
	}

	/* step 3 : send the processed commands to cs */

	EVT_DEBUG(E_TRAJECTORY,"EVENT CIRCLE d_cur=%" PRIi32 ", d_consign=%" PRIi32
		  ", d_speed=%" PRIi32 ", a_cur=%" PRIi32 ", a_consign=%" PRIi32
		  ", a_speed=%" PRIi32,
	      rs_get_distance(traj->robot), d_consign, get_quadramp_distance_speed(traj),
	      rs_get_angle(traj->robot), a_consign, get_quadramp_angle_speed(traj));

	cs_set_consign(traj->csm_angle, a_consign);
	cs_set_consign(traj->csm_distance, d_consign);
}

/* trajectory event */
void trajectory_manager_event(void * param)
{
	struct trajectory *traj = (struct trajectory *)param;

	switch (traj->state) {
	case RUNNING_XY_START:
	case RUNNING_XY_ANGLE:
	case RUNNING_XY_ANGLE_OK:
	case RUNNING_XY_F_START:
	case RUNNING_XY_F_ANGLE:
	case RUNNING_XY_F_ANGLE_OK:
	case RUNNING_XY_B_START:
	case RUNNING_XY_B_ANGLE:
	case RUNNING_XY_B_ANGLE_OK:
		trajectory_manager_xy_event(traj);
		break;

	case RUNNING_CIRCLE:
		trajectory_manager_circle_event(traj);
		break;

	default:
		break;
	}
}

/*********** *CIRCLE */

/* make the robot orbiting around (x,y) on a circle whose radius is
 * radius_mm, and exit when relative destination angle is reached. The
 * flags set if we go forward or backwards, and CW/CCW. */
void trajectory_circle_rel(struct trajectory *traj,
			   double x, double y,
			   double radius_mm,
			   double rel_a_deg,
			   uint8_t flags)
{
	double dst_angle;

	delete_event(traj);

	traj->target.circle.center.x = x;
	traj->target.circle.center.y = y;
	traj->target.circle.radius = radius_mm;
	traj->target.circle.flags = flags;

	/* convert in steps  */
	dst_angle = RAD(rel_a_deg) *
		(traj->position->phys.distance_imp_per_mm) *
		(traj->position->phys.track_mm) / 2.0;

	traj->target.circle.dest_angle = rs_get_angle(traj->robot);
	traj->target.circle.dest_angle += dst_angle;

	DEBUG(E_TRAJECTORY, "Circle rel (x,y)=%2.2f,%2.2f r=%2.2f flags=%x dst_angle=%"PRIi32"",
	      x, y, radius_mm, flags, traj->target.circle.dest_angle);

	traj->state = RUNNING_CIRCLE;
	trajectory_manager_event(traj);
	schedule_event(traj);
}

/*
 * Compute the fastest distance and angle speeds matching the radius
 * from current traj_speed
 */
/* static  */void circle_get_da_speed_from_radius(struct trajectory *traj,
						  double radius_mm,
						  double *speed_d,
						  double *speed_a)
{
	/* speed_d = coef * speed_a */
	double coef;
	double speed_d2, speed_a2;

	coef = 2. * radius_mm / traj->position->phys.track_mm;

	speed_d2 = traj->a_speed * coef;
	if (speed_d2 < traj->d_speed) {
		*speed_d = speed_d2;
		*speed_a = traj->a_speed;
	}
	else {
		speed_a2 = traj->d_speed / coef;
		*speed_d = traj->d_speed;
		*speed_a = speed_a2;
	}
}

/* return the distance in millimeters that corresponds to an angle in
 * degree and a radius in mm */
/* static  */double circle_get_dist_from_degrees(double radius_mm, double a_deg)
{
	double a_rad = RAD(a_deg);
	return a_rad * radius_mm;
}

/*
 * Start a circle of specified radius around the specified center
 * (relative with d,a). The distance is specified in mm.
 */
void trajectory_circle(struct trajectory *traj,
		       double center_d_mm, double center_a_rad,
		       double radius_mm, double dist_mm)
{
/* 	double */

/* 	DEBUG(E_TRAJECTORY, "CIRCLE to d=%f a_rad=%f", center_d_mm, */
/* 	      center_a_rad); */
/* 	delete_event(traj); */
/* 	traj->state = RUNNING_CIRCLE; */


}

/*
 * Start a circle of specified radius around the specified center
 * (absolute). The distance is specified in mm.
 */
void trajectory_circle_abs_dist_mm(struct trajectory *traj,
				   double x_rel_mm, double y_rel_mm,
				   double radius_mm, double dist_mm)
{
}

/*
 * Start a circle of specified radius around the specified center
 * (absolute). The distance is specified in degrees.
 */
void trajectory_circle_abs_dist_deg(struct trajectory *traj,
				    double x_rel_mm, double y_rel_mm,
				    double radius_mm, double dist_degrees)
{

}
