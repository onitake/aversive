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

/** set speed consign in quadramp filter */
void set_quadramp_speed(struct trajectory *traj, double d_speed, double a_speed)
{
	struct quadramp_filter * q_d, * q_a;
	q_d = traj->csm_distance->consign_filter_params;
	q_a = traj->csm_angle->consign_filter_params;
	quadramp_set_1st_order_vars(q_d, ABS(d_speed), ABS(d_speed));
	quadramp_set_1st_order_vars(q_a, ABS(a_speed), ABS(a_speed));
}

/** get angle speed consign in quadramp filter */
double get_quadramp_angle_speed(struct trajectory *traj)
{
	struct quadramp_filter *q_a;
	q_a = traj->csm_angle->consign_filter_params;
	return q_a->var_1st_ord_pos;
}

/** get distance speed consign in quadramp filter */
double get_quadramp_distance_speed(struct trajectory *traj)
{
	struct quadramp_filter *q_d;
	q_d = traj->csm_distance->consign_filter_params;
	return q_d->var_1st_ord_pos;
}

/** set speed consign in quadramp filter */
void set_quadramp_acc(struct trajectory *traj, double d_acc, double a_acc)
{
	struct quadramp_filter * q_d, * q_a;
	q_d = traj->csm_distance->consign_filter_params;
	q_a = traj->csm_angle->consign_filter_params;
	quadramp_set_2nd_order_vars(q_d, ABS(d_acc), ABS(d_acc));
	quadramp_set_2nd_order_vars(q_a, ABS(a_acc), ABS(a_acc));
}

/** remove event if any */
void delete_event(struct trajectory *traj)
{
	set_quadramp_speed(traj, traj->d_speed, traj->a_speed);
	set_quadramp_acc(traj, traj->d_acc, traj->a_acc);
	if ( traj->scheduler_task != -1) {
		DEBUG(E_TRAJECTORY, "Delete event");
		scheduler_del_event(traj->scheduler_task);
		traj->scheduler_task = -1;
	}
}

/** schedule the trajectory event */
void schedule_event(struct trajectory *traj)
{
	if ( traj->scheduler_task != -1) {
		DEBUG(E_TRAJECTORY, "Schedule event, already scheduled");
	}
	else {
		traj->scheduler_task =
			scheduler_add_periodical_event_priority(&trajectory_manager_event,
								(void*)traj,
								TRAJ_EVT_PERIOD, 30);
	}
}

/** do a modulo 2.pi -> [-Pi,+Pi], knowing that 'a' is in [-3Pi,+3Pi] */
double simple_modulo_2pi(double a)
{
	if (a < -M_PI) {
		a += M_2PI;
	}
	else if (a > M_PI) {
		a -= M_2PI;
	}
	return a;
}

/** do a modulo 2.pi -> [-Pi,+Pi] */
double modulo_2pi(double a)
{
        double res = a - (((int32_t) (a/M_2PI)) * M_2PI);
	return simple_modulo_2pi(res);
}

/** near the target (dist) ? */
uint8_t is_robot_in_dist_window(struct trajectory *traj, double d_win)
{
	double d = traj->target.pol.distance - rs_get_distance(traj->robot);
	d = ABS(d);
	d = d / traj->position->phys.distance_imp_per_mm;
	return (d < d_win);
}

/** near the target (dist in x,y) ? */
uint8_t is_robot_in_xy_window(struct trajectory *traj, double d_win)
{
	double x1 = traj->target.cart.x;
	double y1 = traj->target.cart.y;
	double x2 = position_get_x_double(traj->position);
	double y2 = position_get_y_double(traj->position);
	return ( sqrt ((x2-x1) * (x2-x1) + (y2-y1) * (y2-y1)) < d_win );
}

/** near the angle target in radian ? Only valid if
 *  traj->target.pol.angle is set (i.e. an angle command, not an xy
 *  command) */
uint8_t is_robot_in_angle_window(struct trajectory *traj, double a_win_rad)
{
	double a;

	/* convert relative angle from imp to rad */
	a = traj->target.pol.angle - rs_get_angle(traj->robot);
	a /= traj->position->phys.distance_imp_per_mm;
	a /= traj->position->phys.track_mm;
	a *= 2.;
	return ABS(a) < a_win_rad;
}

enum trajectory_state trajectory_get_state(struct trajectory *traj)
{
	return traj->state;
}

/* distance unit conversions */

double pos_mm2imp(struct trajectory *traj, double pos)
{
	return pos * traj->position->phys.distance_imp_per_mm;
}

double pos_imp2mm(struct trajectory *traj, double pos)
{
	return pos / traj->position->phys.distance_imp_per_mm;
}

double speed_mm2imp(struct trajectory *traj, double speed)
{
	return speed *
		traj->position->phys.distance_imp_per_mm /
		traj->cs_hz;
}

double speed_imp2mm(struct trajectory *traj, double speed)
{
	return speed * traj->cs_hz /
		traj->position->phys.distance_imp_per_mm;
}

double acc_mm2imp(struct trajectory *traj, double acc)
{
	return acc * traj->position->phys.distance_imp_per_mm /
		(traj->cs_hz * traj->cs_hz);
}

double acc_imp2mm(struct trajectory *traj, double acc)
{
	return acc * traj->cs_hz * traj->cs_hz /
		traj->position->phys.distance_imp_per_mm;
}

/* angle unit conversions */

double pos_rd2imp(struct trajectory *traj, double pos)
{
	return pos *
		(traj->position->phys.distance_imp_per_mm *
		 traj->position->phys.track_mm / 2.);
}

double pos_imp2rd(struct trajectory *traj, double pos)
{
	return pos /
		(traj->position->phys.distance_imp_per_mm *
		 traj->position->phys.track_mm / 2.);
}

double speed_rd2imp(struct trajectory *traj, double speed)
{
	return speed *
		(traj->position->phys.distance_imp_per_mm *
		 traj->position->phys.track_mm /
		 (2. * traj->cs_hz));
}

double speed_imp2rd(struct trajectory *traj, double speed)
{
	return speed /
		(traj->position->phys.distance_imp_per_mm *
		 traj->position->phys.track_mm /
		 (2. * traj->cs_hz));
}

double acc_rd2imp(struct trajectory *traj, double acc)
{
	return acc *
		(traj->position->phys.distance_imp_per_mm *
		 traj->position->phys.track_mm /
		 (2. * traj->cs_hz * traj->cs_hz));
}

double acc_imp2rd(struct trajectory *traj, double acc)
{
	return acc /
		(traj->position->phys.distance_imp_per_mm *
		 traj->position->phys.track_mm /
		 (2. * traj->cs_hz * traj->cs_hz));
}

