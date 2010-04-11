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
 *  Revision : $Id: strat_utils.h,v 1.5 2009-11-08 17:24:33 zer0 Exp $
 *
 */


struct xy_point {
	int16_t x;
	int16_t y;
};

/* wait traj end flag or cond. return 0 if cond become true, else
 * return the traj flag */
#define WAIT_COND_OR_TRAJ_END(cond, mask)				\
	({								\
		uint8_t __err = 0;					\
		while ( (! (cond)) && (__err == 0)) {			\
			__err = test_traj_end(TRAJ_FLAGS_NO_NEAR);	\
		}							\
		__err;							\
	})								\

int16_t distance_between(int16_t x1, int16_t y1, int16_t x2, int16_t y2);
int16_t distance_from_robot(int16_t x, int16_t y);
int16_t simple_modulo_360(int16_t a);
int16_t angle_abs_to_rel(int16_t a_abs);
void rel_da_to_abs_xy(double d_rel, double a_rel_rad, double *x_abs, double *y_abs);
double norm(double x, double y);
void rel_xy_to_abs_xy(double x_rel, double y_rel, double *x_abs, double *y_abs);
void abs_xy_to_rel_da(double x_abs, double y_abs, double *d_rel, double *a_rel_rad);
void rotate(double *x, double *y, double rot);
uint8_t is_in_area(int16_t x, int16_t y, int16_t margin);
uint8_t robot_is_in_area(int16_t margin);
uint8_t robot_is_near_disc(void);
uint8_t y_is_more_than(int16_t y);
uint8_t x_is_more_than(int16_t x);
int16_t fast_sin(int16_t deg);
int16_t fast_cos(int16_t deg);
uint8_t get_color(void);
uint8_t get_opponent_color(void);
int8_t get_opponent_xy(int16_t *x, int16_t *y);
int8_t get_opponent_da(int16_t *d, int16_t *a);
int8_t get_opponent_xyda(int16_t *x, int16_t *y, int16_t *d, int16_t *a);
uint8_t opponent_is_behind(void);
