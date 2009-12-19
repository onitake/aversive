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
 *  Revision : $Id: strat_base.h,v 1.3 2009-05-02 10:08:09 zer0 Exp $
 *
 */

/* return values for strats and sub trajs */
#define END_TRAJ       1 /* traj successful */
#define END_BLOCKING   2 /* blocking during traj */
#define END_NEAR       4 /* we are near destination */
#define END_OBSTACLE   8 /* There is an obstacle in front of us */
#define END_ERROR     16 /* Cannot do the command */
#define END_INTR      32 /* interrupted by user */
#define END_TIMER     64 /* we don't a lot of time */
#define END_RESERVED 128 /* reserved */

/* default speeds */
#define SPEED_DIST_FAST 2500
#define SPEED_ANGLE_FAST 2000
#define SPEED_DIST_SLOW 1000
#define SPEED_ANGLE_SLOW 1000
#define SPEED_DIST_VERY_SLOW 400
#define SPEED_ANGLE_VERY_SLOW 400

/* useful traj flags */
#define TRAJ_SUCCESS(f) (f & (END_TRAJ|END_NEAR))
#define TRAJ_FLAGS_STD (END_TRAJ|END_BLOCKING|END_NEAR|END_OBSTACLE|END_INTR|END_TIMER)
#define TRAJ_FLAGS_NO_TIMER (END_TRAJ|END_BLOCKING|END_NEAR|END_OBSTACLE|END_INTR)
#define TRAJ_FLAGS_NO_NEAR (END_TRAJ|END_BLOCKING|END_OBSTACLE|END_INTR|END_TIMER)
#define TRAJ_FLAGS_NO_NEAR_NO_TIMER (END_TRAJ|END_BLOCKING|END_OBSTACLE|END_INTR)
#define TRAJ_FLAGS_SMALL_DIST (END_TRAJ|END_BLOCKING|END_INTR)

/* area */
#define AREA_X 3000
#define AREA_Y 2100

#define START_X 200
#define START_Y 200
#define START_A 45

#define CENTER_X 1500
#define CENTER_Y 1050

#define CORNER_X 3000
#define CORNER_Y 2100

/* only valid after a END_OBSTACLE */
struct opponent_obstacle {
	int16_t x;
	int16_t y;
	int16_t a;
	int16_t d;
};
extern struct opponent_obstacle opponent_obstacle;

/* stop as fast as possible, without ramp */
void strat_hardstop(void);

#define DO_NOT_SET_POS -1000
/* Reset position. If arg == DO_NOT_SET_POS, don't update value for
 * it. */
void strat_reset_pos(int16_t x, int16_t y, int16_t a);

/* decrease gain on angle PID, and go forward until we reach the
 * border. */
uint8_t strat_calib(int16_t d, uint8_t flags);

/* launch start procedure */
void strat_start(void);

/* go to an x,y point without checking for obstacle or blocking. It
 * should be used for very small dist only. Return END_TRAJ if we
 * reach destination, or END_BLOCKING if the robot blocked more than 3
 * times. */
uint8_t strat_goto_xy_force(int16_t x, int16_t y);

/* escape from disc polygon or another zone */
struct build_zone;
uint8_t strat_escape(struct build_zone *zone, uint8_t flags);

/* return true if we have to brake due to an obstacle */
uint8_t strat_obstacle(void);

/* set/get user strat speed */
void strat_set_speed(uint16_t d, uint16_t a);
void strat_get_speed(uint16_t *d, uint16_t *a);

/* when user type ctrl-c we can interrupt traj */
void interrupt_traj(void);
void interrupt_traj_reset(void);

/* limit speed when we are close of opponent */
void strat_limit_speed_enable(void);
void strat_limit_speed_disable(void);
void strat_limit_speed(void);

/* get name of traj error with its number */
const char *get_err(uint8_t err);

/* test trajectory end condition */
uint8_t test_traj_end(uint8_t why);

/* loop until test_traj_end() is true */
#define wait_traj_end(why) __wait_traj_end_debug(why, __LINE__)
uint8_t __wait_traj_end_debug(uint8_t why, uint16_t line);
