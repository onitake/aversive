/*  
 *  Copyright Droids Corporation, Microb Technology, Eirbot (2008)
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
 *  Revision : $Id: strat.h,v 1.7 2009-11-08 17:24:33 zer0 Exp $
 *
 */

#ifndef _STRAT_H_
#define _STRAT_H_

/* convert coords according to our color */
#define COLOR_Y(y)     ((mainboard.our_color==I2C_COLOR_YELLOW)? (y) : (AREA_Y-(y)))
#define COLOR_A(a)     ((mainboard.our_color==I2C_COLOR_YELLOW)? (a) : (-a))
#define COLOR_SIGN(x)  ((mainboard.our_color==I2C_COLOR_YELLOW)? (x) : (-x))
#define COLOR_INVERT(x) ((mainboard.our_color==I2C_COLOR_YELLOW)? (x) : (!x))

/* area */
#define AREA_X 3000
#define AREA_Y 2100

#define START_X 200
#define START_Y COLOR_Y(200)
#define START_A COLOR_A(45)

#define CENTER_X 1500
#define CENTER_Y 1050

#define CORNER_X 3000
#define CORNER_Y COLOR_Y(2100)

/* XXX these offset are not related to corn, but to waypoints */
#define OFFSET_CORN_X 150
#define OFFSET_CORN_Y 222
#define STEP_CORN_X 225
#define STEP_CORN_Y 250

#define CORN_NB 18
#define TOMATO_NB 14

#define WAYPOINTS_NBX 13
#define WAYPOINTS_NBY 8

/*
 * Corn position and lines, valid for YELLOW.
 *
 *           vertical lines
 *            O     1     2     3     4     5
 * 2100 +-----|-----|-----|-----|-----|-----|-----+
 *      |        c5          c9         c14       |
 *      |  c2          c7         c11         c17 |   diag
 *      |        c4          c8         c13       |   lines
 *     0/  c1          c6         c10         c16 \0
 *  y   |        c3                     c12       |
 *     1/  c0                                 c15 \1
 *      |-----+                             +-----|
 *     2/     |                             |     \2
 *      | Yellow                           Blue   |
 *   0  +-----+-----------------------------+-----+
 *     0                  x                      3000
 *
 * Ball (tomato) and i,j coords (for waypoints)
 *
 * 2100 +--0--1--2--3--4--5--6--7--8--9-10-11-12--+
 *      7              t5          t9             |
 *      6        t3          t7         t11       |
 *      5  t1          t4          t8         t13 |
 *      4        t2          t6         t10       |
 *  y   3  t0                                 t12 |
 *      2                                         |
 *      1-----+                             +-----|
 *      0     |                             |     |
 *      | Yellow                           Blue   |
 *   0  +-----+-----------------------------+-----+
 *     0                  x                      3000
 *
 *
 * Corn position and lines, valid for BLUE.
 *
 *           vertical lines
 *            5     4     3     2     1     0
 *    0 +-----|-----|-----|-----|-----|-----|-----+
 *      |        c14         c9         c5        |
 *      |  c17         c11        c7          c2  |   diag
 *      |        c13         c8         c4        |   lines
 *     0/  c16         c10        c6          c1  \0
 *  y   |        c12                    c3        |
 *     1/  c15                                c0  \1
 *      |-----+                             +-----|
 *     2/     |                             |     \2
 *      | Yellow                           Blue   |
 * 2100 +-----+-----------------------------+-----+
 *    3000                x                       0
 *
 * Ball (tomato) and i,j coords (for waypoints)
 *
 *    0 +-12-11-10--9--8--7--6--5--4--3--2--1--0--+
 *      7              t9          t5             |
 *      6       t11          t7         t3        |
 *      5 t13          t8          t4         t1  |
 *      4       t10          t6         t2        |
 *  y   3 t12                                 t0  |
 *      2                                         |
 *      1-----+                             +-----|
 *      0     |                             |     |
 *      | Yellow                           Blue   |
 * 2100 +-----+-----------------------------+-----+
 *   3000                 x                       0
 *
 */

/* useful traj flags */
#define TRAJ_SUCCESS(f) (f & (END_TRAJ|END_NEAR))
#define TRAJ_FLAGS_STD (END_TRAJ|END_BLOCKING|END_NEAR|END_OBSTACLE|END_INTR|END_TIMER)
#define TRAJ_FLAGS_NO_TIMER (END_TRAJ|END_BLOCKING|END_NEAR|END_OBSTACLE|END_INTR)
#define TRAJ_FLAGS_NO_NEAR (END_TRAJ|END_BLOCKING|END_OBSTACLE|END_INTR|END_TIMER)
#define TRAJ_FLAGS_NO_NEAR_NO_TIMER (END_TRAJ|END_BLOCKING|END_OBSTACLE|END_INTR)
#define TRAJ_FLAGS_SMALL_DIST (END_TRAJ|END_BLOCKING|END_INTR)

/* default acc */
#define ACC_DIST  16.
#define ACC_ANGLE 16.

/* default speeds */
#define SPEED_DIST_FAST 1500.
#define SPEED_ANGLE_FAST 1000.
#define SPEED_DIST_SLOW 500.
#define SPEED_ANGLE_SLOW 500.
#define SPEED_DIST_VERY_SLOW 200.
#define SPEED_ANGLE_VERY_SLOW 200.

#define SPEED_CLITOID_SLOW 250.
#define SPEED_CLITOID_FAST 500.


/* strat infos structures */
struct strat_conf {
	uint8_t dump_enabled;

#define STRAT_CONF_XXX   0x01
	uint8_t flags;
};

extern struct strat_conf strat_conf;
extern volatile uint8_t strat_lpack60;
extern volatile uint8_t strat_rpack60;
extern volatile uint8_t strat_want_pack;

/* in strat.c */
void strat_conf_dump(const char *caller);
void strat_reset_infos(void); /* reset current known state */
void strat_preinit(void);
void strat_init(void);
void strat_exit(void);
void strat_dump_flags(void);
void strat_goto_near(int16_t x, int16_t y, uint16_t dist);
uint8_t strat_main(void);
void strat_event(void *dummy);
void strat_event_enable(void);
void strat_event_disable(void);

#endif
