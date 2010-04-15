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

/*
 *
 *
 *           vertical lines
 *            O     1     2     3     4     5
 * 2100 +-----|-----|-----|-----|-----|-----|-----+
 *      |        o           o           o        |
 *      |  o           o           o           o  |   diag
 *      |        o           o           o        |   lines
 *     0/  o           o           o           o  \0
 *  y   |        o                       o        |
 *     1/  o                                   o  \1
 *      |                                         |
 *     2/------                             ------\2
 *      |     |                             |     |
 *      |     |                             |     |
 *   0  +-----+-----------------------------+-----+
 *     0                  x                      3000
 */

/* useful traj flags */
#define TRAJ_SUCCESS(f) (f & (END_TRAJ|END_NEAR))
#define TRAJ_FLAGS_STD (END_TRAJ|END_BLOCKING|END_NEAR|END_OBSTACLE|END_INTR|END_TIMER)
#define TRAJ_FLAGS_NO_TIMER (END_TRAJ|END_BLOCKING|END_NEAR|END_OBSTACLE|END_INTR)
#define TRAJ_FLAGS_NO_NEAR (END_TRAJ|END_BLOCKING|END_OBSTACLE|END_INTR|END_TIMER)
#define TRAJ_FLAGS_NO_NEAR_NO_TIMER (END_TRAJ|END_BLOCKING|END_OBSTACLE|END_INTR)
#define TRAJ_FLAGS_SMALL_DIST (END_TRAJ|END_BLOCKING|END_INTR)

/* default acc */
#define ACC_DIST  10.
#define ACC_ANGLE 10.

/* default speeds */
#define SPEED_DIST_FAST 2500.
#define SPEED_ANGLE_FAST 2000.
#define SPEED_DIST_SLOW 1000.
#define SPEED_ANGLE_SLOW 1000.
#define SPEED_DIST_VERY_SLOW 400.
#define SPEED_ANGLE_VERY_SLOW 400.

/* strat infos structures */

struct strat_bbox {
	int32_t x1;
	int32_t y1;
	int32_t x2;
	int32_t y2;
};

struct strat_conf {
#define STRAT_CONF_XXX   0x01
	uint8_t flags;
};

struct strat_status {
#define STRAT_STATUS_LHARVEST 0x01
#define STRAT_STATUS_RHARVEST 0x02
	uint8_t flags;
};

/* all infos related to strat */
struct strat_infos {
	uint8_t dump_enabled;
	struct strat_conf conf;
	struct strat_bbox area_bbox;
	struct strat_status status;
};
extern struct strat_infos strat_infos;

/* in strat.c */
void strat_dump_infos(const char *caller); /* show current known state
					      of area */
void strat_dump_conf(void);
void strat_reset_infos(void); /* reset current known state */
void strat_preinit(void);
void strat_init(void);
void strat_exit(void);
void strat_dump_flags(void);
void strat_goto_near(int16_t x, int16_t y, uint16_t dist);
uint8_t strat_main(void);
void strat_event(void *dummy);

#endif
