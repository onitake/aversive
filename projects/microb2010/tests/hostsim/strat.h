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
 *              /- line 0
 *             |   /- line 1
 *             |  |   /- line 2
 *             |  |  |
 *     +---C1--------------------------C1---+
 *     |          z0a   z1    z0b           |
 *     |                 .                  |
 *     |                 .                  |
 *    C3       0  1  2   ^                  C3
 *  y  |       3  4  5 <   >                |
 *    C2       6  7  8   v                  C2 
 *     |------ 9 10 11   .            ------| 
 *     |     |           .            |     | 
 *     | yellow |           .            |blue| 
 *     +-----+--L1--L2-------L2--L1---+-----+
 *                       x
 */

/* static columns */
#define LINE0_X 600
#define LINE1_X 850
#define LINE2_X 1100

#define COL0_X 600
#define COL0_Y COLOR_Y(1175)
#define COL1_X 850
#define COL1_Y COLOR_Y(1175)
#define COL2_X 1100
#define COL2_Y COLOR_Y(1175)

#define COL3_X 600
#define COL3_Y COLOR_Y(975)
#define COL4_X 850
#define COL4_Y COLOR_Y(975)
#define COL5_X 1100
#define COL5_Y COLOR_Y(975)

#define COL6_X 600
#define COL6_Y COLOR_Y(775)
#define COL7_X 850
#define COL7_Y COLOR_Y(775)
#define COL8_X 1100
#define COL8_Y COLOR_Y(775)

#define COL9_X 600
#define COL9_Y COLOR_Y(575)
#define COL10_X 850
#define COL10_Y COLOR_Y(575)
#define COL11_X 1100
#define COL11_Y COLOR_Y(575)

/* distance to go backward before pickup in dispenser */
#define DIST_BACK_DISPENSER 35

/* diag of the pentagon (pentacle ?) */
#define DISC_PENTA_DIAG 530

#define COL_DISP_MAX_TRIES 5
#define LIN_DISP_MAX_TRIES 3

/* useful traj flags */
#define TRAJ_SUCCESS(f) (f & (END_TRAJ|END_NEAR))
#define TRAJ_FLAGS_STD (END_TRAJ|END_BLOCKING|END_NEAR|END_OBSTACLE|END_INTR|END_TIMER)
#define TRAJ_FLAGS_NO_TIMER (END_TRAJ|END_BLOCKING|END_NEAR|END_OBSTACLE|END_INTR)
#define TRAJ_FLAGS_NO_NEAR (END_TRAJ|END_BLOCKING|END_OBSTACLE|END_INTR|END_TIMER)
#define TRAJ_FLAGS_NO_NEAR_NO_TIMER (END_TRAJ|END_BLOCKING|END_OBSTACLE|END_INTR)
#define TRAJ_FLAGS_SMALL_DIST (END_TRAJ|END_BLOCKING|END_INTR)

/* default speeds */
#define SPEED_DIST_FAST 2500
#define SPEED_ANGLE_FAST 2000
#define SPEED_DIST_SLOW 1000
#define SPEED_ANGLE_SLOW 1000
#define SPEED_DIST_VERY_SLOW 400
#define SPEED_ANGLE_VERY_SLOW 400

/* strat infos structures */

struct bbox {
	int32_t x1;
	int32_t y1;
	int32_t x2;
	int32_t y2;
};

struct conf {
#define STRAT_CONF_ONLY_ONE_ON_DISC      0x01
#define STRAT_CONF_BYPASS_STATIC2        0x02
#define STRAT_CONF_TAKE_ONE_LINTEL       0x04
#define STRAT_CONF_SKIP_WHEN_CHECK_FAILS 0x08
#define STRAT_CONF_STORE_STATIC2         0x10
#define STRAT_CONF_BIG_3_TEMPLE          0x20
#define STRAT_CONF_EARLY_SCAN            0x40
#define STRAT_CONF_PUSH_OPP_COLS         0x80
	uint8_t flags;
	uint8_t scan_opp_min_time;
	uint8_t delay_between_opp_scan;
	uint8_t scan_our_min_time;
	uint8_t delay_between_our_scan;
	uint8_t wait_opponent;
	uint8_t lintel_min_time;
	int16_t scan_opp_angle;
};

struct static_columns {
#define STATIC_COL_LINE0_DONE 0x01
#define STATIC_COL_LINE1_DONE 0x02
#define STATIC_COL_LINE2_DONE 0x04
	uint8_t flags;
	uint8_t configuration;
};

struct column_dispenser {
	int8_t count;
	uint8_t last_try_time;
	uint8_t scan_left;
	int16_t checkpoint_x;
	int16_t checkpoint_y;
	int16_t scan_a;
	int16_t eject_a;
	int16_t pickup_a;
	int16_t recalib_x;
	int16_t recalib_y;
	char *name;
};

struct lintel_dispenser {
	int8_t count;
	uint8_t last_try_time;
	int16_t x;
	char *name;
};

struct temple {
#define TEMPLE_F_VALID    0x01 /* structure is valid */
#define TEMPLE_F_MONOCOL  0x02 /* temple has only one col */
#define TEMPLE_F_ON_DISC  0x04 /* temple is on disc (else it's on other zone) */
#define TEMPLE_F_OPPONENT 0x08 /* temple was originally built by opponent */
#define TEMPLE_F_LINTEL   0x10 /* lintel on top (don't put another lintel) */

	uint8_t flags;
	/* position of the robot when we built it */
	int16_t x;
	int16_t y;
	int16_t a;

	/* position of the robot checkpoint */
	int16_t checkpoint_x;
	int16_t checkpoint_y;

	/* position and level of each col */
	uint8_t level_l;
	uint8_t dist_l;
	uint8_t angle_l;

	uint8_t level_r;
	uint8_t dist_r;
	uint8_t angle_r;

#define TEMPLE_DISABLE_TIME 5
	uint8_t last_try_time;

	struct build_zone *zone;
};

struct build_zone {
#define ZONE_F_VALID 0x01      /* zone is valid */
#define ZONE_F_DISC  0x02      /* specific disc zone */
#define ZONE_F_ZONE1 0x04      /* specific zone 1 */
#define ZONE_F_ZONE0 0x08      /* specific zone 0 */
#define ZONE_F_BUSY  0x10      /* this zone is busy */
	uint8_t flags;
	uint8_t level;
	int16_t checkpoint_x;
	int16_t checkpoint_y;

#define ZONE_DISABLE_TIME 5
	uint8_t last_try_time;
	char *name;
};

#define MAX_TEMPLE 5
#define MAX_ZONE 5

/* all infos related to strat */
struct strat_infos {
	uint8_t dump_enabled;
	struct conf conf;
	struct bbox area_bbox;
	uint8_t taken_lintel;
	uint8_t col_in_boobs;
	uint8_t lazy_pickup_done;
	uint8_t i2c_loaded_skipped;
	struct static_columns s_cols;
	struct column_dispenser c1;
	struct column_dispenser c2;
	struct column_dispenser c3;
	struct lintel_dispenser l1;
	struct lintel_dispenser l2;
	struct build_zone zone_list[MAX_ZONE];
	struct temple temple_list[MAX_TEMPLE];
};
extern struct strat_infos strat_infos;

/* in strat.c */
void strat_dump_infos(const char *caller); /* show current known state
					      of area */
void strat_dump_temple(struct temple *temple);
void strat_dump_conf(void);
void strat_reset_infos(void); /* reset current known state */
void strat_preinit(void);
void strat_init(void);
void strat_exit(void);
void strat_dump_flags(void);
void strat_goto_near(int16_t x, int16_t y, uint16_t dist);
uint8_t strat_main(void);
void strat_event(void *dummy);

/* in strat_static_columns.c */
uint8_t strat_static_columns(uint8_t configuration);
uint8_t strat_static_columns_pass2(void);

/* in strat_lintel.c */
uint8_t strat_goto_lintel_disp(struct lintel_dispenser *disp);
uint8_t strat_pickup_lintels(void);

/* in strat_column_disp.c */
uint8_t strat_eject_col(int16_t eject_a, int16_t pickup_a);
uint8_t strat_pickup_columns(void);
uint8_t strat_goto_col_disp(struct column_dispenser **disp);

/* in strat_building.c */
uint8_t strat_goto_disc(int8_t level);
uint8_t strat_goto_build_zone(struct build_zone *build_zone, uint8_t level);
uint8_t strat_build_new_temple(struct build_zone *build_zone);
uint8_t strat_goto_temple(struct temple *temple);
uint8_t strat_grow_temple(struct temple *temple);
uint8_t strat_grow_temple_column(struct temple *temple);
struct temple *strat_get_best_temple(void);
struct temple *strat_get_our_temple_on_disc(uint8_t valid);
struct build_zone *strat_get_best_zone(void);
struct temple *strat_get_free_temple(void);

/* in strat_scan.c */
struct scan_disc_result;
void scanner_dump_state(void);
int8_t strat_scan_get_checkpoint(uint8_t mode, int16_t *ckpt_rel_x,
				 int16_t *ckpt_rel_y, int16_t *back_mm);
uint8_t strat_scan_disc(int16_t angle, uint8_t mode,
			struct scan_disc_result *result);
uint8_t strat_goto_disc_angle(int16_t a_deg, int8_t level);
int16_t strat_get_temple_angle(struct temple *temple);
int16_t strat_temple_angle_to_scan_angle(int16_t temple_angle);
uint8_t strat_build_on_opponent_temple(void);
uint8_t strat_check_temple_and_build(void);

#endif
