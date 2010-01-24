/*  
 *  Copyright Droids, Microb Technology (2009)
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
 *  Revision : $Id: strat.c,v 1.6 2009-11-08 17:24:33 zer0 Exp $
 *
 *  Olivier MATZ <zer0@droids-corp.org> 
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <aversive/pgmspace.h>
#include <aversive/queue.h>
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
#include "i2c_commands.h"


#define COL_DISP_MARGIN 400 /* stop 40 cm in front of dispenser */
#define COL_SCAN_PRE_MARGIN 250


#ifdef TEST_BEACON

#define BEACON_MAX_SAMPLES 100
struct beacon_sample {
	int16_t posx;
	int16_t posy;
	int16_t posa;
	int16_t oppx;
	int16_t oppy;
	uint8_t time;
};

static struct beacon_sample beacon_sample[BEACON_MAX_SAMPLES];
static uint8_t beacon_prev_time = 0;
static uint8_t beacon_cur_idx = 0;

static void beacon_update_samples(void)
{
	int16_t opp_a, opp_d, opp_x, opp_y;
	int8_t err;
	uint8_t time;

	time = time_get_s();

	/* one sample per second max */
	if (time <= beacon_prev_time)
		return;
	/* limit max number of samples */
	if (beacon_cur_idx >= BEACON_MAX_SAMPLES)
		return;

	memset(&beacon_sample[beacon_cur_idx], 0, sizeof(beacon_sample[beacon_cur_idx]));
	beacon_prev_time = time;
	beacon_sample[beacon_cur_idx].time = time;
	
	/* get opponent pos; if not found, just set struct to 0 */
	err = get_opponent_xyda(&opp_x, &opp_y, &opp_d, &opp_a);
	if (err == -1)
		return;

	beacon_sample[beacon_cur_idx].posx = position_get_x_s16(&mainboard.pos);
	beacon_sample[beacon_cur_idx].posy = position_get_y_s16(&mainboard.pos);
	beacon_sample[beacon_cur_idx].posa = position_get_a_deg_s16(&mainboard.pos);
	beacon_sample[beacon_cur_idx].oppx = opp_x;
	beacon_sample[beacon_cur_idx].oppy = opp_y;
	beacon_cur_idx++;
}

void beacon_dump_samples(void)
{
	uint16_t i;

	for (i=0; i<BEACON_MAX_SAMPLES; i++) {
		printf_P(PSTR("%d: pos=(%d,%d,%d) opp=(%d,%d) time=%d\r\n"),
			 i,
			 beacon_sample[i].posx,
			 beacon_sample[i].posy,
			 beacon_sample[i].posa,
			 beacon_sample[i].oppx,
			 beacon_sample[i].oppy,
			 beacon_sample[i].time);
	}
}
#endif

struct strat_infos strat_infos = {
	/* conf */
	.conf = {
		.flags = 0,
		/* scanner disabled by default */
		.scan_opp_min_time = 90,
		.delay_between_opp_scan = 90,
		.scan_our_min_time = 90,
		.delay_between_our_scan = 90,
		.wait_opponent = 0,
		.lintel_min_time = 0,
		.scan_opp_angle = -1,
	},

	/* static columns */
	.s_cols = { 
		.flags = 0, 
		.configuration = 0,
	},

	/* column dispensers ; be carreful, positions are
	 * color-dependent, so COLOR_Y() and COLOR_A() should be
	 * used. All angles here are _absolute_ */
	.c1 = {
		.checkpoint_x = 2711 - COL_SCAN_PRE_MARGIN,
		.checkpoint_y = AREA_Y - COL_DISP_MARGIN,
		.scan_left = 0,
		.scan_a = 180,
		.eject_a = 180,
		.recalib_x = 2711,
		.recalib_y = AREA_Y - (ROBOT_LENGTH/2 + DIST_BACK_DISPENSER),
		.pickup_a = 90,
		.name = "col_disp1",
	},
	.c2 = {
		.checkpoint_x = AREA_X - COL_DISP_MARGIN,
		.checkpoint_y = 800 - COL_SCAN_PRE_MARGIN,
		.scan_left = 1,
		.scan_a = -90,
		.eject_a = -90,
		.recalib_x = AREA_X - (ROBOT_LENGTH/2 + DIST_BACK_DISPENSER),
		.recalib_y = 800,
		.pickup_a = 0,
		.name = "col_disp2",
	},
	.c3 = {
		.checkpoint_x = AREA_X-COL_DISP_MARGIN,
		.checkpoint_y = 1300 + COL_SCAN_PRE_MARGIN,
		.scan_a = 90,
		.scan_left = 0,
		.eject_a = -90,
		.recalib_x = AREA_X - (ROBOT_LENGTH/2 + DIST_BACK_DISPENSER),
		.recalib_y = 1300,
		.pickup_a = 0,
		.name = "col_disp3",
	},
	
	/* lintel dispensers */
	.l1 = {
		.x = 912, /* XXX for red only */
		.name = "lin_disp1",
	},
	.l2 = {
		.x = 1312,  /* XXX for red only */
		.name = "lin_disp2",
	},

	/* build zones */
	.zone_list = {
#define ZONE_DISC_NUM 0
		{
			.flags = ZONE_F_VALID | ZONE_F_DISC,
			.level = 2,
			.checkpoint_x = 0,
			.checkpoint_x = 0,
			.name = "disc",
		},
#define ZONE_1A_NUM 1
		{
			.flags = ZONE_F_VALID,
			.level = 1,
			.checkpoint_x = 1385,
			.checkpoint_y = 1700,
			.name = "z1a",
		},
#define ZONE_1B_NUM 2
		{
			.flags = ZONE_F_VALID,
			.level = 1,
			.checkpoint_x = 1615,
			.checkpoint_y = 1700,
			.name = "z1b",
		},
#define ZONE_0B_NUM 3
		{
			.flags = ZONE_F_VALID,
			.level = 0,
			.checkpoint_x = 2100,
			.checkpoint_y = 1700,
			.name = "z0b",
		},
#define ZONE_0A_NUM 4
		{
			.flags = ZONE_F_VALID,
			.level = 0,
			.checkpoint_x = 900,
			.checkpoint_y = 1700,
			.name = "z0a",
		},
	}
};

/*************************************************************/

/*                  INIT                                     */

/*************************************************************/

void strat_set_bounding_box(void)
{
	if (get_color() == I2C_COLOR_RED) {
		strat_infos.area_bbox.x1 = 300;
		strat_infos.area_bbox.y1 = 200;
		strat_infos.area_bbox.x2 = 2720; /* needed for c1 */
		strat_infos.area_bbox.y2 = 1800;
	}
	else {
		strat_infos.area_bbox.x1 = 200;
		strat_infos.area_bbox.y1 = 300;
		strat_infos.area_bbox.x2 = 2720; /* needed for c1 */
		strat_infos.area_bbox.y2 = 1900;
	}

	polygon_set_boundingbox(strat_infos.area_bbox.x1,
				strat_infos.area_bbox.y1,
				strat_infos.area_bbox.x2,
				strat_infos.area_bbox.y2);
}

/* called before each strat, and before the start switch */
void strat_preinit(void)
{
	time_reset();
	interrupt_traj_reset();
	mainboard.flags =  DO_ENCODERS | DO_CS | DO_RS |
		DO_POS | DO_BD | DO_POWER;

#ifndef HOST_VERSION
	i2c_mechboard_mode_init();
	if (get_color() == I2C_COLOR_RED)
		i2c_mechboard_mode_prepare_pickup(I2C_LEFT_SIDE);
	else
		i2c_mechboard_mode_prepare_pickup(I2C_RIGHT_SIDE);
#endif
	strat_dump_conf();
	strat_dump_infos(__FUNCTION__);
}

void strat_dump_conf(void)
{
	if (!strat_infos.dump_enabled)
		return;

	printf_P(PSTR("-- conf --\r\n"));

	printf_P(PSTR("  one build on disc: "));
	if (strat_infos.conf.flags & STRAT_CONF_ONLY_ONE_ON_DISC)
		printf_P(PSTR("on\r\n"));
	else
		printf_P(PSTR("off\r\n"));

	printf_P(PSTR("  bypass static2: "));
	if (strat_infos.conf.flags & STRAT_CONF_BYPASS_STATIC2)
		printf_P(PSTR("on\r\n"));
	else
		printf_P(PSTR("off\r\n"));

	printf_P(PSTR("  take one lintel: "));
	if (strat_infos.conf.flags & STRAT_CONF_TAKE_ONE_LINTEL)
		printf_P(PSTR("on\r\n"));
	else
		printf_P(PSTR("off\r\n"));

	printf_P(PSTR("  skip this temple when temple check fails: "));
	if (strat_infos.conf.flags & STRAT_CONF_SKIP_WHEN_CHECK_FAILS)
		printf_P(PSTR("on\r\n"));
	else
		printf_P(PSTR("off\r\n"));

	printf_P(PSTR("  store static2: "));
	if (strat_infos.conf.flags & STRAT_CONF_STORE_STATIC2)
		printf_P(PSTR("on\r\n"));
	else
		printf_P(PSTR("off\r\n"));

	printf_P(PSTR("  (big3) try to build a temple with 3 lintels: "));
	if (strat_infos.conf.flags & STRAT_CONF_BIG_3_TEMPLE)
		printf_P(PSTR("on\r\n"));
	else
		printf_P(PSTR("off\r\n"));

	printf_P(PSTR("  early opponent scan: "));
	if (strat_infos.conf.flags & STRAT_CONF_EARLY_SCAN)
		printf_P(PSTR("on\r\n"));
	else
		printf_P(PSTR("off\r\n"));

	printf_P(PSTR("  push opponent columns: "));
	if (strat_infos.conf.flags & STRAT_CONF_PUSH_OPP_COLS)
		printf_P(PSTR("on\r\n"));
	else
		printf_P(PSTR("off\r\n"));

	printf_P(PSTR("  scan opponent min time: %d\r\n"),
		 strat_infos.conf.scan_opp_min_time);
	printf_P(PSTR("  delay between oppnent scan: %d\r\n"),
		 strat_infos.conf.delay_between_opp_scan);
	printf_P(PSTR("  scan our min time: %d\r\n"),
		 strat_infos.conf.scan_our_min_time);
	printf_P(PSTR("  delay between our scan: %d\r\n"),
		 strat_infos.conf.delay_between_our_scan);
	printf_P(PSTR("  wait opponent gone before scan: %d\r\n"),
		 strat_infos.conf.wait_opponent);
	printf_P(PSTR("  lintel min time: %d\r\n"),
		 strat_infos.conf.lintel_min_time);
	printf_P(PSTR("  scan_opp_angle: %d\r\n"),
		 strat_infos.conf.scan_opp_angle);
}

void strat_dump_temple(struct temple *temple)
{
	if (!strat_infos.dump_enabled)
		return;

	printf_P(PSTR("  temple %p (%s): "), temple, temple->zone->name);

	if (temple->flags & TEMPLE_F_MONOCOL)
		printf_P(PSTR("MONOCOL "));
	else
		printf_P(PSTR("BICOL "));

	if (temple->flags & TEMPLE_F_ON_DISC)
		printf_P(PSTR("ON_DISC "));
	else
		printf_P(PSTR("ON_ZONE_0_1 "));
	
	if (temple->flags & TEMPLE_F_OPPONENT)
		printf_P(PSTR("OPPONENT "));
	else
		printf_P(PSTR("OURS "));

	if (temple->flags & TEMPLE_F_LINTEL)
		printf_P(PSTR("LIN_ON_TOP "));
	else
		printf_P(PSTR("COL_ON_TOP "));

	printf_P(PSTR("\r\n"));

	printf_P(PSTR("   pos=(%d,%d,%d) ckpt=(%d,%d) ltime=%d\r\n"),
		 temple->x, temple->y, temple->a,
		 temple->checkpoint_x, temple->checkpoint_y,
		 temple->last_try_time);
	printf_P(PSTR("   L: lev=%d da=%d,%d\r\n"),
		 temple->level_l, temple->dist_l, temple->angle_l);
	printf_P(PSTR("   R: lev=%d da=%d,%d\r\n"),
		 temple->level_l, temple->dist_l, temple->angle_l);
}

void strat_dump_zone(struct build_zone *zone)
{
	if (!strat_infos.dump_enabled)
		return;

	printf_P(PSTR("  zone %s: "), zone->name);

	if (zone->flags & ZONE_F_DISC)
		printf_P(PSTR("DISC "));
	else if (zone->flags & ZONE_F_ZONE1)
		printf_P(PSTR("ZONE1 "));
	else if (zone->flags & ZONE_F_ZONE0)
		printf_P(PSTR("ZONE0 "));

	if (zone->flags & ZONE_F_BUSY)
		printf_P(PSTR("BUSY "));
	else
		printf_P(PSTR("FREE "));
	
	printf_P(PSTR("\r\n"));

	printf_P(PSTR("    lev=%d ckpt=(%d,%d) ltime=%d\r\n"),
		 zone->level,
		 zone->checkpoint_x, zone->checkpoint_y,
		 zone->last_try_time);
}

void strat_dump_static_cols(void)
{
	if (!strat_infos.dump_enabled)
		return;

	printf_P(PSTR("  static cols: l0=%d l1=%d l2=%d\r\n"),
		 strat_infos.s_cols.flags & STATIC_COL_LINE0_DONE,
		 strat_infos.s_cols.flags & STATIC_COL_LINE1_DONE,
		 strat_infos.s_cols.flags & STATIC_COL_LINE2_DONE);
}

void strat_dump_col_disp(void)
{
	if (!strat_infos.dump_enabled)
		return;

	printf_P(PSTR("  c1 cnt=%d ltt=%d\r\n"),
		 strat_infos.c1.count, strat_infos.c1.last_try_time);
	printf_P(PSTR("  c2 cnt=%d ltt=%d\r\n"),
		 strat_infos.c2.count, strat_infos.c2.last_try_time);
	printf_P(PSTR("  c3 cnt=%d ltt=%d\r\n"),
		 strat_infos.c3.count, strat_infos.c3.last_try_time);
}

void strat_dump_lin_disp(void)
{
	if (!strat_infos.dump_enabled)
		return;
	printf_P(PSTR("  l1 cnt=%d ltt=%d\r\n"),
		 strat_infos.l1.count, strat_infos.l1.last_try_time);
	printf_P(PSTR("  l2 cnt=%d ltt=%d\r\n"),
		 strat_infos.l2.count, strat_infos.l2.last_try_time);

}

void strat_dump_all_temples(void)
{
	struct temple *temple;
	uint8_t i;

	if (!strat_infos.dump_enabled)
		return;

	for (i=0; i<MAX_TEMPLE; i++) {
		temple = &strat_infos.temple_list[i];
		if (!(temple->flags & TEMPLE_F_VALID))
			continue;
		strat_dump_temple(temple);
	}
}

void strat_dump_all_zones(void)
{
	struct build_zone *zone;
	uint8_t i;

	if (!strat_infos.dump_enabled)
		return;

	for (i=0; i<MAX_ZONE; i++) {
		zone = &strat_infos.zone_list[i];
		if (!(zone->flags & ZONE_F_VALID))
			continue;
		strat_dump_zone(zone);
	}
}

/* display current information about the state of the game */
void strat_dump_infos(const char *caller)
{
	if (!strat_infos.dump_enabled)
		return;

	printf_P(PSTR("%s() dump strat infos:\r\n"), caller);
	strat_dump_static_cols();
	strat_dump_col_disp();
	strat_dump_lin_disp();
	strat_dump_all_temples();
	strat_dump_all_zones();
}

/* init current area state before a match. Dump update user conf
 * here */
void strat_reset_infos(void)
{
	uint8_t i;

	/* /!\ don't do a big memset() as there is static data */
	strat_infos.s_cols.flags = 0;
	strat_infos.c1.count = 5;
	strat_infos.c1.last_try_time = 0;
	strat_infos.c2.count = 5;
	strat_infos.c2.last_try_time = 0;
	strat_infos.c3.count = 5;
	strat_infos.c3.last_try_time = 0;
	strat_infos.l1.count = 1;
	strat_infos.l1.last_try_time = 0;
	strat_infos.l2.count = 1;
	strat_infos.l2.last_try_time = 0;

	strat_infos.taken_lintel = 0;
	strat_infos.col_in_boobs = 0;
	strat_infos.lazy_pickup_done = 0;
	strat_infos.i2c_loaded_skipped = 0;
	
	memset(strat_infos.temple_list, 0, sizeof(strat_infos.temple_list));

	for (i=0; i<MAX_ZONE; i++)
		strat_infos.zone_list[i].flags = ZONE_F_VALID;
	strat_infos.zone_list[ZONE_DISC_NUM].flags |= ZONE_F_DISC;
	strat_infos.zone_list[ZONE_1A_NUM].flags |= ZONE_F_ZONE1;
	strat_infos.zone_list[ZONE_1B_NUM].flags |= ZONE_F_ZONE1;
	strat_infos.zone_list[ZONE_0A_NUM].flags |= ZONE_F_ZONE0;
	strat_infos.zone_list[ZONE_0B_NUM].flags |= ZONE_F_ZONE0;

	strat_set_bounding_box();

	/* set lintel position, depending on color */
	if (mainboard.our_color == I2C_COLOR_RED) {
		strat_infos.l1.x = 912;
		strat_infos.l2.x = 1312;
	}
	else {
		strat_infos.l1.x = 888;
		strat_infos.l2.x = 1288;
	}
}

/* call it just before launching the strat */
void strat_init(void)
{
#ifdef HOST_VERSION
	printf("not implemented\n");
#else
	pickup_wheels_on();
	strat_reset_infos();

	/* we consider that the color is correctly set */

	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);
	time_reset();
	interrupt_traj_reset();

	/* used in strat_base for END_TIMER */
	mainboard.flags = DO_ENCODERS | DO_CS | DO_RS | 
		DO_POS | DO_BD | DO_TIMER | DO_POWER;

#ifdef TEST_BEACON
	beacon_prev_time = 0;
	beacon_cur_idx = 0;
#endif
#endif
}


/* call it after each strat */
void strat_exit(void)
{
#ifdef HOST_VERSION
	printf("not implemented\n");
#else
	uint8_t flags;

	pickup_wheels_off();
	mainboard.flags &= ~(DO_TIMER);
	strat_hardstop();
	time_reset();
	wait_ms(1000);
	IRQ_LOCK(flags);
	mainboard.flags &= ~(DO_CS);
	pwm_ng_set(LEFT_PWM, 0);
	pwm_ng_set(RIGHT_PWM, 0);
	IRQ_UNLOCK(flags);
#endif
}

/* called periodically */
void strat_event(void *dummy)
{
	/* limit speed when opponent is close */
	strat_limit_speed();

#ifdef TEST_BEACON
	beacon_update_samples();
#endif
}

#ifndef HOST_VERSION
/* do static cols + first temples */
static uint8_t strat_beginning(void)
{
	uint8_t err;

	/* don't limit the speed when opponent is near: it can change
	 * the radius of the curves */
	strat_limit_speed_disable();

	err = strat_static_columns(0);

	strat_limit_speed_enable();

	if (!TRAJ_SUCCESS(err))
		return err;

	/* go to disc to build the first temple */

	/* XXX if opponent is near disc, go to zone1 */
	err = strat_goto_disc(2);
	if (!TRAJ_SUCCESS(err))
		return err;
	DEBUG(E_USER_STRAT, "disc reached");

	/* can return END_ERROR or END_TIMER, should not happen
	 * here */
	err = strat_build_new_temple(&strat_infos.zone_list[0]);
	if (!TRAJ_SUCCESS(err))
		return err;

	/* bypass static2 if specified */
	if (strat_infos.conf.flags & STRAT_CONF_BYPASS_STATIC2) {
		err = strat_escape(&strat_infos.zone_list[0], TRAJ_FLAGS_STD);
		return err;
	}

	/* get the last 2 columns, and build them on previous temple */
	err = strat_static_columns_pass2();
	if (!TRAJ_SUCCESS(err))
		return err;

	/* early opponent scan, for offensive strategy */
	if (strat_infos.conf.flags & STRAT_CONF_EARLY_SCAN) {
		err = strat_pickup_lintels();
		/* ignore code */

		/* try to build on opponent (scan must be enabled) */
		err = strat_build_on_opponent_temple();
		/* ignore code */
	}

	return err;
}
#endif

/* return true if we need to grab some more elements (lintel/cols) */
uint8_t need_more_elements(void)
{
#ifdef HOST_VERSION
	printf("not implemented\n");
#else
	if (time_get_s() <= 75) {
		/* we have at least one col on each arm, build now */
		if ((get_column_count_left() >= 1) && 
		    (get_column_count_right() >= 1))
			return 0;
	}
	else {
		if (get_column_count())
			return 0;
	}
#endif
	return 1;
}

/* dump state (every 5 s max) */
#define DUMP_RATE_LIMIT(dump, last_print)		\
	do {						\
		if (time_get_s() - last_print > 5) {	\
			dump();				\
			last_print = time_get_s();	\
		}					\
	} while (0)


uint8_t strat_main(void)
{
#ifdef HOST_VERSION
	printf("not implemented\n");
	return END_ERROR;
#else
	uint8_t err;
	struct temple *temple = NULL;
	struct build_zone *zone = NULL;

	uint8_t last_print_cols = 0;
	uint8_t last_print_lin = 0;
	uint8_t last_print_temple = 0;
	uint8_t last_print_zone = 0;

	/* do static cols + first temple */
	err = strat_beginning();

	/* skip error code */

	while (1) {
		
		if (err == END_TIMER) {
			DEBUG(E_USER_STRAT, "End of time");
			strat_exit();
			break;
		}

		/* we have at least one col on each arm, build now */
		if (need_more_elements() == 0) {
			
			/* try to build on opponent, will return
			 * END_TRAJ without doing anything if
			 * disabled */
			err = strat_build_on_opponent_temple();
			if (!TRAJ_SUCCESS(err))
				continue;
			if (need_more_elements())
				continue;

			/* try to scan and build on our temple, will
			 * return END_TRAJ without doing anything if
			 * disabled */
			err = strat_check_temple_and_build();
			if (!TRAJ_SUCCESS(err))
				continue;
			if (need_more_elements())
				continue;

			/* Else, do a simple build, as before */

			temple = strat_get_best_temple();

			/* one valid temple found */
			if (temple) {
				DUMP_RATE_LIMIT(strat_dump_all_temples, last_print_temple);

				err = strat_goto_temple(temple);
				if (!TRAJ_SUCCESS(err))
					continue;

				/* can return END_ERROR or END_TIMER,
				 * should not happen here */
				err = strat_grow_temple(temple);
				if (!TRAJ_SUCCESS(err))
					continue;
				
				err = strat_escape(temple->zone, TRAJ_FLAGS_STD);
				if (!TRAJ_SUCCESS(err))
					continue;

				continue;
			}

			zone = strat_get_best_zone();
			if (zone) {
				DUMP_RATE_LIMIT(strat_dump_all_zones, last_print_zone);

				DEBUG(E_USER_STRAT, "goto zone %s", zone->name);
				err = strat_goto_build_zone(zone, zone->level);
				if (!TRAJ_SUCCESS(err))
					continue;
				DEBUG(E_USER_STRAT, "zone reached");
				
				/* no error code except END_ERROR, should not happen */
				err = strat_build_new_temple(zone);

				err = strat_escape(zone, TRAJ_FLAGS_STD);
				if (!TRAJ_SUCCESS(err))
					continue;

				continue;
			}

			/* XXX hey what can we do here... :'( */
			DEBUG(E_USER_STRAT, "panic :)");
			time_wait_ms(1000);
			continue;
		}

		/* else we need some elements (lintels, then columns) */
		else {
			if (strat_infos.l1.count != 0 && strat_infos.l2.count != 0)
				DUMP_RATE_LIMIT(strat_dump_lin_disp, last_print_lin);

			err = strat_pickup_lintels();
			 /* can return an error code, but we have
			  * nothing to do because pickup_column()
			  * starts with a goto_and_avoid() */
			if (!TRAJ_SUCCESS(err))
				nop();
			
			DUMP_RATE_LIMIT(strat_dump_col_disp, last_print_cols);

			err = strat_pickup_columns();
			if (!TRAJ_SUCCESS(err))
				nop(); /* nothing to do */

			/* XXX check here that we have elements, or do
			 * something else */
			/* if we cannot take elements, try to build */
		}
	}
	return END_TRAJ;
#endif
}
