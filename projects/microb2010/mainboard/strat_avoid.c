/*
 *  Copyright Droids, Microb Technology (2010)
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
#include <stdint.h>
#include <math.h>

#include <aversive.h>
#include <aversive/pgmspace.h>

#include <ax12.h>
#include <uart.h>
#include <pwm_ng.h>
#include <clock_time.h>
#include <spi.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <trajectory_manager.h>
#include <trajectory_manager_utils.h>
#include <trajectory_manager_core.h>
#include <vect_base.h>
#include <lines.h>
#include <polygon.h>
#include <obstacle_avoidance.h>
#include <blocking_detection_manager.h>
#include <robot_system.h>
#include <position_manager.h>

#include <diagnostic.h>

#include <rdline.h>
#include <parse.h>

#include "../common/i2c_commands.h"
#include "i2c_protocol.h"
#include "main.h"
#include "strat.h"
#include "strat_db.h"
#include "strat_base.h"
#include "strat_corn.h"
#include "strat_utils.h"
#include "sensor.h"
#include "actuator.h"

/* XXX TODO
static
const
change x,y -> i,j to avoid confusion with coords
could be optimized in mem space: it is not needed to store the x,y coord,
   we can process it from idx. however it will be less optimized for speed

*/

/* XXX enum possible ? else just rename */
#define START      0
#define UP         1
#define UP_RIGHT   2
#define DOWN_RIGHT 3
#define DOWN       4
#define DOWN_LEFT  5
#define UP_LEFT    6
#define END        7

struct djpoint {
	uint16_t weight;
	struct djpoint *parent;

	uint8_t parent_pos:3;
	uint8_t updated:1;
	uint8_t todo:1;
	uint8_t reserved:3;
};

/* database for dijkstra */
static struct djpoint djpoints[WAYPOINTS_NBX][WAYPOINTS_NBY];

/* return index from neigh pointer */
#define PT2IDX(neigh) ( ((void *)(neigh)-(void *)(&djpoints)) / sizeof(*neigh) )

void dump(void)
{
	int8_t i, j;
	struct djpoint *pt;
	struct waypoint_db *wp;

	printf_P(PSTR("         "));
	for (i=0; i<WAYPOINTS_NBX; i++) {
		printf_P(PSTR(" %2d "), i);
	}
	printf_P(PSTR("\r\n"));

	for (j=WAYPOINTS_NBY*2-1; j>=0; j--) {
		printf_P(PSTR("%3d   "), j/2);

		if ((j&1) == 0)
			printf_P(PSTR("    "));

		for (i=0; i<WAYPOINTS_NBX; i++) {
			pt = &djpoints[i][j/2];
			wp = &strat_db.wp_table[i][j/2];

			if (((i+j) & 1) == 0)
				continue;

			if (wp->type == WP_TYPE_OBSTACLE)
				printf_P(PSTR("     X  "));
			else if (wp->dangerous)
				printf_P(PSTR("     D  "));
			else if (wp->type == WP_TYPE_CORN &&
				 wp->corn.color == I2C_COB_WHITE)
				printf_P(PSTR("     W  "));
			else if (wp->type == WP_TYPE_CORN &&
				 wp->corn.color == I2C_COB_BLACK)
				printf_P(PSTR("     B  "));
			else if (wp->type == WP_TYPE_CORN &&
				 wp->corn.color == I2C_COB_UNKNOWN)
				printf_P(PSTR("     U  "));
 			else if (wp->type == WP_TYPE_WAYPOINT ||
				 wp->type == WP_TYPE_TOMATO)
				printf_P(PSTR(" %5d  "), pt->weight);
 			else
 				printf_P(PSTR("     ?  "));
		}
		printf_P(PSTR("\r\n"));
	}
}

static inline uint8_t opposite_position(uint8_t pos)
{
	pos += 3;
	if (pos > UP_LEFT)
		pos -= 6;
	return pos;
}

/* is point reachable by the robot ? */
static uint8_t is_reachable(uint8_t i, uint8_t j)
{
	struct waypoint_db *wp;

	wp = &strat_db.wp_table[i][j];
	if (wp->type == WP_TYPE_WAYPOINT)
		return 1;
	if (wp->type == WP_TYPE_TOMATO)
		return 1;
	if (wp->type == WP_TYPE_CORN &&
	    wp->present == 0)
		return 1;
	return 0;
}

/* return coord of the entry in the table from the pointer */
static void djpoint2ij(struct djpoint *pt, uint8_t *i, uint8_t *j)
{
	int8_t idx = PT2IDX(pt);
	*i = idx / WAYPOINTS_NBY;
	*j = idx % WAYPOINTS_NBY;
}

/* get the neighbour of the point at specified position */
static struct djpoint *get_neigh(struct djpoint *pt,
				 uint8_t position)
{
	uint8_t i,j;
	struct djpoint *neigh;

	djpoint2ij(pt, &i, &j);

	switch (position) {
	case UP:
		j++;
		break;
	case UP_RIGHT:
		if (!(i & 1)) j++;
		i++;
		break;
	case DOWN_RIGHT:
		if (i & 1) j--;
		i++;
		break;
	case DOWN:
		j--;
		break;
	case DOWN_LEFT:
		if (i & 1) j--;
		i--;
		break;
	case UP_LEFT:
		if (!(i & 1)) j++;
		i--;
		break;
	default:
		return NULL;
	}
	if (i >= WAYPOINTS_NBX || j >= WAYPOINTS_NBY)
		return NULL;

	if (is_reachable(i, j) == 0)
		return NULL;

	neigh = &djpoints[i][j];
	return neigh;
}

static struct djpoint *get_next_neigh(struct djpoint *pt, uint8_t *position)
{
	struct djpoint *neigh = NULL;
	uint8_t pos = *position + 1;

	for (pos = *position + 1; pos < END; pos++) {
		neigh = get_neigh(pt, pos);
		if (neigh != NULL)
			break;
	}

	*position = pos;
	return neigh;
}

/* browse all points */
#define POINT_FOREACH(cur)						\
	for (cur = &djpoints[0][0];					\
	     cur < &djpoints[WAYPOINTS_NBX][WAYPOINTS_NBY];		\
	     cur ++)

/* XXX comment */
#define NEIGH_FOREACH(neigh, pos, point)			\
	for (pos = START, neigh = get_next_neigh(point, &pos);	\
	     neigh;						\
	     neigh = get_next_neigh(point, &pos))

int dijkstra_init(void)
{
	return 0;
}

/* return distance between p1 and p2 */
static uint16_t dist(struct djpoint *p1, struct djpoint *p2)
{
	int16_t x1, y1, x2, y2;
	double vx, vy;
	uint8_t i, j;

	djpoint2ij(p1, &i, &j);
	ijcoord_to_xycoord(i, j, &x1, &y1);

	djpoint2ij(p2, &i, &j);
	ijcoord_to_xycoord(i, j, &x2, &y2);

	vx = x2 - x1;
	vy = y2 - y1;
	return sqrt(vx * vx + vy * vy);
}

void dijkstra_process_neighs(struct djpoint *pt)
{
 	struct djpoint *neigh;
	uint8_t pos, parent_pos;
	uint16_t weight;
	uint8_t i,j;

	djpoint2ij(pt, &i, &j);
	printf_P(PSTR("at %d %d:\r\n"), i, j);

	NEIGH_FOREACH(neigh, pos, pt) {
		weight = pt->weight + dist(pt, neigh);
		parent_pos = opposite_position(pos);

		/* bonus if we keep the same direction */
		if (parent_pos == pt->parent_pos ||
		    pt->parent_pos == END)
			weight = weight - 1;

		printf_P(PSTR("  pos=%d: ppos=%d opos=%d nw=%d w=%d\r\n"), pos,
		       pt->parent_pos, parent_pos,
		       neigh->weight, weight);
		if (neigh->weight == 0 || weight < neigh->weight) {
			djpoint2ij(neigh, &i, &j);
			//printf_P(PSTR("    %d,%d updated\r\n"), i, j);
			neigh->weight = weight;
			neigh->parent_pos = parent_pos;
			neigh->updated = 1;
		}
	}
}

int dijkstra(struct djpoint *start)
{
 	struct djpoint *cur;
	uint8_t todolist = 1;

	start->todo = 1;

	while (todolist) {
		printf_P(PSTR("\r\n"));
		dump();
		/* process all neighbours of todo list */
		POINT_FOREACH(cur) {
			if (!cur->todo)
				continue;
			dijkstra_process_neighs(cur);
			cur->todo = 0;
		}

		/* convert updated list in todo list */
		todolist = 0;
		POINT_FOREACH(cur) {
			if (!cur->updated)
				continue;
			todolist = 1;
			cur->todo = 1;
			cur->updated = 0;
		}
	}
	return 0; /* XXX */
}

/* init waypoints position */
void init_djpoints(void)
{
	int8_t i, j;
	struct djpoint *pt;

	for (i=0; i<WAYPOINTS_NBX; i++) {

		for (j=0; j<WAYPOINTS_NBY; j++) {
			pt = &djpoints[i][j];
			pt->parent_pos = END;
			pt->updated = 0;
			pt->todo = 0;
			pt->weight = 0;
		}
	}
}

int get_path(struct djpoint *start, struct djpoint *end)
{
 	struct djpoint *cur;
	uint8_t prev_direction = 0;
	int8_t idx;
	int16_t x, y;

	for (cur = start;
	     cur != NULL && cur->parent_pos != END && cur != end;
	     cur = get_neigh(cur, cur->parent_pos)) {
		if (prev_direction != cur->parent_pos) {
			idx = PT2IDX(cur);
			corn_idx_to_xycoord(idx, &x, &y);
			printf_P(PSTR("%d %d (%d)\r\n"),
				 x, y, cur->parent_pos);
		}
		prev_direction = cur->parent_pos;
	}
	idx = PT2IDX(end);
	corn_idx_to_xycoord(idx, &x, &y);
	printf_P(PSTR("%d %d\r\n"), x, y);

	return 0; /* XXX */
}
