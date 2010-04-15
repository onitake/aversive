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

#include <vect_base.h>

/* XXX TODO
static
const
change x,y -> i,j to avoid confusion with coords
could be optimized in mem space: it is not needed to store the x,y coord,
   we can process it from idx. however it will be less optimized for speed

*/

#define OFFSET_CORN_X 150
#define OFFSET_CORN_Y 222
#define STEP_CORN_X 225
#define STEP_CORN_Y 250

#define CORN_NB 18

#define WAYPOINTS_NBX 13
#define WAYPOINTS_NBY 8

/* enum is better */
#define TYPE_WAYPOINT   0
#define TYPE_DANGEROUS  1
#define TYPE_WHITE_CORN 2
#define TYPE_BLACK_CORN 3
#define TYPE_OBSTACLE   4
#define TYPE_UNKNOWN    5

/* XXX enum possible ? else just rename */
#define START      0
#define UP         1
#define UP_RIGHT   2
#define DOWN_RIGHT 3
#define DOWN       4
#define DOWN_LEFT  5
#define UP_LEFT    6
#define END        7

struct point {
	int32_t x;
	int32_t y;
};

struct djpoint {
	struct point pos;
	uint16_t weight;
	struct djpoint *parent;

	uint8_t type:3;
	uint8_t parent_pos:3;
	uint8_t updated:1;
	uint8_t todo:1;
};

uint8_t corn_table[CORN_NB];

const uint8_t corn_coord_i[CORN_NB] = {
	0, 0, 0, 2, 2, 2, 4, 4, 6,
	6, 8, 8, 10, 10, 10, 12, 12, 12,
};

const uint8_t corn_coord_j[CORN_NB] = {
	2, 4, 6, 3, 5, 7, 4, 6, 5,
	7, 4, 6, 3, 5, 7, 2, 4, 6,
};

static struct djpoint djpoints[WAYPOINTS_NBX][WAYPOINTS_NBY];

/* table to find the symetric idx */
uint8_t corn_sym[] = {
	15, 16, 17, 12, 13, 14, 10, 11, 8, 9, 6, 7, 3, 4, 5, 0, 1, 2
};

uint8_t corn_side_confs[9][2] = {
	{ 1, 4 },
	{ 0, 4 },
	{ 2, 4 },
	{ 2, 3 },
	{ 0, 3 },
	{ 1, 3 },
	{ 1, 6 },
	{ 0, 6 },
	{ 2, 6 },
};
uint8_t corn_center_confs[4][2] = {
	{ 5, 8 },
	{ 7, 8 },
	{ 5, 9 },
	{ 7, 8 },
};


/* return index from neigh pointer */
#define PT2IDX(neigh) ( ((void *)(neigh)-(void *)(&djpoints)) / sizeof(*neigh) )

void dump(void)
{
	int8_t i, j;
	struct djpoint *pt;

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

			if (((i+j) & 1) == 0)
				continue;

			if (pt->type == TYPE_OBSTACLE)
				printf_P(PSTR("     X  "));
			else if (pt->type == TYPE_DANGEROUS)
				printf_P(PSTR("     D  "));
			else if (pt->type == TYPE_WHITE_CORN)
				printf_P(PSTR("     W  "));
			else if (pt->type == TYPE_BLACK_CORN)
				printf_P(PSTR("     B  "));
 			else if (pt->type == TYPE_WAYPOINT)
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

/* return coord of the entry in the table from the pointer */
static void djpoint2ij(struct djpoint *pt, int8_t *x, int8_t *y)
{
	int8_t idx = PT2IDX(pt);
	*x = idx / WAYPOINTS_NBY;
	*y = idx % WAYPOINTS_NBY;
}

/* get the neighbour of the point at specified position */
static struct djpoint *get_neigh(struct djpoint *pt,
				 uint8_t position)
{
	int8_t i,j;
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
	if (i < 0 || j < 0 || i >= WAYPOINTS_NBX || j >= WAYPOINTS_NBY)
		return NULL;

	neigh = &djpoints[i][j];

	if (neigh->type != TYPE_WAYPOINT)
		return NULL;

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

static uint16_t dist(struct djpoint *p1, struct djpoint *p2)
{
	double vx, vy;
	vx = p2->pos.x - p1->pos.x;
	vy = p2->pos.y - p1->pos.y;
	return sqrt(vx * vx + vy * vy);
}

void dijkstra_process_neighs(struct djpoint *pt)
{
 	struct djpoint *neigh;
	uint8_t pos, parent_pos;
	uint16_t weight;
	int8_t i,j;

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

int8_t ijcoord_to_corn_idx(uint8_t i, uint8_t j)
{
	uint8_t n;
	for (n = 0; n < CORN_NB; n ++) {
		if (i == corn_coord_i[n] &&
		    j == corn_coord_j[n])
			return n;
	}
	return -1;
}

int8_t corn_idx_to_coordij(uint8_t idx, uint8_t *i, uint8_t *j)
{
	if (idx >= CORN_NB)
		return -1;
	*i = corn_coord_i[idx];
	*j = corn_coord_j[idx];
	return 0;
}

/* return the index of the closest corn at these coordinates. If the
 * corn is really too far (~20cm), return -1. The x and y pointer are
 * updated with the real position of the corn */
int8_t xycoord_to_corn_idx(int16_t *x, int16_t *y)
{
	uint8_t idx = -1, n, i, j;
	int16_t d, x_corn, y_corn;
	int16_t x_corn_min = 0, y_corn_min = 0;
	int16_t d_min = 0;

	for (n = 0; n < CORN_NB; n ++) {
		corn_idx_to_coordij(n, &i, &j);
		x_corn = (OFFSET_CORN_X + i*STEP_CORN_X);
		y_corn = (OFFSET_CORN_Y + j*STEP_CORN_Y);
		d = xy_norm(x_corn, y_corn, *x, *y);
		if (d < 200 && (d_min == 0 || d < d_min)) {
			d_min = d;
			idx = n;
			x_corn_min = x_corn;
			y_corn_min = y_corn;
		}
	}
	if (d_min == 0)
		return -1;

	*x = x_corn_min;
	*y = y_corn_min;

	return idx;
}

int8_t corn_get_sym(int8_t i)
{
	if (i >= CORN_NB)
		return -1;
	return corn_sym[i];
}

void init_corn_table(int8_t conf_side, int8_t conf_center)
{
	int8_t sym, i;

	/* before match */
	if (conf_side == -1) {
		for (i=0; i<CORN_NB; i++)
			corn_table[i] = TYPE_UNKNOWN;
		return;
	}

	/* for test */
	printf_P(PSTR("confs = %d, %d\r\n"), conf_side, conf_center);
	for (i=0; i<CORN_NB; i++) {
		if (i == corn_side_confs[conf_side][0] ||
		    i == corn_side_confs[conf_side][1]) {
			corn_table[i] = TYPE_BLACK_CORN;
			continue;
		}
		sym = corn_get_sym(i);
		if (sym == corn_side_confs[conf_side][0] ||
		    sym == corn_side_confs[conf_side][1]) {
			corn_table[i] = TYPE_BLACK_CORN;
			continue;
		}
		if (i == corn_center_confs[conf_center][0] ||
		    i == corn_center_confs[conf_center][1]) {
			corn_table[i] = TYPE_BLACK_CORN;
			continue;
		}
		sym = corn_get_sym(i);
		if (sym == corn_center_confs[conf_center][0] ||
		    sym == corn_center_confs[conf_center][1]) {
			corn_table[i] = TYPE_BLACK_CORN;
			continue;
		}
		corn_table[i] = TYPE_WHITE_CORN;
	}
}

/* init waypoints position */
void init_waypoints(void)
{
	int8_t i, j;
	int32_t x, y;
	struct djpoint *pt;

	x = OFFSET_CORN_X;
	for (i=0; i<WAYPOINTS_NBX; i++) {

		if (i & 1)
			y = OFFSET_CORN_Y - STEP_CORN_Y/2;
		else
			y = OFFSET_CORN_Y;

		for (j=0; j<WAYPOINTS_NBY; j++) {
			pt = &djpoints[i][j];

			pt->pos.x = x;
			pt->pos.y = y;

			pt->type = TYPE_WAYPOINT;
			pt->parent_pos = END;
			pt->updated = 0;
			pt->todo = 0;

			y += STEP_CORN_Y;
		}

		x += STEP_CORN_X;
	}
}

/* update the type and weight of waypoints, before starting
 * dijkstra */
void update_waypoints(void)
{
	int8_t i, j, c;
	struct djpoint *pt;

	for (i=0; i<WAYPOINTS_NBX; i++) {

		for (j=0; j<WAYPOINTS_NBY; j++) {
			pt = &djpoints[i][j];

			pt->weight = 0;

			/* corn */
			c = ijcoord_to_corn_idx(i, j);
			if (c >= 0) {
				pt->type = corn_table[c];
				continue;
			}
			/* too close of border */
			if ((i & 1) == 1 && j == (WAYPOINTS_NBY-1)) {
				pt->type = TYPE_OBSTACLE;
				continue;
			}
			/* hill */
			if (i >= 2 && i < (WAYPOINTS_NBX-2) && j < 2) {
				pt->type = TYPE_OBSTACLE;
				continue;
			}
			/* dangerous points */
			if (i == 0 || i == (WAYPOINTS_NBX-1)) {
				pt->type = TYPE_DANGEROUS;
				continue;
			}
			if ( (i&1) == 0 && j == (WAYPOINTS_NBY-1)) {
				pt->type = TYPE_DANGEROUS;
				continue;
			}
			pt->type = TYPE_WAYPOINT;
		}
	}
}

int get_path(struct djpoint *start, struct djpoint *end)
{
 	struct djpoint *cur;
	uint8_t prev_direction = 0;

	for (cur=start;
	     cur != NULL && cur->parent_pos != END && cur != end;
	     cur = get_neigh(cur, cur->parent_pos)) {
		if (prev_direction != cur->parent_pos) {
			printf_P(PSTR("%d %d (%d)\r\n"),
				 cur->pos.x, cur->pos.y,
				 cur->parent_pos);
		}
		prev_direction = cur->parent_pos;
	}
	printf_P(PSTR("%d %d\r\n"), end->pos.x, end->pos.y);

	return 0; /* XXX */
}

#if 0
int main(void)
{
 	struct djpoint *start;
 	struct djpoint *end;

	start = &djpoints[1][1];
	end = &djpoints[12][1];

	init_corn_table(0, 0);
	init_waypoints();
	update_waypoints();

	dijkstra(end);
	dump();

	get_path(start, end);

	return 0;
}
#endif
