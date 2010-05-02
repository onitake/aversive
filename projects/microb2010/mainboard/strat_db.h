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

#define WAYPOINTS_NBX 13
#define WAYPOINTS_NBY 8

struct corn_db {
	/* I2C_COB_UNKNOWN, I2C_COB_WHITE, I2C_COB_BLACK */
	uint8_t color:2;
	uint8_t reserved:6;

	/* index in corn table */
	uint8_t idx;
};

struct tomato_db {
	uint8_t idx;
};

/* structure describing the status of a waypoint */
struct waypoint_db {
	/* type of the waypoint */
#define WP_TYPE_WAYPOINT 0 /* no object on it */
#define WP_TYPE_OBSTACLE 1 /* cannot reach this point */
#define WP_TYPE_TOMATO   2 /* place for a tomato */
#define WP_TYPE_CORN     3 /* place for a corn */
	uint8_t type:2;

	/* true if point is near the border */
	uint8_t dangerous:1;

	/* true if element is present */
	uint8_t present:1;

	/* visited by opponent */
	uint8_t opp_visited:1;

	uint8_t reserved:3;

	/* absolute position of the waypoint */
/* 	int16_t x; */
/* 	int16_t y; */

	union {
		struct corn_db corn;
		struct tomato_db tomato;
	};
};

/* database reflecting the status of objects on area */
struct strat_db {
	uint8_t dump_enabled;

	/* table of waypoints */
	struct waypoint_db wp_table[WAYPOINTS_NBX][WAYPOINTS_NBY];

	/* corn_table: pointers to waypoints */
	struct waypoint_db *corn_table[CORN_NB];

	/* tomato_table: pointers to waypoints */
	struct waypoint_db *tomato_table[TOMATO_NB];

	/* number of oranges remaining */
	uint8_t our_oranges_count;
	uint8_t opp_oranges_count;
};

/* global structure storing the database */
extern struct strat_db strat_db;

/* return the nearest waypoint that is not a corn: xp and yp contains
 * the input and output, and ip, jp are only outputs. return 0 on
 * success. */
int8_t xycoord_to_ijcoord(int16_t *xp, int16_t *yp, uint8_t *ip, uint8_t *jp);

/* convert i,j coords to x,y coords */
int8_t ijcoord_to_xycoord(uint8_t i, uint8_t j, int16_t *x, int16_t *y);

/* return the index of a corn given its i,j coords. */
int8_t ijcoord_to_corn_idx(uint8_t i, uint8_t j);

/* return the i,j coords of a corn given its index */
int8_t corn_idx_to_ijcoord(uint8_t idx, uint8_t *i, uint8_t *j);

/* return the index of a corn given its x,y coords. */
int8_t corn_idx_to_xycoord(uint8_t idx, int16_t *x, int16_t *y);

/* return the index of the closest corn at these coordinates. If the
 * corn is really too far (~20cm), return NULL. The x and y pointer are
 * updated with the real position of the corn */
struct waypoint_db *xycoord_to_corn_idx(int16_t *x, int16_t *y);

/* set color of a corn
 * type is I2C_COB_BLACK, I2C_COB_WHITE, I2C_COB_UNKNOWN
 * it will update the symetric corn if != UNKOWN
 * it will also deduct color of some other cobs */
void corn_set_color(struct waypoint_db *wp, uint8_t color);

/* return the idx of the symetric corn */
int8_t corn_get_sym_idx(int8_t i);

/*********** TOMATO */

/* return the index of a tomato given its i,j coords. */
int8_t ijcoord_to_tomato_idx(uint8_t i, uint8_t j);

/* return the i,j coords of a tomato given its index */
int8_t tomato_idx_to_ijcoord(uint8_t idx, uint8_t *i, uint8_t *j);

/* return the index of a tomato given its x,y coords. */
int8_t tomato_idx_to_xycoord(uint8_t idx, int16_t *x, int16_t *y);

/* return the index of the closest tomato at these coordinates. If the
 * tomato is really too far (~20cm), return NULL. The x and y pointer are
 * updated with the real position of the tomato */
struct waypoint_db *xycoord_to_tomato_idx(int16_t *x, int16_t *y);

/*
 * Init internal database. The initialization is done with UNKNOWN
 * corn with all objects present
 */
void strat_db_init(void);

/* dump infos about area and objects */
void strat_db_dump(const char *caller);
