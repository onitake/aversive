#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

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
#define TYPE_WAYPOINT 0
#define TYPE_DANGEROUS 1
#define TYPE_WHITE_CORN 2
#define TYPE_BLACK_CORN 3
#define TYPE_OBSTACLE 4

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

	printf("         ");
	for (i=0; i<WAYPOINTS_NBX; i++) {
		printf(" %2d ", i);
	}
	printf("\n");

	for (j=WAYPOINTS_NBY*2-1; j>=0; j--) {
		printf("%3d   ", j/2);

		if ((j&1) == 0)
			printf("    ");

		for (i=0; i<WAYPOINTS_NBX; i++) {
			pt = &djpoints[i][j/2];

			if (((i+j) & 1) == 0)
				continue;

			if (pt->type == TYPE_OBSTACLE)
				printf("     X  ");
			else if (pt->type == TYPE_DANGEROUS)
				printf("     D  ");
			else if (pt->type == TYPE_WHITE_CORN)
				printf("     W  ");
			else if (pt->type == TYPE_BLACK_CORN)
				printf("     B  ");
 			else if (pt->type == TYPE_WAYPOINT)
				printf(" %5d  ", pt->weight);
 			else
 				printf("     ?  ");
		}
		printf("\n");
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
	printf("at %d %d:\n", i, j);

	NEIGH_FOREACH(neigh, pos, pt) {
		weight = pt->weight + dist(pt, neigh);
		parent_pos = opposite_position(pos);

		/* bonus if we keep the same direction */
		if (parent_pos == pt->parent_pos ||
		    pt->parent_pos == END)
			weight = weight - 1;

		printf("  pos=%d: ppos=%d opos=%d nw=%d w=%d\n", pos,
		       pt->parent_pos, parent_pos,
		       neigh->weight, weight);
		if (neigh->weight == 0 || weight < neigh->weight) {
			djpoint2ij(neigh, &i, &j);
			//printf("    %d,%d updated\n", i, j);
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
		printf("\n");
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

int8_t coord_to_corn_idx(int8_t i, int8_t j)
{
	if (i == 0 && j == 2) return 0;
	if (i == 0 && j == 4) return 1;
	if (i == 0 && j == 6) return 2;
	if (i == 2 && j == 3) return 3;
	if (i == 2 && j == 5) return 4;
	if (i == 2 && j == 7) return 5;
	if (i == 4 && j == 4) return 6;
	if (i == 4 && j == 6) return 7;
	if (i == 6 && j == 5) return 8;
	if (i == 6 && j == 7) return 9;
	if (i == 8 && j == 4) return 10;
	if (i == 8 && j == 6) return 11;
	if (i == 10 && j == 3) return 12;
	if (i == 10 && j == 5) return 13;
	if (i == 10 && j == 7) return 14;
	if (i == 12 && j == 2) return 15;
	if (i == 12 && j == 4) return 16;
	if (i == 12 && j == 6) return 17;
	return -1;
}

int8_t corn_get_sym(int8_t i)
{
	if (i >= CORN_NB)
		return -1;
	return corn_sym[i];
}

void init_corn_table(uint8_t conf_side, uint8_t conf_center)
{
	int8_t sym, i;

	printf("confs = %d, %d\n", conf_side, conf_center);
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
			c = coord_to_corn_idx(i, j);
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
		if (prev_direction != cur->parent_pos)
			printf("%d %d (%d)\n", cur->pos.x, cur->pos.y, cur->parent_pos);
		prev_direction = cur->parent_pos;
	}
	printf("%d %d\n", end->pos.x, end->pos.y);

	return 0; /* XXX */
}

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
