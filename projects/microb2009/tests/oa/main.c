#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>

#include <aversive.h>
#include <aversive/error.h>

#include <vect_base.h>
#include <lines.h>
#include <polygon.h>
#include <obstacle_avoidance.h>

#ifndef HOST_VERSION
#error only for host
#endif

#define M_2PI (M_PI*2)
#define E_USER_STRAT 200

#define ROBOT_X 1000
#define ROBOT_Y 1000
#define ROBOT_A 0.5   /* radian */
#define ROBOT_A_DEG ((int)((ROBOT_A*180)/3.14159))

#define OPP_X 2000
#define OPP_Y 1500

#define DST_X 1500
#define DST_Y 2000

#define PLAYGROUND_X_MIN 250
#define PLAYGROUND_X_MAX 2750
#define PLAYGROUND_Y_MIN 250
#define PLAYGROUND_Y_MAX 1850

int16_t robot_x = ROBOT_X;
int16_t robot_y = ROBOT_Y;
double robot_a = ROBOT_A;
int16_t robot_a_deg = ROBOT_A_DEG;

int16_t opp_x = OPP_X;
int16_t opp_y = OPP_Y;

int16_t dst_x = DST_X;
int16_t dst_y = DST_Y;


#define EDGE_NUMBER 5


double norm(double x, double y)
{
	return sqrt(x*x + y*y);
}

/* return the distance between two points */
int16_t distance_between(int16_t x1, int16_t y1, int16_t x2, int16_t y2)
{
	int32_t x,y;
	x = (x2-x1);
	x = x*x;
	y = (y2-y1);
	y = y*y;
	return sqrt(x+y);
}

void rotate(double *x, double *y, double rot)
{
	double l, a;
	
	l = norm(*x, *y);
	a = atan2(*y, *x);

	a += rot;
	*x = l * cos(a);
	*y = l * sin(a);
}


void set_rotated_pentagon(poly_t *pol, int16_t radius,
			      int16_t x, int16_t y)
{

	double c_a, s_a;
	uint8_t i;
	double px1, py1, px2, py2;
	double a_rad;

	a_rad = atan2(y - robot_y, x - robot_x);


	/* generate pentagon  */
	c_a = cos(-2*M_PI/EDGE_NUMBER);
	s_a = sin(-2*M_PI/EDGE_NUMBER);

	/*
	px1 = radius;
	py1 = 0;
	*/
	px1 = radius * cos(a_rad + 2*M_PI/(2*EDGE_NUMBER));
	py1 = radius * sin(a_rad + 2*M_PI/(2*EDGE_NUMBER));


	for (i = 0; i < EDGE_NUMBER; i++){
		oa_poly_set_point(pol, x + px1, y + py1, i);
		
		px2 = px1*c_a + py1*s_a;
		py2 = -px1*s_a + py1*c_a;

		px1 = px2;
		py1 = py2;
	}

}

void set_rotated_poly(poly_t *pol, int16_t w, int16_t l,
		      int16_t x, int16_t y)
{
	double tmp_x, tmp_y;
	double a_rad;

	a_rad = atan2(y - robot_y, x - robot_x);

	/* point 1 */
	tmp_x = w;
	tmp_y = l;
	rotate(&tmp_x, &tmp_y, a_rad);
	tmp_x += x;
	tmp_y += y;
	oa_poly_set_point(pol, tmp_x, tmp_y, 0);
	
	/* point 2 */
	tmp_x = -w;
	tmp_y = l;
	rotate(&tmp_x, &tmp_y, a_rad);
	tmp_x += x;
	tmp_y += y;
	oa_poly_set_point(pol, tmp_x, tmp_y, 1);
	
	/* point 3 */
	tmp_x = -w;
	tmp_y = -l;
	rotate(&tmp_x, &tmp_y, a_rad);
	tmp_x += x;
	tmp_y += y;
	oa_poly_set_point(pol, tmp_x, tmp_y, 2);
	
	/* point 4 */
	tmp_x = w;
	tmp_y = -l;
	rotate(&tmp_x, &tmp_y, a_rad);
	tmp_x += x;
	tmp_y += y;
	oa_poly_set_point(pol, tmp_x, tmp_y, 3);
}

#define DISC_PENTA_DIAG 500
#define DISC_X 1500
#define DISC_Y 1050

void set_central_disc_poly(poly_t *pol)
{
	set_rotated_pentagon(pol, DISC_PENTA_DIAG,
			     DISC_X, DISC_Y);
}

/* /!\ half size */
#define O_WIDTH  360
#define O_LENGTH 500

void set_opponent_poly(poly_t *pol, int16_t w, int16_t l)
{
	int16_t x, y;
	x = opp_x;
	y = opp_y;
	DEBUG(E_USER_STRAT, "oponent at: %d %d", x, y);
	
	/* place poly even if invalid, because it's -100 */
	set_rotated_poly(pol, w, l, x, y);
}

/* don't care about polygons further than this distance for
 * strat_escape */
#define ESCAPE_POLY_THRES 1000

/* don't reduce opp if opp is too far */
#define REDUCE_POLY_THRES 600

/* has to be longer than any poly */
#define ESCAPE_VECT_LEN 3000

/* go in playground, loop until out of poly */
/* XXX return end timer??? */
static int8_t go_in_area(void)
{
	point_t poly_pts_area[4];
	poly_t poly_area;

	point_t robot_pt, disc_pt, dst_pt;

	robot_pt.x = robot_x;
	robot_pt.y = robot_y;
	disc_pt.x = DISC_X;
	disc_pt.y = DISC_Y;

	/* Go in playground */
	if (!is_in_boundingbox(&robot_pt)){
		NOTICE(E_USER_STRAT, "not in playground %d, %d", robot_x, robot_y);

		poly_area.l = 4;
		poly_area.pts = poly_pts_area;
		poly_pts_area[0].x = PLAYGROUND_X_MIN;
		poly_pts_area[0].y = PLAYGROUND_Y_MIN;

		poly_pts_area[1].x = PLAYGROUND_X_MAX;
		poly_pts_area[1].y = PLAYGROUND_Y_MIN;

		poly_pts_area[2].x = PLAYGROUND_X_MAX;
		poly_pts_area[2].y = PLAYGROUND_Y_MAX;

		poly_pts_area[3].x = PLAYGROUND_X_MIN;
		poly_pts_area[3].y = PLAYGROUND_Y_MAX;

		
		is_crossing_poly(robot_pt, disc_pt, &dst_pt, &poly_area);
		NOTICE(E_USER_STRAT, "pt dst %d, %d", dst_pt.x, dst_pt.y);
		
		/* XXX */
		robot_pt.x = dst_pt.x;
		robot_pt.y = dst_pt.y;

		robot_x = dst_pt.x;
		robot_y = dst_pt.y;

		NOTICE(E_USER_STRAT, "GOTO %d,%d",
		       dst_pt.x, dst_pt.y);

		return 1;
	}

	return 0;
}


/*
 * Escape from polygons if needed.
 */
static int8_t escape_from_poly(poly_t *pol_disc,
			       int16_t opp_x, int16_t opp_y, 
			       int16_t opp_w, int16_t opp_l, 
			       poly_t *pol_opp)
{
	uint8_t in_disc = 0, in_opp = 0;
	double escape_dx = 0, escape_dy = 0;
	double disc_dx = 0, disc_dy = 0;
	double opp_dx = 0, opp_dy = 0;
	double len;

	point_t robot_pt, opp_pt, disc_pt, dst_pt;
	point_t intersect_disc_pt, intersect_opp_pt;

	robot_pt.x = robot_x;
	robot_pt.y = robot_y;
	opp_pt.x = opp_x;
	opp_pt.y = opp_y;
	disc_pt.x = DISC_X;
	disc_pt.y = DISC_Y;

	/* escape from other poly if necessary */
	if (is_point_in_poly(pol_disc, robot_x, robot_y) == 1)
		in_disc = 1;
	if (is_point_in_poly(pol_opp, robot_x, robot_y) == 1)
		in_opp = 1;

	if (in_disc == 0 && in_opp == 0) {
		NOTICE(E_USER_STRAT, "no need to escape");
		return 0;
	}
	
	NOTICE(E_USER_STRAT, "in_disc=%d, in_opp=%d", in_disc, in_opp);
	
	/* process escape vector */

	if (distance_between(robot_x, robot_y, DISC_X, DISC_Y) < ESCAPE_POLY_THRES) {
		disc_dx = robot_x - DISC_X;
		disc_dy = robot_y - DISC_Y;
		NOTICE(E_USER_STRAT, " robot is near disc: vect=%2.2f,%2.2f",
		       disc_dx, disc_dy);
		len = norm(disc_dx, disc_dy);
		if (len != 0) {
			disc_dx /= len;
			disc_dy /= len;
		}
		else {
			disc_dx = 1.0;
			disc_dy = 0.0;
		}
		escape_dx += disc_dx;
		escape_dy += disc_dy;
	}

	if (distance_between(robot_x, robot_y, opp_x, opp_y) < ESCAPE_POLY_THRES) {
		opp_dx = robot_x - opp_x;
		opp_dy = robot_y - opp_y;
		NOTICE(E_USER_STRAT, " robot is near opp: vect=%2.2f,%2.2f",
		       opp_dx, opp_dy);
		len = norm(opp_dx, opp_dy);
		if (len != 0) {
			opp_dx /= len;
			opp_dy /= len;
		}
		else {
			opp_dx = 1.0;
			opp_dy = 0.0;
		}
		escape_dx += opp_dx;
		escape_dy += opp_dy;
	}

	/* normalize escape vector */
	len = norm(escape_dx, escape_dy);
	if (len != 0) {
		escape_dx /= len;
		escape_dy /= len;
	}
	else {
		if (pol_disc != NULL) {
			/* rotate 90° */
			escape_dx = disc_dy;
			escape_dy = disc_dx;
		}
		else if (pol_opp != NULL) {
			/* rotate 90° */
			escape_dx = opp_dy;
			escape_dy = opp_dx;
		}
		else { /* should not happen */
			opp_dx = 1.0;
			opp_dy = 0.0;
		}
	}

	NOTICE(E_USER_STRAT, " escape vect = %2.2f,%2.2f",
	       escape_dx, escape_dy);

	/* process the correct len of escape vector */

	dst_pt.x = robot_pt.x + escape_dx * ESCAPE_VECT_LEN;
	dst_pt.y = robot_pt.y + escape_dy * ESCAPE_VECT_LEN;

	NOTICE(E_USER_STRAT, "robot pt %d %d \r\ndst point %d,%d",
	       robot_pt.x, robot_pt.y, 
	       dst_pt.x, dst_pt.y);

	if (in_disc) {
		if (is_crossing_poly(robot_pt, dst_pt, &intersect_disc_pt,
				     pol_disc) == 1) {
			/* we add 2 mm to be sure we are out of th polygon */
			dst_pt.x = intersect_disc_pt.x + escape_dx * 2;
			dst_pt.y = intersect_disc_pt.y + escape_dy * 2;
			if (is_point_in_poly(pol_opp, dst_pt.x, dst_pt.y) != 1) {

				if (!is_in_boundingbox(&dst_pt))
					return -1;
				
				robot_x = dst_pt.x;
				robot_y = dst_pt.y;

				/* XXX */
				NOTICE(E_USER_STRAT, "GOTO %d,%d (%d)",
				       robot_x, robot_y,
				       is_point_in_poly(pol_opp, robot_x, robot_y));
				return 0;
			}
		}
	}

	if (in_opp) {
		if (is_crossing_poly(robot_pt, dst_pt, &intersect_opp_pt,
				     pol_opp) == 1) {
			/* we add 2 cm to be sure we are out of th polygon */
			dst_pt.x = intersect_opp_pt.x + escape_dx * 2;
			dst_pt.y = intersect_opp_pt.y + escape_dy * 2;

			if (is_point_in_poly(pol_disc, dst_pt.x, dst_pt.y) != 1) {

				if (!is_in_boundingbox(&dst_pt))
					return -1;
				
				robot_x = dst_pt.x;
				robot_y = dst_pt.y;

				/* XXX */
				NOTICE(E_USER_STRAT, "GOTO %d,%d (%d)",
				       robot_x, robot_y,
				       is_point_in_poly(pol_opp, robot_x, robot_y));
				return 0;
			}
		}
	}

	/* should not happen */
	return -1;
}




static int8_t goto_and_avoid(int16_t x, int16_t y)
{
	int8_t len = -1, i;
	point_t *p;
	poly_t *pol_disc, *pol_opp;
	int16_t posx, posy;
	int8_t ret;
	int16_t opp_w;
	int16_t opp_l;
	point_t p_dst;

	opp_w = O_WIDTH;
	opp_l = O_LENGTH;

	posx = robot_x;
	posy = robot_y;
	
	oa_init();
	pol_disc = oa_new_poly(EDGE_NUMBER);
	set_central_disc_poly(pol_disc);
	pol_opp = oa_new_poly(4);
	set_opponent_poly(pol_opp, O_WIDTH, O_LENGTH);

	go_in_area();

	p_dst.x = x;
	p_dst.y = y;
	if (!is_in_boundingbox(&p_dst)) {
		NOTICE(E_USER_STRAT, " dst is not in playground");
		return -1;
	}

	if (is_point_in_poly(pol_disc, x, y)) {
		NOTICE(E_USER_STRAT, " dst is in disc");
		return -1;
	}
	if (is_point_in_poly(pol_opp, x, y)) {
		NOTICE(E_USER_STRAT, " dst is in opp");
		return -1;
	}
	while (opp_w && opp_l) {

		ret = escape_from_poly(pol_disc,
				       opp_x, opp_y, 
				       opp_w, opp_l, 
				       pol_opp);
		if (ret == 0) {
			oa_reset();
			oa_start_end_points(robot_x, robot_y, x, y);
			oa_dump();
	
			len = oa_process();
			if (len >= 0)
				break;
		}
		if (distance_between(robot_x, robot_y, opp_x, opp_y) < REDUCE_POLY_THRES ) {
			if (opp_w == 0)
				opp_l /= 2;
			opp_w /= 2;
		}
		else {
			NOTICE(E_USER_STRAT, "oa_process() returned %d", len);

			/* XXX */
			return -1;
		}

		NOTICE(E_USER_STRAT, "reducing opponent %d %d", opp_w, opp_l);
		set_opponent_poly(pol_opp, opp_w, opp_l);
	}
	
	p = oa_get_path();
	for (i=0 ; i<len ; i++) {
		DEBUG(E_USER_STRAT, "With avoidance %d: x=%d y=%d", i, p->x, p->y);
		p++;
	}
	
	return 0;
}

/* log function, add a command to configure
 * it dynamically */
void mylog(struct error * e, ...) 
{
	va_list ap;

	va_start(ap, e);

	vfprintf(stdout, e->text, ap);
	printf_P(PSTR("\r\n"));
	va_end(ap);
}

#ifdef HOST_VERSION
int main(int argc, char **argv)
#else
int main(void)
#endif
{
#ifdef HOST_VERSION
	if (argc != 8) {
		printf("bad args\n");
		return -1;
	}
	robot_x = atoi(argv[1]);
	robot_y = atoi(argv[2]);
	robot_a_deg = atoi(argv[3]);
	robot_a = (((double)robot_a_deg*M_PI)/3.14159);
	dst_x = atoi(argv[4]);
	dst_y = atoi(argv[5]);
	opp_x = atoi(argv[6]);
	opp_y = atoi(argv[7]);
#endif
	/* LOGS */
	error_register_emerg(mylog);
	error_register_error(mylog);
	error_register_warning(mylog);
	error_register_notice(mylog);
	error_register_debug(mylog);
	
	polygon_set_boundingbox(PLAYGROUND_X_MIN, PLAYGROUND_Y_MIN,
				PLAYGROUND_X_MAX, PLAYGROUND_Y_MAX);

	DEBUG(E_USER_STRAT, "robot at: %d %d %d", robot_x, robot_y, robot_a_deg);
	goto_and_avoid(dst_x, dst_y);

	return 0;
}
