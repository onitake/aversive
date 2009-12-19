#include <inttypes.h>
#include <stdint.h>
#include <math.h>

#include <vect_base.h>
#include <lines.h>
#include <polygon.h>

#include <stdio.h>

#define EPSILON 0.000001

int main(void)
{
	vect_t v, w;
	int32_t ps;
	float n1, n2;
	float a;

	point_t p1, p2, p3, p4, p5, p6, p7, p;
	point_t p8, p9, p10, p11;
	line_t l1, l2, l3, l4;
	uint8_t ret;

	poly_t poly1, poly2;

	point_t poly_pts1[4];
	point_t poly_pts2[5];

	polygon_set_boundingbox(25, 25, 275, 185);

	/* basic vect test */
	v.x = 1;
	v.y = 0;

	vect_rot_trigo(&v);
	if (v.x != 0 || v.y  != 1)
		printf("error rot rigo ok1\r\n");

	vect_rot_trigo(&v);
	vect_rot_trigo(&v);
	vect_rot_trigo(&v);

	if (v.x != 1 || v.y != 0)
		printf("error rot trigo ok2\r\n");

	vect_rot_retro(&v);
	if (v.x != 0 || v.y  != -1)
		printf("error rot rigo ok1\r\n");

	vect_rot_retro(&v);
	vect_rot_retro(&v);
	vect_rot_retro(&v);

	if (v.x != 1 || v.y  != 0)
		printf("error rot retro ok2\r\n");



	w.x = 2;
	w.y = 2;
	ps = vect_pscal(&v, &w);
	
	n1 = vect_norm(&v);
	n2 = vect_norm(&w);
	if (fabs(n1-1.)>EPSILON || fabs(n2-2.828427)>EPSILON)
		printf("error in norm\r\n");
	
	a = acos((float)ps/(n1*n2))*180./M_PI;

	if (fabs(a-45.)>EPSILON)
		printf("error in norm/pscal\r\n");

	a = vect_get_angle(&v, &w)*180./M_PI;
	if (fabs(a-45.)>EPSILON)
		printf("error in get angle\r\n");

	/* basic lines tests */
	p1.x = 0;
	p1.y = 0;

	p2.x = 0;
	p2.y = 10;

	pts2line(&p1, &p2, &l1);
	printf("%2.2f %2.2f %2.2f\r\n", l1.a, l1.b, l1.c);


	p3.x = 10;
	p3.y = 0;

	pts2line(&p1, &p3, &l2);
	printf("%2.2f %2.2f %2.2f\r\n", l2.a, l2.b, l2.c);


	p4.x = 10;
	p4.y = 10;

	p5.x = 20;
	p5.y = 20;

	pts2line(&p4, &p5, &l3);
	printf("%2.2f %2.2f %2.2f\r\n", l3.a, l3.b, l3.c);

	p6.x = 30;
	p6.y = 0;

	p7.x = 0;
	p7.y = 30;

	pts2line(&p6, &p7, &l4);
	printf("%2.2f %2.2f %2.2f\r\n", l4.a, l4.b, l4.c);


	intersect_line(&l1, &l2, &p);
	printf("* %" PRIi32 " %" PRIi32 "\r\n", p.x, p.y);

	intersect_line(&l1, &l4, &p);
	printf("* %" PRIi32 " %" PRIi32 "\r\n", p.x, p.y);

	intersect_line(&l2, &l4, &p);
	printf("* %" PRIi32 " %" PRIi32 "\r\n", p.x, p.y);

	intersect_line(&l3, &l4, &p);
	printf("* %" PRIi32 " %" PRIi32 "\r\n", p.x, p.y);

	ret = intersect_segment(&p1, &p2, &p4, &p5, &p);
	printf("%d (%" PRIi32 " %" PRIi32 ")\r\n", ret, p.x, p.y);
	if (ret != 0)
		printf("error in segment cros\r\n");

	ret = intersect_segment(&p4, &p5, &p6, &p7, &p);
	printf("%d (%" PRIi32 " %" PRIi32 ")\r\n", ret, p.x, p.y);
	if (ret != 1)
		printf("error in segment cros\r\n");

	ret = intersect_segment(&p1, &p2, &p1, &p3, &p);
	printf("%d (%" PRIi32 " %" PRIi32 ")\r\n", ret, p.x, p.y);
	if (ret != 2)
		printf("error in segment cros\r\n");

	p8.x = 105;
	p8.y = 60;
	p9.x = 200;
	p9.y = 150;
	p10.x = 195;
	p10.y = 150;
	p11.x = 195;
	p11.y = 60;
	ret = intersect_segment(&p8, &p9, &p10, &p11, &p);
	printf("%d (%" PRIi32 " %" PRIi32 ")\r\n", ret, p.x, p.y);
	if (ret != 1)
		printf("error in segment cros\r\n");




	p6.x = 30;
	p6.y = 0;

	p7.x = 0;
	p7.y = 30;
	
	p8.x = 10;
	p8.y = 10;

	pts2line(&p6, &p7, &l3);
	proj_pt_line(&p8, &l3, &p);
	printf("proj: %" PRIi32 " %" PRIi32 "\r\n", p.x, p.y);
	if (p.x != 15 || p.y != 15)
		printf("error in proj 1\r\n");


	p6.x = 0;
	p6.y = 0;

	p7.x = 0;
	p7.y = 30;
	
	p8.x = 10;
	p8.y = 10;

	pts2line(&p6, &p7, &l3);
	proj_pt_line(&p8, &l3, &p);
	printf("proj: %" PRIi32 " %" PRIi32 "\r\n", p.x, p.y);
	if (p.x != 0 || p.y != 10)
		printf("error in proj 2\r\n");
	

	p6.x = 30;
	p6.y = 0;

	p7.x = 0;
	p7.y = 0;
	
	p8.x = 10;
	p8.y = 10;

	pts2line(&p6, &p7, &l3);
	proj_pt_line(&p8, &l3, &p);
	printf("proj: %" PRIi32 " %" PRIi32 "\r\n", p.x, p.y);
	if (p.x != 10 || p.y != 0)
		printf("error in proj 3\r\n");


	p1.x = 0;
	p1.y = 10;

	p2.x = 20;
	p2.y = 20;

	pts2line(&p1, &p2, &l1);
	printf("%2.2f %2.2f %2.2f\r\n", l1.a, l1.b, l1.c);

	p2.x = 0;
	p2.y = 10;

	p1.x = 10;
	p1.y = -10;

	pts2line(&p1, &p2, &l1);
	printf("%2.2f %2.2f %2.2f\r\n", l1.a, l1.b, l1.c);


	/* basic poly tests */

	poly1.pts = poly_pts1;
	poly1.l = 4;

	poly2.pts = poly_pts2;
	poly2.l = 5;


	poly1.pts[0].x = 0;
	poly1.pts[0].y = 0;

	poly1.pts[0].x = 1;
	poly1.pts[0].y = 0;

	poly1.pts[0].x = 1;
	poly1.pts[0].y = 1;

	poly1.pts[0].x = 0;
	poly1.pts[0].y = 1;
	


	poly1.pts[0].x = 0;
	poly1.pts[0].y = 0;

	poly1.pts[1].x = 10;
	poly1.pts[1].y = 0;

	poly1.pts[2].x = 20;
	poly1.pts[2].y = 20;

	poly1.pts[3].x = 0;
	poly1.pts[3].y = 10;


	ret =  is_in_poly(&p6, &poly1);
	printf("%d\r\n", ret);
	if (ret!=0)
		printf("error in is in poly\r\n");
	


	ret =  is_in_poly(&p4, &poly1);
	printf("%d\r\n", ret);
	if (ret!=1)
		printf("error in is in poly\r\n");


	ret =  is_in_poly(&p1, &poly1);
	printf("%d\r\n", ret);
	if (ret!=2)
		printf("error in is in poly\r\n");



	
	return 0;


}
