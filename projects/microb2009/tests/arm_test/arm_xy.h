#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <inttypes.h>

#include <aversive.h>

#define ARM_L 130UL
#define ARM_H 250UL

#define DIM_COEF  50
#define DIM_D  6  /* distance */
#define DIM_H  10 /* height */

/* coefs to fit in a integer */
#define INT_M_SHOULDER 3
#define INT_M_ELBOW 3

#define sqrtf sqrt
#define acosf acos
#define atan2f atan2

typedef struct arm_pts
{
	int16_t p_shoulder;
	uint8_t p_elbow;
} arm_pts_t;


typedef struct arm_pos{
	int32_t h;
	int32_t d;
	int32_t w;

	int32_t goal_h;
	int32_t goal_d;
	int32_t goal_w;


	volatile int8_t state;
	
	void (*wrist_angle_deg2robot)(double wrist_edg,
				      double *wrist_out);

	void (*angle_rad2robot)(double shoulder_deg, double elbow_deg,
				double *shoulder_out, double *elbow_out);

	void (*angle_robot2rad)(double shoulder_robot, double elbow_robot,
				double *shoulder_rad, double *elbow_rad);

	int8_t ELBOW_AX12;
	int8_t WRIST_AX12;

	
}Arm_Pos;



/* process shoulder + elbow angles from height and distance */
int8_t cart2angle(int32_t h, int32_t d, double *alpha, double *beta);

/* process height and distance from shoulder + elbow angles */
void angle2cart(double alpha, double beta, int32_t *h, int32_t *d);

void wrist_angle_deg2robot_r(double wrist_edg,
			     double *wrist_out);

void wrist_angle_deg2robot_l(double wrist_edg,
			     double *wrist_out);

/* convert an angle in degree into a robot-specific unit 
 * for shoulder and elbow */
void angle_rad2robot_r(double shoulder_deg, double elbow_deg,
		       double *shoulder_out, double *elbow_out);

void angle_rad2robot_l(double shoulder_deg, double elbow_deg,
		       double *shoulder_out, double *elbow_out);


/* convert  a robot-specific unit into an angle in radian 
 * for shoulder and elbow */
void angle_robot2rad_r(double shoulder_robot, double elbow_robot,
		       double *shoulder_rad, double *elbow_rad);
void angle_robot2rad_l(double shoulder_robot, double elbow_robot,
		       double *shoulder_rad, double *elbow_rad);


/* generate the angle matrix for each (d,h) position */
void init_arm_matrix(void);
void dump_matrix(void);

/* same than cart2angle, but uses a bilinear interpolation */
int8_t coord2ij(int32_t d, int32_t h, int32_t *d_o, int32_t *h_o);

int8_t arm_get_coord(int16_t d,int16_t h, int32_t *d_o, int32_t *h_o);

void test_arm_pos(void);

/*
int8_t arm_do_step(int32_t *arm_h, int32_t *arm_d, int32_t fin_h, int32_t fin_d, 
		   int32_t *as, int32_t *ae, uint32_t *next_time,
		   int32_t *s_quad);
*/

int8_t arm_do_step(Arm_Pos *arm_pos,
		   int32_t *arm_h, int32_t *arm_d, int32_t fin_h, int32_t fin_d, 
		   int32_t *as, int32_t *ae, int32_t *aw, 
		   double *as_fin_rad, double *ae_fin_rad,
		   uint32_t *next_time,
		   int32_t *s_quad, int32_t *e_quad);

#if 0

int main(void)
{
	double a, b;
	int8_t ret;
	int32_t as, ae;
	ret = cart2angle(100, 100, &a, &b);
	printf("ret %d %f %f\n", ret, a*180/M_PI, b*180/M_PI);
  
	init_arm_matrix();
	dump_matrix();
  
  
	int32_t px = 100;
	int32_t py = -230;
	ret = coord2ij(px,py, &as, &ae);
	printf("ret %d %ld %ld\n", ret, as, ae);
  
  
	ret = cart2angle(py, px, &a, &b);
	printf("ret %d %f %f\n", ret, a*180/M_PI, b*180/M_PI);
  
	ret = arm_get_coord(px,py, &as, &ae);
	printf("ret %d %ld %ld\n", ret, as, ae);
  
	test_arm_pos();
	return 0;
}
#endif
