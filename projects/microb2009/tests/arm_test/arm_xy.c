#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <inttypes.h>

#include <aversive.h>

#include <aversive/wait.h>


#include <uart.h>
#include <i2c.h>
#include <ax12.h>
#include <parse.h>
#include <rdline.h>
#include <pwm_ng.h>
#include <encoders_microb.h>
#include <timer.h>
#include <scheduler.h>
#include <pid.h>
#include <time.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <adc.h>
#include <spi.h>

#include "main.h"

#include "arm_xy.h"

arm_pts_t mat_angle[DIM_D][DIM_H];



/* process shoulder + elbow angles from height and distance */
int8_t cart2angle(int32_t h, int32_t d, double *alpha, double *beta)
{
	double theta, l, phi;

	//l = SquareRootDouble((double)(d*d + h*h));
	l = sqrt((double)(d*d + h*h));
	if  (l>2*ARM_L)
		return -1;
	theta = atan2f((double)h, (double)d);
	phi = acosf(l/(2*ARM_L));
    
	*alpha = theta+phi;
	*beta = -2*phi;

	return 0;
}


/* process height and distance from shoulder + elbow angles */
void angle2cart(double alpha, double beta, int32_t *h, int32_t *d)
{
	double tmp_a;
	int32_t tmp_h, tmp_d;

	tmp_h = ARM_L * sin(alpha);
	tmp_d = ARM_L * cos(alpha);
	
	tmp_a = alpha+beta;
	*h = tmp_h + ARM_L * sin(tmp_a);
	*d = tmp_d + ARM_L * cos(tmp_a);
	
}

void wrist_angle_deg2robot_r(double wrist_deg,
                     double *wrist_out)
{
	*wrist_out = wrist_deg * 3.41  + 875;
}


void wrist_angle_deg2robot_l(double wrist_deg,
                     double *wrist_out)
{
	*wrist_out = wrist_deg * 3.41  + 40;
}


#define ARM_S_OFFSET -16000
#define ARM_E_OFFSET 660

/* convert an angle in radian into a robot-specific unit 
 * for shoulder and elbow */
void angle_rad2robot_r(double shoulder_rad, double elbow_rad,
                     double *shoulder_robot, double *elbow_robot)
{
	*shoulder_robot = -shoulder_rad * 4 * 66 * 512. / (2*M_PI) + ARM_S_OFFSET;
	*elbow_robot = elbow_rad * 3.41 * 360. / (2*M_PI) + ARM_E_OFFSET;
}


#define ARM_LEFT_S_OFFSET -16000
#define ARM_LEFT_E_OFFSET 470


/* convert an angle in radian into a robot-specific unit 
 * for shoulder and elbow for LEFT ARM*/
void angle_rad2robot_l(double shoulder_rad, double elbow_rad,
		       double *shoulder_robot, double *elbow_robot)
{
	*shoulder_robot = -shoulder_rad * 4 * 66 * 512. / (2*M_PI) + ARM_LEFT_S_OFFSET;
	*elbow_robot = elbow_rad * 3.41 * 360. / (2*M_PI) + ARM_LEFT_E_OFFSET;
}



/* convert  a robot-specific unit into an angle in radian 
 * for shoulder and elbow */
void angle_robot2rad_r(double shoulder_robot, double elbow_robot,
		     double *shoulder_rad, double *elbow_rad)
{
	*shoulder_rad = -((shoulder_robot - ARM_S_OFFSET)*(2*M_PI))/(4 * 66 * 512.);
	*elbow_rad = ((elbow_robot - ARM_E_OFFSET) * (2*M_PI))/(3.41 * 360.);		
}

/* convert  a robot-specific unit into an angle in radian 
 * for shoulder and elbow for LEFT ARM*/
void angle_robot2rad_l(double shoulder_robot, double elbow_robot,
		     double *shoulder_rad, double *elbow_rad)
{
	*shoulder_rad = -((shoulder_robot - ARM_LEFT_S_OFFSET)*(2*M_PI))/(4 * 66 * 512.);
	*elbow_rad = ((elbow_robot - ARM_LEFT_E_OFFSET) * (2*M_PI))/(3.41 * 360.);		
}


/* generate the angle matrix for each (d,h) position */
void init_arm_matrix(void)
{
	int8_t d, h, ret;
	double a, b;
  
	for (d = 0; d < DIM_D; d++){
		for (h = 0; h < DIM_H; h++){
			ret = cart2angle((h * DIM_COEF - ARM_H), d * DIM_COEF, &a, &b);
			if (ret) {
				mat_angle[d][h].p_shoulder = 0;
				mat_angle[d][h].p_elbow = 0;
			}
			else {
				angle_rad2robot_r(a, b, &a, &b);
				mat_angle[d][h].p_shoulder = a / INT_M_SHOULDER;
				mat_angle[d][h].p_elbow = b / INT_M_ELBOW;
			}      
		}
	}
}

void dump_matrix(void)
{
	int8_t d, h;
	for (h = 0; h < DIM_H; h++) {
		for (d = 0; d < DIM_D; d++) {
			printf("%6d %4d ", mat_angle[d][h].p_shoulder*INT_M_SHOULDER, mat_angle[d][h].p_elbow*INT_M_ELBOW);
		}
		printf("\n");
	}
}

/* same than cart2angle, but uses a bilinear interpolation */
int8_t coord2ij(int32_t d, int32_t h, int32_t *d_o, int32_t *h_o)
{
	int8_t i_o, j_o;
    
	int32_t a00, a10, a11, a01;
	uint16_t b00, b10, b11, b01;
    
	int32_t poids1, poids2;
	int32_t atmp1, atmp2, btmp1, btmp2;
	
	if (d*d + h*h >(ARM_L*2)*(ARM_L*2))
		return -6; 
    
	i_o = d / DIM_COEF;
	j_o = (h+ARM_H) / DIM_COEF;
    
	if (i_o >= DIM_D || j_o >= DIM_H || i_o < 0 || j_o < 0)
		return -1;

	a00 = mat_angle[i_o][j_o].p_shoulder * INT_M_SHOULDER;
	b00 = mat_angle[i_o][j_o].p_elbow * INT_M_ELBOW;
	if (!(a00 || b00))
		return -2;
	a10 = mat_angle[i_o+1][j_o].p_shoulder * INT_M_SHOULDER;
	b10 = mat_angle[i_o+1][j_o].p_elbow * INT_M_ELBOW;
	if (!(a10 || b10))
		return -3;
    
	a11 = mat_angle[i_o+1][j_o+1].p_shoulder * INT_M_SHOULDER;
	b11 = mat_angle[i_o+1][j_o+1].p_elbow * INT_M_ELBOW;
	if (!(a11 || b11))
		return -4;
    
	a01 = mat_angle[i_o][j_o+1].p_shoulder * INT_M_SHOULDER;
	b01 = mat_angle[i_o][j_o+1].p_elbow * INT_M_ELBOW;
	if (!(a01 || b01))
		return -5;
    
	poids1 = d - DIM_COEF * i_o;
	poids2 = DIM_COEF * (i_o+1) - d;
    
	atmp1 = (poids1*a10 + poids2*a00)/DIM_COEF;
	btmp1 = (poids1*b10 + poids2*b00)/DIM_COEF;

	atmp2 = (poids1*a11 + poids2*a01)/DIM_COEF;
	btmp2 = (poids1*b11 + poids2*b01)/DIM_COEF;
    
	poids1 = (h+ARM_H) - DIM_COEF*j_o;
	poids2 = DIM_COEF*(j_o+1) - (h+ARM_H);

	*d_o = (poids1*atmp2 + poids2*atmp1) / DIM_COEF;
	*h_o = (poids1*btmp2 + poids2*btmp1) / DIM_COEF;
    
	return 0;
}


int8_t arm_get_coord(int16_t d,int16_t h, int32_t *d_o, int32_t *h_o)
{
	int8_t ret;
	double a, b;
  
	ret = coord2ij(d,h, d_o,h_o);
	if (!ret)
		return ret;
      
	ret = cart2angle(h, d, &a, &b);
	if (ret<0)
		return ret;
      
	angle_rad2robot_r(a, b, &a, &b);
	*d_o = a;
	*h_o = b;  
	return ret;

}

void test_arm_pos(void)
{
	int16_t d, h;
	int8_t ret;
	int32_t as, ae;
  
	for (d=-300; d<300; d+=25){
		for (h=-300; h<300; h+=25){
			ret = arm_get_coord(d,h, &as, &ae);
			if (ret < 0)
				as = ae = 0;
			printf("%7ld %4ld ", as, ae);
		}
		printf("\n");
	}
}
/*
#define ARM_PERIOD 50000

#define ARM_MAX_DIST 80
#define ARM_MAX_E 10000
#define ARM_MAX_S 30000
*/

//100000 us for max shoulder / max elbow
//en us
#define ARM_PERIOD 50000L

#define ARM_MAX_DIST 40L
// 2331 step/s => 
//#define ARM_MAX_E ((2331L*ARM_PERIOD)/1000000L)
/*
// 800step/CS => 160step/ms
#define ARM_MAX_S ((160L*ARM_PERIOD)/1000L)
*/
// 4000step/CS => 800step/ms
//#define ARM_MAX_S ((800L*ARM_PERIOD)/1000L)



/* TEST TEST XXX */
#define ARM_MAX_E (((1500L*ARM_PERIOD)/1000000L))
#define ARM_MAX_S (((1500L*ARM_PERIOD)/1000L))




/* return:
   0 => at last pos
   >0 => need more step
   <0 => unreachable pos
*/
int8_t arm_do_step(Arm_Pos *arm_pos,
		   int32_t *arm_h, int32_t *arm_d, int32_t fin_h, int32_t fin_d, 
		   int32_t *as, int32_t *ae, int32_t *aw,
		   double *as_fin_rad, double *ae_fin_rad,
		   uint32_t *next_time,
		   int32_t *s_quad, int32_t *e_quad)
{
	int8_t ret;
	int32_t diff_h, diff_d, l;
	double as_fin, ae_fin, as_cur, ae_cur, as_cur_rad, ae_cur_rad;//, as_fin_rad, ae_fin_rad;
	double as_coef, ae_coef;
	int32_t as_diff, ae_diff;

	
	diff_h = fin_h-*arm_h;
	diff_d = fin_d-*arm_d;

	//printf("1 dh %ld dd %ld\r\n", diff_h, diff_d);

	l = sqrt(diff_h*diff_h + diff_d*diff_d);
	
	if (l>ARM_MAX_DIST){
		diff_h = diff_h*ARM_MAX_DIST/l;
		diff_d = diff_d*ARM_MAX_DIST/l;
	}

	//printf("2 l: %ld dh %ld dd %ld\r\n", l, diff_h, diff_d);
	

	fin_h = *arm_h+diff_h;
	fin_d = *arm_d+diff_d;

	//printf("2 fh %ld fd %ld\r\n", fin_h, fin_d);

	/* calc for current angle */
	ret = cart2angle(*arm_h, *arm_d, &as_cur_rad, &ae_cur_rad);
	if (ret)
		return ret;

	/* calc for next step */
	ret = cart2angle(fin_h, fin_d, as_fin_rad, ae_fin_rad);
	if (ret)
		return ret;



	//printf("3 as_cur %f ae_cur %f \r\n", as_cur_rad, ae_cur_rad);
	//printf("4 as_fin %f ae_fin %f \r\n", *as_fin_rad, *ae_fin_rad);


	arm_pos->angle_rad2robot(as_cur_rad, ae_cur_rad,
				 &as_cur, &ae_cur);
	arm_pos->angle_rad2robot(*as_fin_rad, *ae_fin_rad,
				 &as_fin, &ae_fin);


	//printf("5 as_cur %f ae_cur %f \r\n", as_cur, ae_cur);
	//printf("6 as_fin %f ae_fin %f \r\n", as_fin, ae_fin);


	as_diff = as_fin-as_cur;
	ae_diff = ae_fin-ae_cur;

	//printf("7 asdiff %ld aediff %ld \r\n", as_diff, ae_diff);


	*arm_h = fin_h;
	*arm_d = fin_d;

	/* XXX we are at pos, set default speed */
	*s_quad = 800;
	*e_quad = 0x3ff;

	if (!as_diff && !ae_diff){
		/* printf("reaching end\r\n"); */
		*as = as_fin;
		*ae = ae_fin;
		*next_time = 0;
		

		return 0;
	}

	/* test if one activator is in position */
	if (as_diff == 0){
		//printf("as reached\r\n");
		ae_coef = (double)ARM_MAX_E/(double)ae_diff;
		*next_time = ARM_PERIOD*ABS(ae_coef);

		*e_quad = ABS(ae_coef)*ABS(ae_diff);

		*as = as_cur;
		*ae = ae_cur+ae_diff;


	}
	else if (ae_diff == 0){
		//printf("ae reached\r\n");
		as_coef = (double)ARM_MAX_S/(double)as_diff;
		*next_time = ARM_PERIOD/ABS(as_coef);

	
		*s_quad = ABS(as_coef)*ABS(as_diff);
	
		*as = as_cur+as_diff;
		*ae = ae_cur;
	}

	else{
	    as_coef = (double)ARM_MAX_S/(double)as_diff;
	    ae_coef = (double)ARM_MAX_E/(double)ae_diff;
	    
	    //printf("asc %f aec %f\r\n", as_coef, ae_coef);
	    
	    *as = as_cur+as_diff;
	    *ae = ae_cur+ae_diff;
	    
	    
	    if (ABS(as_coef)>=ABS(ae_coef)){
	    	/* elbow is limitating */
	    
	    	//printf("e limit %ld %ld \r\n", as_diff, ae_diff);
	    
	    	*next_time = ARM_PERIOD/ABS(ae_coef);
	    	//*next_time = ARM_PERIOD*ae_diff/ARM_MAX_E;
	    
	    
	    	*s_quad = ABS(ae_coef)*ABS(as_diff);
	    	*e_quad = ABS(ae_coef)*ABS(ae_diff);
	    
	    
	    }
	    else{
	    	/* shoulder is limitating */
	    
	    	//printf("s limit %ld %ld \r\n", as_diff, ae_diff);
	    
	    	*next_time = ARM_PERIOD/ABS(as_coef);
	    
	    	*s_quad = ABS(as_coef)*ABS(as_diff);
	    	*e_quad = ABS(as_coef)*ABS(ae_diff);
	    
	    
    	    }
	}

	*s_quad = (*s_quad*CS_PERIOD)/ARM_PERIOD;
	*e_quad = (0x3ff*(*e_quad))/ARM_MAX_E;

	//printf("max e %ld\r\n", *e_quad);
	/*avoid limits*/
	if (!*s_quad)
		*s_quad = 1;
	if (!*e_quad)
		*e_quad = 1;


	//printf("8 sq %ld eq %ld \r\n", *s_quad, *e_quad);

	return 1;
	
}


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
