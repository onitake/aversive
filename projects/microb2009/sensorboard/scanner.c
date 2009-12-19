#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <aversive.h>
#include <aversive/wait.h>
#include <aversive/error.h>
#include <aversive/pgmspace.h>
#include <pwm_ng.h>
#include <pid.h>
#include <time.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <adc.h>
#include <spi.h>
#include <ax12.h>

#include <time.h>
#include <blocking_detection_manager.h>

#include <encoders_spi.h>

#include <rdline.h>

#include "sensor.h"

#include <uart.h>
 
#include "main.h"
#include "scanner.h"

#include "cmdline.h"


#include "scan_h_l.h"


#include <vect_base.h>
#include "img_processing.h"


#include "../common/i2c_commands.h"

#define SCANNER_DEBUG(args...) DEBUG(E_USER_SCANNER, args)
#define SCANNER_NOTICE(args...) NOTICE(E_USER_SCANNER, args)
#define SCANNER_ERROR(args...) ERROR(E_USER_SCANNER, args)


#define MODULO_TIMER (1023L)

#define COEFF_TIMER (2)
#define COEFF_MULT (1000L)
#define COEFF_MULT2 (1000L)

double  TELEMETRE_A =  TELEMETRE_A_INIT;
double  TELEMETRE_B = TELEMETRE_B_INIT;


                                                       
                                                       
struct scan_params scan_params;

static volatile int32_t scan_delta_pos;
static volatile int32_t scan_tick_cur = 0;
static volatile int32_t scan_tick_prev = 0;
/*static volatile int32_t count = 0;
static volatile int32_t count_diff_rising  = 0;
static volatile int32_t count_diff_falling = 0;
*/
static int32_t scanner_coeff = 0;

//static volatile int8_t valid_scanner = 0;

static volatile int32_t scan_pos_cur = 0;

static int32_t pos_ref = 0;


int32_t encoders_spi_get_value_scanner(void *number)
{
	int32_t ret;

	ret = encoders_spi_get_value(number);
	return ret*4;
}


void encoders_spi_set_value_scanner(void * number, int32_t v)
{
	encoders_spi_set_value(number, v/4);
} 

int32_t encoders_spi_update_scanner(void * number)
{
	int32_t ret;
	uint8_t flags;
	
	IRQ_LOCK(flags);
	ret = encoders_spi_get_value_scanner(number);
	scan_delta_pos = ret - scan_pos_cur;
	scan_pos_cur = ret;
	scan_tick_prev = scan_tick_cur;
	scan_tick_cur = TCNT3;

	scanner_coeff = (scan_delta_pos * COEFF_MULT) / 
	  ((scan_tick_cur - scan_tick_prev  + MODULO_TIMER + 1) & MODULO_TIMER);

	IRQ_UNLOCK(flags);
	

	return ret;
}

int32_t encoders_spi_get_value_scanner_interpolation(void * number)
{
	uint8_t flags;
	int32_t pos;

	IRQ_LOCK(flags);
	pos = scan_pos_cur;
	pos += (scanner_coeff * ((TCNT3 - scan_tick_cur + MODULO_TIMER + 1)& MODULO_TIMER ))/
	  COEFF_MULT;

	IRQ_UNLOCK(flags);
	
	return pos;
}


void scanner_reset_pos(void)
{
	pwm_ng_set(SCANNER_PWM, 0);
	encoders_spi_set_value_scanner(SCANNER_ENCODER, 0);
}

void scanner_init(void)
{

	scan_params.working = 0;
	scan_params.must_stop = 0;

	scanner_reset_pos();
	pos_ref = encoders_spi_get_value_scanner(SCANNER_ENCODER);

	//memset(&scanner, 0, sizeof(struct scanner));

	scan_delta_pos = 0;

	/*for(i=0;i<SCANNER_MAX_SAMPLE;i++)
		scanner_sample_size[i] = 0;*/



}


#define SCANNER_OFFSET_CALIBRE 1500

#define CALIBRE_LASER_SAMPLE 100






void scanner_calibre_laser(void)
{
	unsigned int i;
	int32_t laser_value = 0;

	/* arm out */
	pwm_ng_set(&gen.servo3, SCANNER_POS_OUT);

	/* set mirror to have vertical laser */
	cs_set_consign(&sensorboard.scanner.cs, (35L*SCANNER_STEP_TOUR)/100);

	wait_ms(500);

	/* 
	   laser must point on the ground:
	   we sample laser in order to determine 
	   laser cold/warm calibration
	*/
	
	for (i = 0; i<CALIBRE_LASER_SAMPLE; i++){
		laser_value += 	adc_get_value( ADC_REF_AVCC | MUX_ADC13 );
		wait_ms(2);
	}
	
	laser_value/=CALIBRE_LASER_SAMPLE;

	SCANNER_NOTICE("laser calibration value %"PRIi32"", laser_value);

	quadramp_set_1st_order_vars(&sensorboard.scanner.qr, scan_params.speed, scan_params.speed); /* set speed */
	cs_set_consign(&sensorboard.scanner.cs, 0);
	wait_ms(200);


	TELEMETRE_B = TELEMETRE_B_INIT + ((double)(424 - laser_value))*6./90.;

	SCANNER_NOTICE("TEL B value %2.2f", TELEMETRE_B);
	

	/* arm in */

	pwm_ng_set(&gen.servo3, SCANNER_POS_IN);


}





void scanner_calibre_mirror(void)
{

	sensorboard.flags &= ~DO_CS;



	/* set arm in calibre position */	
	pwm_ng_set(&gen.servo3, SCANNER_POS_CALIBRE);
	wait_ms(500);



	/* init scanner pos */
	pwm_ng_set(SCANNER_PWM, 100);

	/* find rising edge of the mirror*/
	wait_ms(100);
	while (sensor_get(SCANNER_POS_SENSOR));
	wait_ms(100);
	while (!sensor_get(SCANNER_POS_SENSOR));

	pwm_ng_set(SCANNER_PWM, 0);


	scanner_reset_pos();
	pid_reset(&sensorboard.scanner.pid);
	cs_set_consign(&sensorboard.scanner.cs, 0);


	quadramp_reset(&sensorboard.scanner.qr);


	sensorboard.flags |= DO_CS;


	/* 
	   set mirror to set laser point at maximum 
	   distance from robot 
	*/
	encoders_spi_set_value_scanner(SCANNER_ENCODER, -SCANNER_OFFSET_CALIBRE);
	wait_ms(100);
	
	/* arm in */

	pwm_ng_set(&gen.servo3, SCANNER_POS_IN);

}


/* arm must be in OUT position! */
void scanner_do_scan(void){
	scan_params.working = 1;
	scan_params.dropzone_h = -1;


	quadramp_set_1st_order_vars(&sensorboard.scanner.qr, scan_params.speed, scan_params.speed); /* set speed */

	scan_params.sample_i = MAX_SAMPLE;
	scan_params.pos_start_scan = encoders_spi_get_value_scanner(SCANNER_ENC);
	
	memset(scan_params.sample_tab, 0xff, MAX_SAMPLE*sizeof(uint8_t));
	
	cs_set_consign(&sensorboard.scanner.cs, scan_params.pos_start_scan+SCANNER_STEP_TOUR*200L);
	
	scan_params.last_col_n = 0;
	scan_params.last_row_n = 0;
	scan_params.last_sample = 0;

	
}


void scanner_end_process(void)
{
	int16_t dropzone_x, dropzone_y;
	int8_t ret;
	uint32_t t1, t2;

	t1 = time_get_us2();


	SCANNER_NOTICE("process img algo %d", scan_params.algo);
	if (scan_params.algo == I2C_SCANNER_ALGO_COLUMN_DROPZONE) {
		SCANNER_NOTICE("column dropzone h: %d x:%d y:%d", scan_params.drop_zone.working_zone,
			       scan_params.drop_zone.center_x, scan_params.drop_zone.center_y);
	
		ret = get_column_dropzone(scan_params.sample_tab, PIX_PER_SCAN, MAX_SAMPLE/PIX_PER_SCAN, 
					  scan_params.drop_zone.working_zone, scan_params.drop_zone.center_x, scan_params.drop_zone.center_y,
					  &dropzone_x, &dropzone_y);

		scan_params.dropzone_h = ret;
		scan_params.dropzone_x = dropzone_x;
		scan_params.dropzone_y = dropzone_y;

		SCANNER_NOTICE("best column h:%d x:%d y:%d", 
			 scan_params.dropzone_h,
			 scan_params.dropzone_x, scan_params.dropzone_y);
	
		
	}
	else if (scan_params.algo == I2C_SCANNER_ALGO_CHECK_TEMPLE){
		SCANNER_NOTICE("checktemple h: %d x:%d y:%d", scan_params.check_temple.level, 
			       scan_params.check_temple.temple_x, scan_params.check_temple.temple_y);
		ret = is_temple_there(scan_params.sample_tab, PIX_PER_SCAN, MAX_SAMPLE/PIX_PER_SCAN, 
				      scan_params.check_temple.level, scan_params.check_temple.temple_x, scan_params.check_temple.temple_y);

		scan_params.dropzone_h = ret?scan_params.check_temple.level:-1;
	}
	else if (scan_params.algo == I2C_SCANNER_ALGO_TEMPLE_DROPZONE){
		SCANNER_NOTICE("temple dropzone  h: %d x:%d y:%d", scan_params.drop_zone.working_zone, 
			       scan_params.drop_zone.center_x, scan_params.drop_zone.center_y);
		ret = find_temple_dropzone(scan_params.sample_tab, PIX_PER_SCAN, MAX_SAMPLE/PIX_PER_SCAN, 
					   scan_params.drop_zone.working_zone, scan_params.drop_zone.center_x, scan_params.drop_zone.center_y,
					   &dropzone_x, &dropzone_y);

		scan_params.dropzone_h = ret;
		scan_params.dropzone_x = dropzone_x;
		scan_params.dropzone_y = dropzone_y;

		SCANNER_NOTICE("best temple h:%d x:%d y:%d", 
			 scan_params.dropzone_h,
			 scan_params.dropzone_x, scan_params.dropzone_y);

	}

	scan_params.working = 0;

	t2 = time_get_us2();
	SCANNER_NOTICE("process total time %"PRIi32"",t2-t1); 
	
		
}


void scanner_scan_autonomous(void)
{
	/* arm out*/
	pwm_ng_set(&gen.servo3, SCANNER_POS_OUT);
	time_wait_ms(300);
	
	scanner_do_scan();

	while(scan_params.sample_i > 0){
		time_wait_ms(10);
	}

	/* arm in */
	pwm_ng_set(&gen.servo3, SCANNER_POS_IN);

}


/* 
 * called from IRQ:
 * mode can be off/prepare/start, see in i2c_commands.h
 */
void scanner_set_mode(uint8_t mode)
{
	if (mode == I2C_SENSORBOARD_SCANNER_PREPARE){
		/* reset flag max pos */
		scan_params.max_column_detected = 0;

		/* arm out */
		pwm_ng_set(&gen.servo3, SCANNER_POS_OUT);
		
	}
	else if (mode == I2C_SENSORBOARD_SCANNER_STOP){
		/* arm in */
		pwm_ng_set(&gen.servo3, SCANNER_POS_IN);
		scan_params.must_stop = 1;	
	}
	else if (mode == I2C_SENSORBOARD_SCANNER_START){
		scan_params.max_column_detected = 0;
		scan_params.must_stop = 0;
	

		/* start scan in background */
		scanner_do_scan();		
	}

		
}

/*
void scanner_stop(void)
{
	sensorboard.scanner.on = 0;
	pwm_ng_set(SCANNER_PWM, 0);
}
*/


/*
int32_t encoders_spi_get_scanner_speed(void * dummy)
{
	return scanner_speed;
}
*/


//uint8_t sample_tab[MAX_SAMPLE];
//uint16_t sample_i = 0;


//#define offset_a (75.*M_PI/180.)
//float offset_a;
//float offset_b;

//int32_t pos_start_scan;


/* get motor angle in radian; return mirror angle in radian, cos a sin a */
void ang2_a_mirror(float b, float * c_a, float* s_a, float* a)
{
	float x2, y2;
	float A, DELTA, B, D;

	b+=scan_params.offset_b;
	x2 = X + l1*cos(b);
	y2 = Y + l1*sin(b);

	A = (l3*l3 + x2*x2 + y2*y2 - l2*l2)/(2*l3);

	DELTA = -(A*A - x2*x2 - y2*y2);
	B = sqrt(DELTA);

	D = x2*x2 + y2*y2;

	*c_a = (x2*A + y2*B)/D;
	*s_a = -(x2*B - y2*A)/D;

	*a = atan2(*s_a, *c_a);

	*a += scan_params.offset_a;
	// *s_a = sin(*a);
	// *c_a = cos(*a);

}

/* get telemeter dist , cos a, sin a, a and return H, L of scanned point */
void ang2_H_L(float l_telemetre, float c_a, float s_a, float a, float *H, float *L)
{
	float d;
	d = h_mirror*c_a/s_a;
	*H = (l_telemetre - l_mirror - d)*sin(2*a);
	*L = l_mirror + d + *H/tan(2*a);

	//*H+= 8*sin(a-scan_params.offset_a);
}



//int32_t last_col_n;
//int32_t last_row_n;
//uint8_t last_sample;

//uint8_t h_limit[] = {40, 53, 66, 78, 94, 111, 123};
//uint8_t h_limit[] = {37, 48, 61, 72, 94, 111, 123};

/* last high is dummy, to deal higher columns */
uint8_t h_limit[] = {68, 79, 93, 107, 121, 138, 155, 170, 250};
#define H_MARGIN 7


#if 0
void do_scan(void * dummy) 
{

	unsigned int i;
	int16_t a;
	int32_t row_n;
	int32_t col_n;


	int32_t tour_pos;
	int32_t pos, last_pos;
	int32_t pos_tmp ;
	int32_t mot_pos;
	float dist;
	uint8_t min_sample;

	float b, c_a, s_a, H, L, m_a;
	int32_t H_fin;


	if (scan_params.sample_i==0)
		return;

	mot_pos = encoders_spi_get_value_scanner_interpolation((void *)SCANNER_ENC);

	if (scan_params.sample_i==1){
		SCANNER_DEBUG("dump end enc %ld %d ", mot_pos, PIX_PER_SCAN);
		//scanner.flags &= (~CS_ON);
		


		/* stop scan at cur pos + 10 round */
		mot_pos = encoders_spi_get_value_scanner_interpolation((void *)SCANNER_ENC);
		mot_pos = SCANNER_STEP_TOUR * ((mot_pos/SCANNER_STEP_TOUR) + 1) ;

		SCANNER_DEBUG("set to %ld ", mot_pos);

		cs_set_consign(&sensorboard.scanner.cs, mot_pos);
		//pwm_ng_set(SCANNER_MOT_PWM, 0);

		
	}

	mot_pos-= scan_params.pos_start_scan;

	a = adc_get_value( ADC_REF_AVCC | MUX_ADC13 );


	//dist = (a-TELEMETRE_B)/TELEMETRE_A;
	dist = TELEMETRE_A * a +TELEMETRE_B;

	//SCANNER_DEBUG("enc val = %ld",	encoders_microb_get_value((void *)SCANNER_ENC));


	//sample_tab[MAX_SAMPLE-sample_i] = a>0x1ff?0x1FF:a;
	//sample_tab[MAX_SAMPLE-sample_i] |= PINF&2?0x200:0;


	row_n = (mot_pos)/(SCANNER_STEP_TOUR/2);
#if 0
	/* separe scan forward/backword */
	if (row_n%2){
		row_n/=2;
	}
	else{
		row_n/=2;
		row_n+=30;
	}
#endif 

	tour_pos = (mot_pos)%(SCANNER_STEP_TOUR);

	b = (2.*M_PI)*(float)tour_pos/(float)(SCANNER_STEP_TOUR);

	ang2_a_mirror(b, &c_a, &s_a, &m_a);
	ang2_H_L(dist, c_a, s_a, m_a, &H, &L);


	SCANNER_DEBUG("%ld %d", tour_pos, a);

	if (H >0){
		printf("zarb H\n");
		H = 0;
	}

	if (dist> SCAN_MAX_DIST){
		H = 0;
		L = 0;
	}

	H = H;//m_a*180/M_PI;
	L = L;//(L-100)*PIX_PER_SCAN;

	//SCANNER_DEBUG("polling : ADC0 = %i %f",a, dist);
	//SCANNER_DEBUG("%f %f  %2.2f %f", H, L, m_a*180./M_PI, dist);


	//SCANNER_DEBUG("%f %f", dist, m_a*180/M_PI);

	H=(H+SCAN_H_MAX)*SCAN_H_COEF;
	L-=SCAN_L_MIN;


	/* XXX may never append */
	if (L<0)
		L=0;


	/* first filter => pixel modulo level */
#define H_BASE  10
#define H_MUL  14
	H_fin = H;//+SCAN_H_MAX;
	//H_fin = ((H_fin-H_BASE)/H_MUL)*H_MUL + H_BASE;

	if (scan_params.filter){
		H_fin =  11; // default is level 11
		for (i=0;i<sizeof(h_limit)/sizeof(uint8_t);i++){
			if (H < h_limit[i]-H_MARGIN){
				H_fin = i;
				break;
			}
		}
	}
	
	//SCANNER_DEBUG("%f %f", dist, m_a*180/M_PI);
	//SCANNER_DEBUG("%f %f", m_a*180/M_PI, b*180/M_PI);

	//SCANNER_DEBUG("%d %f", a, b*180/M_PI);
	//SCANNER_DEBUG("%f %f %f", H, m_a*180/M_PI, offset_b);

	//SCANNER_DEBUG("%d %2.2f ", a, tour_pos);


	//SCANNER_DEBUG("%f %f %ld %d", H, L, tour_pos, a);


	/*
	if (row_n%2){
		//tour_pos = ((SCANNER_STEP_TOUR/2)-tour_pos);
		tour_pos = (tour_pos*PIX_PER_SCAN)/(SCANNER_STEP_TOUR);
	}
	else{
		tour_pos = ((SCANNER_STEP_TOUR-tour_pos)*PIX_PER_SCAN)/(SCANNER_STEP_TOUR);
	}
	*/
	col_n = (PIX_PER_SCAN*L)/(SCAN_L_MAX-SCAN_L_MIN);
	if (col_n>PIX_PER_SCAN) 
		printf("BUG!!! RECALC MAX L");

	//col_n = (PIX_PER_SCAN+col_n -5)%PIX_PER_SCAN;

	//pos = (row_n*SCANNER_STEP_TOUR + tour_pos)/STEP_PER_POS;
	//pos= row_n*PIX_PER_SCAN+tour_pos;
	//last_pos= last_row_n*PIX_PER_SCAN+last_tour_pos;


	
	pos= row_n*PIX_PER_SCAN+col_n;
	last_pos= scan_params.last_row_n*PIX_PER_SCAN+scan_params.last_col_n;
	
	//SCANNER_DEBUG("%ld %ld %ld %ld", row_n, col_n, pos, H_fin);

	//a-=0x100;
	a-=200;
	//a/=10;

	if (0<= pos  && pos <MAX_SAMPLE)// && row_n%2)
		//sample_tab[pos] =  a>0xff?0xFF:a;
		//sample_tab[(int)L] = H ;
		scan_params.sample_tab[pos] = H_fin;
	nop();
	if ((scan_params.last_row_n == row_n) && ABS(last_pos-pos)>1){
		/* we have a hole, pad it with minimal hight */
		if (H_fin>scan_params.last_sample)
			min_sample = scan_params.last_sample;
		else
			min_sample = H_fin;

		//printf("(%ld, %ld) (%ld %ld)", last_col_n, last_row_n, col_n, row_n);

		/* fill grow, avoid erasing curent pos */
		if (pos > last_pos){
			pos_tmp = last_pos;
			last_pos = pos;
			//printf("loop1 on (%ld, %ld) %ld", pos_tmp, last_pos, last_pos-pos_tmp);
		
		}
		else{
			pos_tmp = pos+1;
			//printf("loop1 on (%ld, %ld) %ld", pos_tmp, last_pos, last_pos-pos_tmp);
		}

		
		for (;pos_tmp< last_pos;pos_tmp++){
			if (0< pos_tmp && pos_tmp <MAX_SAMPLE)// && row_n%2)
				//sample_tab[pos_tmp] =  min_sample;
			nop();
			
		}
		

	}

	scan_params.last_row_n = row_n;
	scan_params.last_col_n = col_n;
	scan_params.last_sample = H_fin;

	
	//printf("pos : %ld", pos);
	//sample_tab[sample_i] =  a>0x1ff?0x1FF:a;

	//sample_ok_tab[MAX_SAMPLE-sample_i] = PORTF&2;

	/*
	if (((pos <MAX_SAMPLE)) && (tour_pos<=(SCANNER_STEP_TOUR/2)))
		sample_tab[pos] = 0xffff;
	*/
	scan_params.sample_i--;
}
#endif



void do_scan(void * dummy) 
{

	int i, j;
	int16_t a;
	int32_t row_n;
	int32_t col_n;


	int32_t tour_pos;
	int32_t pos, last_pos;
	int32_t pos_tmp ;
	int32_t mot_pos;
	uint8_t min_sample;
	double H, L;
	int32_t H_fin, L_fin;

	uint8_t flags;

	union{
		uint16_t u16;
		lookup_h_l h_l;
	}val;



	if (scan_params.sample_i==0)
		return;



	scan_params.max_column_detected = !!sensor_get(SCANNER_MAXCOLUMN_SENSOR);

	mot_pos = encoders_spi_get_value_scanner_interpolation((void *)SCANNER_ENC);
	if (scan_params.sample_i==1 || scan_params.max_column_detected || scan_params.must_stop  ){

		/* reset sample num in case of max column detected */
		IRQ_LOCK(flags);
		scan_params.sample_i = 1;
		IRQ_UNLOCK(flags);


		SCANNER_DEBUG("dump end enc %"PRIi32" %d ", mot_pos, PIX_PER_SCAN);
		//scanner.flags &= (~CS_ON);

		/* stop scan at cur pos + 10 round */
		mot_pos = encoders_spi_get_value_scanner_interpolation((void *)SCANNER_ENC);
		mot_pos = SCANNER_STEP_TOUR * ((mot_pos/SCANNER_STEP_TOUR) + 1) ;

		SCANNER_DEBUG("set to %"PRIi32" ", mot_pos);

		cs_set_consign(&sensorboard.scanner.cs, mot_pos);
		//pwm_ng_set(SCANNER_MOT_PWM, 0);


		/* end working */
		
		//scanner_end_process();
		if (!scan_params.must_stop  && !scan_params.max_column_detected)
			process_img_to_zone(scan_params.sample_tab, PIX_PER_SCAN, MAX_SAMPLE/PIX_PER_SCAN);

		
		scan_params.working = 0;
		
	}

	mot_pos-= scan_params.pos_start_scan;

	a = adc_get_value( ADC_REF_AVCC | MUX_ADC13 );


	tour_pos = (mot_pos)%(SCANNER_STEP_TOUR);


	if (scan_params.debug != 0)
		SCANNER_DEBUG("%ld %d ", tour_pos, a);

	/* lookup in precomputed array */


	j = (DIM_DIST * (telemetre_to_cm(a)-TELEMETRE_MIN_CM))/(TELEMETRE_MAX_CM - TELEMETRE_MIN_CM);
	i = (DIM_ANGLE * tour_pos)/STEP_PER_ROUND;
	

	if (j<0)
		j=0;
	if (j>=DIM_DIST)
		j = DIM_DIST-1;
	
	if (i>=DIM_ANGLE)
		i = DIM_ANGLE-1;

	
	val.u16 = pgm_read_word_near(&array_h_l[j][i]);


	//val.u16 = pgm_read_word_near(&array_h_l[(a-TELEMETRE_MIN)/DIST_STEP][mot_pos/ANGLE_STEP]);
	//val.u16 = pgm_read_word_near(&array_h_l[a][tp]);
	H = val.h_l.h;
	L = val.h_l.l;
	/*
	val.u16 = pgm_read_word_near(&array_h_l[(a-TELEMETRE_MIN)/DIST_STEP][mot_pos/ANGLE_STEP]);

	H = val.h_l.h;
	L = val.h_l.l;
	*/
	H_fin = H;
	L_fin = L;


	if (L<=0)
		L = 0;

	col_n = (PIX_PER_SCAN*L)/(SCAN_L_MAX-SCAN_L_MIN);
	if (col_n>=PIX_PER_SCAN) {
		//printf("BUG!!! RECALC MAX L");
		col_n = PIX_PER_SCAN-1;
	}

	//col_n = (PIX_PER_SCAN+col_n -5)%PIX_PER_SCAN;

	//pos = (row_n*SCANNER_STEP_TOUR + tour_pos)/STEP_PER_POS;
	//pos= row_n*PIX_PER_SCAN+tour_pos;
	//last_pos= last_row_n*PIX_PER_SCAN+last_tour_pos;

	row_n = (mot_pos)/(SCANNER_STEP_TOUR/2);

	
	pos= row_n*PIX_PER_SCAN+col_n;
	last_pos= scan_params.last_row_n*PIX_PER_SCAN+scan_params.last_col_n;
	
	//SCANNER_DEBUG("%ld %ld %ld %ld", row_n, col_n, pos, H_fin);

	//a-=0x100;
	a-=200;
	//a/=10;

	if (0<= pos  && pos <MAX_SAMPLE)// && row_n%2)
		//sample_tab[pos] =  a>0xff?0xFF:a;
		//sample_tab[(int)L] = H ;
		scan_params.sample_tab[pos] = H_fin;
	nop();
	if ((scan_params.last_row_n == row_n) && ABS(last_pos-pos)>1){
		/* we have a hole, pad it with minimal hight */
		if (H_fin>scan_params.last_sample)
			min_sample = scan_params.last_sample;
		else
			min_sample = H_fin;

		//printf("(%ld, %ld) (%ld %ld)", last_col_n, last_row_n, col_n, row_n);

		/* fill grow, avoid erasing curent pos */
		if (pos > last_pos){
			pos_tmp = last_pos;
			last_pos = pos;
			//printf("loop1 on (%ld, %ld) %ld", pos_tmp, last_pos, last_pos-pos_tmp);
		
		}
		else{
			pos_tmp = pos+1;
			//printf("loop1 on (%ld, %ld) %ld", pos_tmp, last_pos, last_pos-pos_tmp);
		}

		
		for (;pos_tmp< last_pos;pos_tmp++){
			if (0< pos_tmp && pos_tmp <MAX_SAMPLE){
				//scan_params.sample_tab[pos_tmp] =  min_sample;
				nop();
			}
			
		}
		

	}

	scan_params.last_row_n = row_n;
	scan_params.last_col_n = col_n;
	scan_params.last_sample = H_fin;

	
	//printf("pos : %ld", pos);
	//sample_tab[sample_i] =  a>0x1ff?0x1FF:a;

	//sample_ok_tab[MAX_SAMPLE-sample_i] = PORTF&2;

	/*
	if (((pos <MAX_SAMPLE)) && (tour_pos<=(SCANNER_STEP_TOUR/2)))
		sample_tab[pos] = 0xffff;
	*/
	IRQ_LOCK(flags);
	scan_params.sample_i--;
	IRQ_UNLOCK(flags);

}
