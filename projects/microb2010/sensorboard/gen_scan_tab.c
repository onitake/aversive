#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <inttypes.h>

#include <stdint.h>


#include "scanner.h"

#define ADC_REF_AVCC 0
#define  MUX_ADC13 0



double  TELEMETRE_A =  TELEMETRE_A_INIT;
double  TELEMETRE_B = TELEMETRE_B_INIT;


#define printf_P printf
#define PSTR(a) (a)
int32_t motor_angle = 0;
int32_t scan_dist = 0;


int32_t H_fin, L_fin;
double L, H;

#define nop()

#define ABS(val) ({					\
			__typeof(val) __val = (val);	\
			if (__val < 0)			\
				__val = - __val;	\
			__val;				\
		})

struct scan_params scan_params;

int32_t encoders_spi_get_value_scanner_interpolation(void *a)
{
	return motor_angle;
}


int16_t adc_get_value(uint8_t a)
{
	return scan_dist;
}



/* get motor angle in radian; return mirror angle in radian, cos a sin a */
void ang2_a_mirror(double b, double * c_a, double* s_a, double* a)
{
	double x2, y2;
	double A, DELTA, B, D;

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
void ang2_H_L(double l_telemetre, double c_a, double s_a, double a, double *H, double *L)
{
	double d;
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
//uint8_t h_limit[] = {68, 79, 93, 107, 121, 138, 155, 170, 250};


//uint8_t h_limit[] = {60, 72, 85, 98, 112, 129, 250};

//uint8_t h_limit[] = {80, 97, 118, 134, 145, 160, 250};


//uint8_t h_limit[] = {79, 94, 108, 117, 129, 144, 250};

uint8_t h_limit[] = {83, 95, 108, 120, 135, 147, 164, 250};
#define H_MARGIN (-6)

#define cs_set_consign(a, b) 



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
	double dist;
	uint8_t min_sample;

	double b, c_a, s_a, /*H, L,*/ m_a;
	//	int32_t H_fin;


	//printf("scan\n");
	if (scan_params.sample_i==0)
		return;

	mot_pos = encoders_spi_get_value_scanner_interpolation((void *)SCANNER_ENC);
#if 0
	if (scan_params.sample_i==1){
		printf_P(PSTR("dump end enc %ld %d \r\n"), mot_pos, PIX_PER_SCAN);
		//scanner.flags &= (~CS_ON);
		


		/* stop scan at cur pos + 10 round */
		mot_pos = encoders_spi_get_value_scanner_interpolation((void *)SCANNER_ENC);
		mot_pos = SCANNER_STEP_TOUR * ((mot_pos/SCANNER_STEP_TOUR) + 1) ;

		printf_P(PSTR("set to %ld \r\n"), mot_pos);

		cs_set_consign(&sensorboard.scanner.cs, mot_pos);
		//pwm_ng_set(SCANNER_MOT_PWM, 0);

		
	}

	mot_pos-= scan_params.pos_start_scan;
#endif
	a = adc_get_value( ADC_REF_AVCC | MUX_ADC13 );
	

	//dist = (a-TELEMETRE_B)/TELEMETRE_A;
	//dist = TELEMETRE_A * a +TELEMETRE_B;
	dist = telemetre_to_cm(a);

	//printf_P(PSTR("enc val = %ld\r\n"),	encoders_microb_get_value((void *)SCANNER_ENC));


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

	b = (2.*M_PI)*(double)tour_pos/(double)(SCANNER_STEP_TOUR);

	ang2_a_mirror(b, &c_a, &s_a, &m_a);
	ang2_H_L(dist, c_a, s_a, m_a, &H, &L);


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

	//printf_P(PSTR("polling : ADC0 = %i %f\r\n"),a, dist);
	//printf_P(PSTR("%f %f  %2.2f %f\r\n"), H, L, m_a*180./M_PI, dist);


	//printf_P(PSTR("dist, b, a: %2.2f %2.2f %2.2f\r\n"), dist, b*180/M_PI, m_a*180/M_PI);

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
			if (H < h_limit[i]+H_MARGIN){
				H_fin = i;
				break;
			}
		}
	}

	L_fin = L;
	//printf_P(PSTR("%f %f\r\n"), dist, m_a*180/M_PI);
	//printf_P(PSTR("%f %f\r\n"), m_a*180/M_PI, b*180/M_PI);

	//printf_P(PSTR("%d %f\r\n"), a, b*180/M_PI);
	//printf_P(PSTR("%f %f %f\r\n"), H, m_a*180/M_PI, offset_b);

	//printf_P(PSTR("a, tpos: %d %d \r\n"), a, tour_pos);


	//printf_P(PSTR("%f %f\r\n"), H, L);


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
		printf("BUG!!! RECALC MAX L\r\n");

	//col_n = (PIX_PER_SCAN+col_n -5)%PIX_PER_SCAN;

	//pos = (row_n*SCANNER_STEP_TOUR + tour_pos)/STEP_PER_POS;
	//pos= row_n*PIX_PER_SCAN+tour_pos;
	//last_pos= last_row_n*PIX_PER_SCAN+last_tour_pos;


	
	pos= row_n*PIX_PER_SCAN+col_n;
	last_pos= scan_params.last_row_n*PIX_PER_SCAN+scan_params.last_col_n;
	
	//printf_P(PSTR("%ld %ld %ld %ld\r\n"), row_n, col_n, pos, H_fin);

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

		//printf("(%ld, %ld) (%ld %ld)\r\n", last_col_n, last_row_n, col_n, row_n);

		/* fill grow, avoid erasing curent pos */
		if (pos > last_pos){
			pos_tmp = last_pos;
			last_pos = pos;
			//printf("loop1 on (%ld, %ld) %ld\r\n", pos_tmp, last_pos, last_pos-pos_tmp);
		
		}
		else{
			pos_tmp = pos+1;
			//printf("loop1 on (%ld, %ld) %ld\r\n", pos_tmp, last_pos, last_pos-pos_tmp);
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

	
	//printf("pos : %ld\r\n", pos);
	//sample_tab[sample_i] =  a>0x1ff?0x1FF:a;

	//sample_ok_tab[MAX_SAMPLE-sample_i] = PORTF&2;

	/*
	if (((pos <MAX_SAMPLE)) && (tour_pos<=(SCANNER_STEP_TOUR/2)))
		sample_tab[pos] = 0xffff;
	*/
	scan_params.sample_i--;
}


#define my_offset_a (0*M_PI/180)
#define my_offset_b (34*M_PI/180)

//#define my_offset_a (40*M_PI/180)
//#define my_offset_b (-5*M_PI/180)



/* 
 *2 cause we don't known exactly limits 
 at computation time 
*/
lookup_h_l array_h_l[DIM_DIST*2][DIM_ANGLE*2];



#define pgm_read_word_near(a) (*((uint16_t*)(a)))


void do_scan_quick(void * dummy) 
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
	//double H, L;
	//int32_t H_fin, L_fin;


	union{
		uint16_t u16;
		lookup_h_l h_l;
	}val;
	if (scan_params.sample_i==0)
		return;

	mot_pos = encoders_spi_get_value_scanner_interpolation((void *)SCANNER_ENC);
	if (scan_params.sample_i==1){
		printf_P(PSTR("dump end enc %ld %d \r\n"), mot_pos, PIX_PER_SCAN);
		//scanner.flags &= (~CS_ON);
		


		/* stop scan at cur pos + 10 round */
		mot_pos = encoders_spi_get_value_scanner_interpolation((void *)SCANNER_ENC);
		mot_pos = SCANNER_STEP_TOUR * ((mot_pos/SCANNER_STEP_TOUR) + 1) ;

		printf_P(PSTR("set to %ld \r\n"), mot_pos);

		cs_set_consign(&sensorboard.scanner.cs, mot_pos);
		//pwm_ng_set(SCANNER_MOT_PWM, 0);

		
	}

	mot_pos-= scan_params.pos_start_scan;

	a = adc_get_value( ADC_REF_AVCC | MUX_ADC13 );


	tour_pos = (mot_pos)%(SCANNER_STEP_TOUR);


	if (scan_params.offset_a != 0)
		printf_P(PSTR("%ld %d \r\n"), tour_pos, a);

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



	col_n = (PIX_PER_SCAN*L)/(SCAN_L_MAX-SCAN_L_MIN);
	if (col_n>PIX_PER_SCAN) 
		printf("BUG!!! RECALC MAX L\r\n");

	//col_n = (PIX_PER_SCAN+col_n -5)%PIX_PER_SCAN;

	//pos = (row_n*SCANNER_STEP_TOUR + tour_pos)/STEP_PER_POS;
	//pos= row_n*PIX_PER_SCAN+tour_pos;
	//last_pos= last_row_n*PIX_PER_SCAN+last_tour_pos;

	row_n = (mot_pos)/(SCANNER_STEP_TOUR/2);

	
	pos= row_n*PIX_PER_SCAN+col_n;
	last_pos= scan_params.last_row_n*PIX_PER_SCAN+scan_params.last_col_n;
	
	//printf_P(PSTR("%ld %ld %ld %ld\r\n"), row_n, col_n, pos, H_fin);

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

		//printf("(%ld, %ld) (%ld %ld)\r\n", last_col_n, last_row_n, col_n, row_n);

		/* fill grow, avoid erasing curent pos */
		if (pos > last_pos){
			pos_tmp = last_pos;
			last_pos = pos;
			//printf("loop1 on (%ld, %ld) %ld\r\n", pos_tmp, last_pos, last_pos-pos_tmp);
		
		}
		else{
			pos_tmp = pos+1;
			//printf("loop1 on (%ld, %ld) %ld\r\n", pos_tmp, last_pos, last_pos-pos_tmp);
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

	
	//printf("pos : %ld\r\n", pos);
	//sample_tab[sample_i] =  a>0x1ff?0x1FF:a;

	//sample_ok_tab[MAX_SAMPLE-sample_i] = PORTF&2;

	/*
	if (((pos <MAX_SAMPLE)) && (tour_pos<=(SCANNER_STEP_TOUR/2)))
		sample_tab[pos] = 0xffff;
	*/
	scan_params.sample_i--;
}


int main(int argc, char** argv)
{
	union{
		uint16_t u16;
		lookup_h_l h_l;
	}val;

	int i, j;
	FILE* f, *fin, *f_header;

	unsigned char line[1000];
	unsigned int a, b;
	unsigned int dist_max, angle_max;

	scan_params.filter = 1;	
	

	
	f = fopen("oo", "w");


	TELEMETRE_B = 23.50;

	scan_params.offset_a = my_offset_a;
	scan_params.offset_b = my_offset_b;



	

	/*precompute H & L array */
	for (j = 0, scan_dist = TELEMETRE_MIN; scan_dist< TELEMETRE_MAX; j++, scan_dist+=DIST_STEP){
		
		for (i = 0, motor_angle = 0; motor_angle < STEP_PER_ROUND; i++, motor_angle+=ANGLE_STEP){
			scan_params.sample_i = 100;	
			scan_params.pos_start_scan = 0;
			//printf("%d\n", i);
			do_scan(0); 
			//printf("mangle, dist, h, l: %d %d, (%d, %2.2f)\n", motor_angle, scan_dist, H_fin, L);
			//fprintf(f, "mangle, dist, h, l: %d %d, (%d, %2.2f)\n", motor_angle, scan_dist, H_fin, L);
			/*
			j = (scan_dist - TELEMETRE_MIN)/DIST_STEP;
			i = motor_angle/ANGLE_STEP;
			//printf("%d %d (%d %d) (%d %d %d = %d)\n", i, j, motor_angle, ANGLE_STEP, scan_dist, TELEMETRE_MIN, DIST_STEP, (scan_dist - TELEMETRE_MIN)/DIST_STEP);
			array_h_l[j][i].h = H_fin;
			array_h_l[j][i].l = L_fin;
			*/
			j = (DIM_DIST * (telemetre_to_cm(scan_dist)-TELEMETRE_MIN_CM))/(TELEMETRE_MAX_CM - TELEMETRE_MIN_CM);
			i = (DIM_ANGLE * motor_angle)/STEP_PER_ROUND;
			array_h_l[j][i].h = H_fin;
			array_h_l[j][i].l = L_fin;

		}
	}	


	scan_params.filter = 0;
	if (argc>1 && !strcmp(argv[1], "1"))
		scan_params.filter = 1;	


	printf("max i max j %d %d \n", i, j);
	dist_max = j;
	angle_max = i;

	f_header = fopen("scan_h_l.h", "w");
	fprintf(f_header, "PROGMEM lookup_h_l array_h_l[%d][%d] = {\n", dist_max, angle_max);
	for (j = 0, scan_dist = 0; j<dist_max; j++, scan_dist+=DIST_STEP){
		fprintf(f_header, "{\n");
 
		for (i = 0, motor_angle = 0; i < angle_max; i++, motor_angle+=ANGLE_STEP){
			fprintf(f_header, "{ .h = %d, .l = %d }, ", array_h_l[j][i].h, array_h_l[j][i].l);
			
		}
		fprintf(f_header, "},\n");
	}	
	fprintf(f_header, "};\n", DIM_DIST, DIM_ANGLE);
	
	fclose(f_header);
	


	printf("IIIIIIIIIIII %d %d\n", array_h_l[38][152].h, array_h_l[38][152].l); 
	val.u16 = pgm_read_word_near(&array_h_l[38][152]);
	printf("IIIIIIIIIIII %d %d\n", val.h_l.h, val.h_l.l); 

	motor_angle = 9000;
	scan_dist = 388;
	
	for (motor_angle = 0; motor_angle<14000; motor_angle+=100){
		for (scan_dist = TELEMETRE_MIN; scan_dist<300; scan_dist+=10){
		scan_params.sample_i = 100;	
			scan_params.pos_start_scan = 0;
		
		do_scan(0); 


		//printf("m s %d %d\n", motor_angle, scan_dist);
		//printf("R %d %d\n", H_fin, L_fin);
		
		val.u16 = pgm_read_word_near(&array_h_l[(scan_dist-TELEMETRE_MIN)/DIST_STEP][motor_angle/ANGLE_STEP]);
		//printf("Q %d %d\n\n", val.h_l.h, val.h_l.l); 

		if (val.h_l.h != H_fin || val.h_l.l != L_fin)
			printf("BUG BUG\n");
	}
	}

	fin = fopen("out", "r");
	while(fgets(line, sizeof(line), fin)){
		scan_params.sample_i = 100;	
			scan_params.pos_start_scan = 0;
		
		//printf("[%s]\n", line);
		sscanf(line, "%d %d\n", &a, &b);
		//printf("%d %d\n", a, b);
		motor_angle = a;
		scan_dist = b;
		do_scan_quick(0); 
		/*
		j = (DIM_DIST * (telemetre_to_cm(scan_dist)-TELEMETRE_MIN_CM))/(TELEMETRE_MAX_CM - TELEMETRE_MIN_CM);
		i = (DIM_ANGLE * motor_angle)/STEP_PER_ROUND;
		
		val.u16 = pgm_read_word_near(&array_h_l[j][i]);
		H_fin = val.h_l.h;
		L_fin = val.h_l.l;
		*/
		
		printf("Q %d %d\n", H_fin, L_fin);

		do_scan(0); 

		printf("R %d %d\n", H_fin, L_fin);

		fprintf(f, "mangle, dist, h, l: %d %d, (%d, %d)\n", motor_angle, scan_dist, H_fin, L_fin);

	}
	fclose(fin);

	fclose(f);

	return;
	/*
	for (j = 0, scan_dist = 0; j<DIM_DIST; j++, scan_dist+=DIST_STEP){
		for (i = 0, motor_angle = 0; i < DIM_ANGLE; i++, motor_angle+=ANGLE_STEP){
			do_scan(0); 
			printf("%d %d, (%d, %2.2f)\n", motor_angle, scan_dist, H_fin, L);
	
		}

	}
	*/


}
