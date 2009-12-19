
#define SCANNER_ENC         ((void *)1)

/* relative motor position from mirror axis */
#define X  4.5
#define Y  -1.25

/* length of motor arm*/
#define l1  0.9

/* length of bielle */
#define l2  2.113

/* length of mirror (rotating axe to bielle) */
//#define l3  4.714
#define l3  4.1613

/* distance from telemetre to mirror axis */
#define l_mirror  24.0

/* higth between mirror axis and horizontal laser beam*/
#define h_mirror  -1.0


/* transform telemeter unit to cm
   using linear approximation
   d_telemetre = a * cm + b
*/
//#define TELEMETRE_A (16.76)
//#define TELEMETRE_B (-476.)


#define TELEMETRE_A_INIT 0.067325
#define TELEMETRE_B_INIT 23.225


extern double TELEMETRE_A;
extern double TELEMETRE_B;




#define SCAN_L_MIN 15.
#define SCAN_L_MAX 40.

#define SCAN_H_MAX 40.
#define SCAN_H_COEF (255./(SCAN_H_MAX+10.))

#define SCAN_MAX_DIST 70.

#define MAX_SAMPLE (1500L)


// TRUE encoder value: 3531.75
#define SCANNER_STEP_TOUR (14127L)

//#define PIX_PER_SCAN 30L
#define PIX_PER_SCAN (25L)

void scanner_init(void);
void scanner_set_mode(uint8_t mode);

void scanner_end_process(void);

void scanner_calibre_laser(void);
void scanner_calibre_mirror(void);


void scanner_scan_autonomous(void);

int32_t encoders_spi_update_scanner(void * number);
int32_t encoders_spi_get_value_scanner(void *number);

void do_scan(void * dummy);



struct  scan_params{
	uint16_t sample_i;
	float offset_a;
	float offset_b;
	int speed;
	uint8_t filter;
	int32_t pos_start_scan;

	int32_t last_col_n;
	int32_t last_row_n;
	uint8_t last_sample;

	uint8_t debug;
	uint8_t sample_tab[MAX_SAMPLE+10];
	volatile uint8_t working;
	uint8_t must_stop;


	uint8_t algo;	


	union {
		struct {
			uint8_t working_zone;
			int16_t center_x;
			int16_t center_y;
		} drop_zone;
		
		struct {
			uint8_t level;
			int16_t temple_x;
			int16_t temple_y;
		} check_temple;
	};



	/* 
	   for column drop zone answer
	*/
	int8_t dropzone_h;
	int16_t dropzone_x;
	int16_t dropzone_y;

	uint8_t max_column_detected;
};



#define STEP_PER_ROUND 14127

#define TELEMETRE_MIN 66
#define TELEMETRE_MAX 466


#define TELEMETRE_MIN_CM (25)
#define TELEMETRE_MAX_CM (55)


#define DIM_ANGLE (400/4)
#define DIM_DIST ((TELEMETRE_MAX_CM - TELEMETRE_MIN_CM)*3)



#define ANGLE_STEP (STEP_PER_ROUND/DIM_ANGLE +1)
#define DIST_STEP ((TELEMETRE_MAX - TELEMETRE_MIN) /DIM_DIST +1)


typedef struct _lookup_h_l{
	uint16_t h:4;
	uint16_t l:12;
}lookup_h_l;

#define telemetre_to_cm(t) 	((double)(TELEMETRE_A * ((double)(t)) + TELEMETRE_B))

#define SCAN_DEFAULT_SPEED 600


extern struct scan_params scan_params;


