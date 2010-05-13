
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <aversive.h>
#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <uart.h>
#include <i2c.h>
#include <ax12.h>
#include <parse.h>
#include <rdline.h>
#include <pwm_ng.h>
#include <encoders_spi.h>
#include <timer.h>
#include <scheduler.h>
#include <pid.h>
#include <clock_time.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <adc.h>
#include <spi.h>

#include <blocking_detection_manager.h>

#include "sensor.h"

#include "../common/i2c_commands.h"
#include "main.h"
#include "beacon.h"

struct beacon beacon;

#define BEACON_PWM_VALUE 1000
#define IR_SENSOR() (!!(PINK&(1<<5)))
#define MODULO_TIMER (1023L)
#define COEFF_TIMER (2)
#define COEFF_MULT (100L)
#define COEFF_MULT2 (1000L)
#define BEACON_SIZE (9)
#define BEACON_MAX_SAMPLE (3)

#define OPPONENT_POS_X (11)
#define OPPONENT_POS_Y (22)

#define BEACON_DEBUG(args...) DEBUG(E_USER_BEACON, args)
#define BEACON_NOTICE(args...) NOTICE(E_USER_BEACON, args)
#define BEACON_ERROR(args...) ERROR(E_USER_BEACON, args)

static volatile int32_t rising = -1;
static volatile int32_t falling = -1;

static int32_t get_dist(float size);
static int32_t get_angle(int32_t middle, int32_t ref);

static int32_t pos_ref = 0;
static int32_t invalid_count = 0;

static volatile int32_t beacon_speed;
static volatile int32_t beacon_save_count = 0;
static volatile int32_t beacon_prev_save_count = 0;
static volatile int32_t count = 0;
static volatile int32_t count_diff_rising  = 0;
static volatile int32_t count_diff_falling = 0;
static int32_t beacon_coeff = 0;

static volatile int8_t valid_beacon = 0;

static volatile int32_t beacon_pos;

//static int32_t beacon_sample_size[BEACON_MAX_SAMPLE];

int32_t encoders_spi_get_value_beacon(void *number)
{
	int32_t ret;

	ret = encoders_spi_get_value(number);
	return ret*4;
}

void encoders_spi_set_value_beacon(void * number, int32_t v)
{
	encoders_spi_set_value(number, v/4);
}

int32_t encoders_spi_update_beacon_speed(void * number)
{
	int32_t ret;
	uint8_t flags;

	IRQ_LOCK(flags);
	ret = encoders_spi_get_value_beacon(number);
	beacon_speed = ret - beacon_pos;
	beacon_pos = ret;
	beacon_prev_save_count = beacon_save_count;
	beacon_save_count = TCNT3;
	IRQ_UNLOCK(flags);

	beacon_coeff = COEFF_TIMER  * COEFF_MULT;//beacon_speed * COEFF_MULT / ((beacon_prev_save_count - beacon_save_count + MODULO_TIMER + 1)&MODULO_TIMER);

	return beacon_speed;
}


void beacon_init(void)
{
	//int8_t i;

	beacon_reset_pos();
	pos_ref = encoders_spi_get_value_beacon(BEACON_ENCODER);

	memset(&beacon, 0, sizeof(struct beacon));
	beacon.opponent_x = I2C_OPPONENT_NOT_THERE;

	beacon_speed = 0;

	/*for(i=0;i<BEACON_MAX_SAMPLE;i++)
		beacon_sample_size[i] = 0;*/

	/* set external interrupt (any edge) */
	PCMSK2 = (1<<PCINT21);
	PCICR  = (1<<PCIE2);


}

void beacon_calibre_pos(void)
{
	ballboard.flags &= ~DO_CS;

	/* init beacon pos */
	pwm_ng_set(BEACON_PWM, 100);

	/* find rising edge of the mirror*/
	wait_ms(100);
	while (sensor_get(BEACON_POS_SENSOR));
	wait_ms(100);
	while (!sensor_get(BEACON_POS_SENSOR));

	pwm_ng_set(BEACON_PWM, 0);


	beacon_reset_pos();
	pid_reset(&ballboard.beacon.pid);
	encoders_spi_set_value_beacon(BEACON_ENCODER, BEACON_OFFSET_CALIBRE);

	cs_set_consign(&ballboard.beacon.cs, 0);

	ballboard.flags |= DO_CS;
}

void beacon_start(void)
{
	beacon_reset_pos();
	ballboard.beacon.on = 1;
	cs_set_consign(&ballboard.beacon.cs, 600);
}

void beacon_stop(void)
{
	ballboard.beacon.on = 0;
	pwm_ng_set(BEACON_PWM, 0);
}

void beacon_reset_pos(void)
{
	pwm_ng_set(BEACON_PWM, 0);
	encoders_spi_set_value(BEACON_ENCODER, 0);
}



int32_t encoders_spi_get_beacon_speed(void * dummy)
{
	return beacon_speed;
}


//port K bit 5
/* motor speed (top tour) */
SIGNAL(SIG_PIN_CHANGE2)
{
	uint8_t flags;

	/* rising edge */
	if ( IR_SENSOR()) {
		IRQ_LOCK(flags);
		count = TCNT3;
		rising = beacon_pos;
		count_diff_rising = (count - beacon_save_count + MODULO_TIMER + 1)&MODULO_TIMER;
		valid_beacon = 0;
		IRQ_UNLOCK(flags);

	}
	/* falling edge */
	else {
		IRQ_LOCK(flags);
		count = TCNT3;
		falling = beacon_pos;
		count_diff_falling = (count - beacon_save_count + MODULO_TIMER + 1)&MODULO_TIMER;
		valid_beacon = 1;
		IRQ_UNLOCK(flags);
	}
}

void beacon_calc(void *dummy)
{
	static uint8_t a=0;
	static int32_t local_rising, local_falling;
	static int32_t middle;
	static float size = 0;
	int32_t local_angle;
	int32_t local_dist;

	int32_t local_count_diff_rising ;
	int32_t local_count_diff_falling ;
	int32_t local_beacon_coeff;

	int32_t result_x=0;
	int32_t result_y=0;
	int32_t temp=0;
	int32_t edge=0;
	//int32_t total_size=0;

	uint8_t flags;
	//uint8_t i;
	int8_t local_valid;

	if(a)
		LED4_ON();
	else
		LED4_OFF();

	a = !a;

	if (falling == -1){
		/* 0.5 second timeout */
		if (invalid_count < 25)
			invalid_count++;
		else {
			IRQ_LOCK(flags);
			beacon.opponent_x = I2C_OPPONENT_NOT_THERE;
			IRQ_UNLOCK(flags);
		}
		return;
	}

	invalid_count = 0;
	IRQ_LOCK(flags);
	local_valid = valid_beacon;
	local_count_diff_rising  = count_diff_rising;
	local_count_diff_falling = count_diff_falling ;
	local_rising = rising;
	local_falling = falling;
	local_beacon_coeff = beacon_coeff;
	IRQ_UNLOCK(flags);

	if (local_valid){
		invalid_count = 0;
		//BEACON_DEBUG("rising= %ld\t",local_rising);
		//BEACON_DEBUG("falling= %ld\r\n",local_falling);

		/* recalculate number of pulse by adding the value of the counter, then put value back into motor's round range */
		local_rising  = ((local_rising + (local_count_diff_rising * local_beacon_coeff) / COEFF_MULT)) %(BEACON_STEP_TOUR);
		local_falling = ((local_falling + (local_count_diff_falling * local_beacon_coeff) / COEFF_MULT)) %(BEACON_STEP_TOUR);

		//BEACON_DEBUG("rising1= %ld\t",local_rising);
		//BEACON_DEBUG("falling1= %ld\r\n",local_falling);

		//BEACON_DEBUG("count diff rising= %ld\t",local_count_diff_rising);
		//BEACON_DEBUG("count diff falling= %ld\r\n",local_count_diff_falling);

		/* if around 360 deg, rising > falling, so invert both and recalculate size and middle */
		if(local_falling < local_rising){
			temp          = local_rising;
			local_rising  = local_falling;
			local_falling = temp;
			size          = BEACON_STEP_TOUR - local_falling + local_rising;
			middle        = (local_falling + ((int32_t)(size)/2) + BEACON_STEP_TOUR) %(BEACON_STEP_TOUR);
			edge = local_falling;
		}
		/* else rising > falling */
		else{
			size   = local_falling - local_rising;
			middle = local_rising + (size / 2);
			edge   = local_rising;
		}

		//for(i=BEACON_MAX_SAMPLE-1;i>0;i--){
		//	beacon_sample_size[i] = beacon_sample_size[i-1];
		//	total_size += beacon_sample_size[i];
		//}
		//beacon_sample_size[0] = size;
		//total_size += size;
		//total_size /= BEACON_MAX_SAMPLE;

		//BEACON_DEBUG("rising2= %ld\t",local_rising);
		//BEACON_DEBUG("falling2= %ld\r\n",local_falling);
		/* 			BEACON_DEBUG("size= %ld %ld\t",size, total_size); */
		BEACON_DEBUG("size= %f\r\n",size);
		//BEACON_DEBUG("middle= %ld\r\n",middle);

		local_angle = get_angle(middle,0);
		BEACON_NOTICE("opponent angle= %ld\t",local_angle);

		local_dist = get_dist(size);
		BEACON_NOTICE("opponent dist= %ld\r\n",local_dist);

		beacon_angle_dist_to_x_y(local_angle, local_dist, &result_x, &result_y);

		IRQ_LOCK(flags);
		beacon.opponent_x = result_x;
		beacon.opponent_y = result_y;
		beacon.opponent_angle = local_angle;
		beacon.opponent_dist = local_dist;
		/* for I2C test */
		//beacon.opponent_x = OPPONENT_POS_X;
		//beacon.opponent_y = OPPONENT_POS_Y;
		IRQ_UNLOCK(flags);

		BEACON_NOTICE("opponent x= %ld\t",beacon.opponent_x);
		BEACON_NOTICE("opponent y= %ld\r\n\n",beacon.opponent_y);
	}
	else {
		BEACON_NOTICE("non valid\r\n\n");
	}

	falling = -1;
}

static int32_t get_dist(float size)
{
	int32_t dist=0;
	//int32_t alpha=0;

	//alpha = (size*2*M_PI*COEFF_MULT2);
	//dist = ((2*BEACON_SIZE*BEACON_STEP_TOUR*COEFF_MULT2)/alpha)/2;

	/* function found by measuring points up to 80cm */
	//dist = ((size - 600)*(size - 600)) / 2400 +28;

	/* new function */
	/* dist = a0 + a1*x + a2*x² + a3x³ */
	dist = 1157.3 + (1.4146*size) + (-0.013508*size*size) + (0.00001488*size*size*size);

	return dist;

}

static int32_t get_angle(int32_t middle, int32_t ref)
{
	int32_t ret_angle;

	ret_angle = (middle - ref) * 360 / BEACON_STEP_TOUR;

	if(ret_angle > 360)
		ret_angle -= 360;

	return ret_angle;
}

void beacon_angle_dist_to_x_y(int32_t angle, int32_t dist, int32_t *x, int32_t *y)
{
	uint8_t flags;

	int32_t local_x;
	int32_t local_y;
	int32_t x_opponent;
	int32_t y_opponent;
	int32_t local_robot_angle;

	IRQ_LOCK(flags);
	local_x           = beacon.robot_x;
	local_y           = beacon.robot_y;
	local_robot_angle = beacon.robot_angle;
	IRQ_UNLOCK(flags);

	if (local_robot_angle < 0)
		local_robot_angle += 360;

	x_opponent = cos((local_robot_angle + angle)* 2 * M_PI / 360)* dist;
	y_opponent = sin((local_robot_angle + angle)* 2 * M_PI / 360)* dist;

	//BEACON_DEBUG("x_op= %ld\t",x_opponent);
	//BEACON_DEBUG("y_op= %ld\r\n",y_opponent);
	//BEACON_NOTICE("robot_x= %ld\t",local_x);
	//BEACON_NOTICE("robot_y= %ld\t",local_y);
	//BEACON_NOTICE("robot_angle= %ld\r\n",local_robot_angle);

	*x = local_x + x_opponent;
	*y = local_y + y_opponent;

}
