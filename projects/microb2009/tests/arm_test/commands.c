/*
 *  Copyright Droids Corporation (2008)
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
 *  Revision : $Id: commands.c,v 1.6 2009-03-15 20:08:51 zer0 Exp $
 *
 *  Olivier MATZ <zer0@droids-corp.org> 
 */


#include <stdio.h>
#include <string.h>

#include <aversive/pgmspace.h>
#include <aversive/wait.h>

#include <i2c.h>
#include <ax12.h>
#include <parse.h>
#include <parse_num.h>
#include <parse_string.h>
#include <uart.h>
#include <encoders_microb.h>
#include <pwm_ng.h>
#include <pid.h>
#include <spi.h>
#include <time.h>
#include <quadramp.h>
#include <control_system_manager.h>

#include <scheduler.h>

#include "main.h"

#include "arm_xy.h"

extern AX12 ax12;

#define AX12_MAX_RETRY 2

uint8_t AX12_read_int_retry(AX12 *ax12, uint8_t id, AX12_ADDRESS address,
			 uint16_t *val)
{
	int8_t err, i;
	for (i=0;i<AX12_MAX_RETRY;i++){
		err = AX12_read_int(ax12, id, address, val);
		if (!err) 
			return err;
	}
	
	printf("AX12: retry %d times err: %d\r\n", AX12_MAX_RETRY, err);
	return err;
}

uint8_t AX12_write_int_retry(AX12 *ax12, uint8_t id, AX12_ADDRESS address,
		       uint16_t data)
{
	int8_t err, i;
	for (i=0;i<AX12_MAX_RETRY;i++){
		err = AX12_write_int(ax12, id, address,data);
		if (!err) 
			return err;
	}
	
	printf("AX12: retry %d times err: %d\r\n", AX12_MAX_RETRY, err);
	return err;
}

uint8_t addr_from_string(const char *s)
{
	/* 16 bits */
	if (!strcmp_P(s, PSTR("cw_angle_limit")))
		return AA_CW_ANGLE_LIMIT_L;
	if (!strcmp_P(s, PSTR("ccw_angle_limit")))
		return AA_CCW_ANGLE_LIMIT_L;
	if (!strcmp_P(s, PSTR("max_torque")))
		return AA_MAX_TORQUE_L;
	if (!strcmp_P(s, PSTR("down_calibration")))
		return AA_DOWN_CALIBRATION_L;
	if (!strcmp_P(s, PSTR("up_calibration")))
		return AA_UP_CALIBRATION_L;
	if (!strcmp_P(s, PSTR("torque_limit")))
		return AA_TORQUE_LIMIT_L;
	if (!strcmp_P(s, PSTR("position")))
		return AA_PRESENT_POSITION_L;
	if (!strcmp_P(s, PSTR("speed")))
		return AA_PRESENT_SPEED_L;
	if (!strcmp_P(s, PSTR("load")))
		return AA_PRESENT_LOAD_L;
	if (!strcmp_P(s, PSTR("moving_speed")))
		return AA_MOVING_SPEED_L;
	if (!strcmp_P(s, PSTR("model")))
		return AA_MODEL_NUMBER_L;
	if (!strcmp_P(s, PSTR("goal_pos")))
		return AA_GOAL_POSITION_L;
	if (!strcmp_P(s, PSTR("punch")))
		return AA_PUNCH_L;

	/* 8 bits */
	if (!strcmp_P(s, PSTR("firmware")))
		return AA_FIRMWARE;
	if (!strcmp_P(s, PSTR("id")))
		return AA_ID;
	if (!strcmp_P(s, PSTR("baudrate")))
		return AA_BAUD_RATE;
	if (!strcmp_P(s, PSTR("delay")))
		return AA_DELAY_TIME;
	if (!strcmp_P(s, PSTR("high_lim_temp")))
		return AA_HIGHEST_LIMIT_TEMP;
	if (!strcmp_P(s, PSTR("low_lim_volt")))
		return AA_LOWEST_LIMIT_VOLTAGE;
	if (!strcmp_P(s, PSTR("high_lim_volt")))
		return AA_HIGHEST_LIMIT_VOLTAGE;
	if (!strcmp_P(s, PSTR("status_return")))
		return AA_STATUS_RETURN_LEVEL;
	if (!strcmp_P(s, PSTR("alarm_led")))
		return AA_ALARM_LED;
	if (!strcmp_P(s, PSTR("alarm_shutdown")))
		return AA_ALARM_SHUTDOWN;
	if (!strcmp_P(s, PSTR("torque_enable")))
		return AA_TORQUE_ENABLE;
	if (!strcmp_P(s, PSTR("led")))
		return AA_LED;
	if (!strcmp_P(s, PSTR("cw_comp_margin")))
		return AA_CW_COMPLIANCE_MARGIN;
	if (!strcmp_P(s, PSTR("ccw_comp_margin")))
		return AA_CCW_COMPLIANCE_MARGIN;
	if (!strcmp_P(s, PSTR("cw_comp_slope")))
		return AA_CW_COMPLIANCE_SLOPE;
	if (!strcmp_P(s, PSTR("ccw_comp_slope")))
		return AA_CCW_COMPLIANCE_SLOPE;
	if (!strcmp_P(s, PSTR("voltage")))
		return AA_PRESENT_VOLTAGE;
	if (!strcmp_P(s, PSTR("temp")))
		return AA_PRESENT_TEMP;
	if (!strcmp_P(s, PSTR("reginst")))
		return AA_PRESENT_REGINST;
	if (!strcmp_P(s, PSTR("moving")))
		return AA_MOVING;
	if (!strcmp_P(s, PSTR("lock")))
		return AA_LOCK;
	
	return 0;
}



/**********************************************************/
/* Reset */

/* this structure is filled when cmd_reset is parsed successfully */
struct cmd_reset_result {
	fixed_string_t arg0;
};

/* function called when cmd_reset is parsed successfully */
static void cmd_reset_parsed(void * parsed_result, void * data)
{
	reset();
}

prog_char str_reset_arg0[] = "reset";
parse_pgm_token_string_t cmd_reset_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_reset_result, arg0, str_reset_arg0);

prog_char help_reset[] = "Reset the board";
parse_pgm_inst_t cmd_reset = {
	.f = cmd_reset_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_reset,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_reset_arg0, 
		NULL,
	},
};

/**********************************************************/
/* Spi_Test */

/* this structure is filled when cmd_spi_test is parsed successfully */
struct cmd_spi_test_result {
	fixed_string_t arg0;
};

/* function called when cmd_spi_test is parsed successfully */
static void cmd_spi_test_parsed(void * parsed_result, void * data)
{
#if 0
	uint8_t i, ret;

	for (i=0; i<3; i++) {
		spi_slave_select(0);
		ret = spi_send_and_receive_byte(i);
		spi_slave_deselect(0);
		printf_P(PSTR("Sent %d, received %d\r\n"), i, ret);
	}
#else
	printf_P(PSTR("disabled\r\n"));
#endif
}

prog_char str_spi_test_arg0[] = "spi_test";
parse_pgm_token_string_t cmd_spi_test_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_spi_test_result, arg0, str_spi_test_arg0);

prog_char help_spi_test[] = "Test the SPI";
parse_pgm_inst_t cmd_spi_test = {
	.f = cmd_spi_test_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_spi_test,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_spi_test_arg0, 
		NULL,
	},
};

/**********************************************************/
/* Bootloader */

/* this structure is filled when cmd_bootloader is parsed successfully */
struct cmd_bootloader_result {
	fixed_string_t arg0;
};

/* function called when cmd_bootloader is parsed successfully */
static void cmd_bootloader_parsed(void * parsed_result, void * data)
{
#ifdef __AVR_ATmega128__
#define BOOTLOADER_ADDR 0x1e000
#else
#define BOOTLOADER_ADDR 0x3f000
#endif
	if (pgm_read_byte_far(BOOTLOADER_ADDR) == 0xff) {
		printf_P(PSTR("Bootloader is not present\r\n"));
		return;
	}
	cli();
	/* ... very specific :( */
#ifdef __AVR_ATmega128__
	TIMSK = 0;
	ETIMSK = 0;
#else
	/* XXX */
#endif
	EIMSK = 0;
	UCSR0B = 0;
	UCSR1B = 0;
	SPCR = 0;
	TWCR = 0;
	ACSR = 0;
	ADCSRA = 0;

#ifdef __AVR_ATmega128__
	__asm__ __volatile__ ("ldi r30,0x00\n");
	__asm__ __volatile__ ("ldi r31,0xf0\n");
	__asm__ __volatile__ ("ijmp\n");
#else
	__asm__ __volatile__ ("ldi r30,0x00\n");
	__asm__ __volatile__ ("ldi r31,0xf8\n");
	__asm__ __volatile__ ("ldi eind,0x01\n");
	__asm__ __volatile__ ("eijmp\n");
#endif
}

prog_char str_bootloader_arg0[] = "bootloader";
parse_pgm_token_string_t cmd_bootloader_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_bootloader_result, arg0, str_bootloader_arg0);

prog_char help_bootloader[] = "Bootloader the board";
parse_pgm_inst_t cmd_bootloader = {
	.f = cmd_bootloader_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_bootloader,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_bootloader_arg0, 
		NULL,
	},
};

/**********************************************************/
/* Ax12_Stress */

/* this structure is filled when cmd_ax12_stress is parsed successfully */
struct cmd_ax12_stress_result {
	fixed_string_t arg0;
	uint8_t id;
	uint32_t num;

};

/* function called when cmd_ax12_stress is parsed successfully */
static void cmd_ax12_stress_parsed(void * parsed_result, void * data)
{
	uint32_t i, nb_errs = 0;
	uint8_t val;
	microseconds t = time_get_us2();
	struct cmd_ax12_stress_result *res = parsed_result;

	for (i=0; i<res->num; i++) {
		if (AX12_read_byte(&ax12, res->id, AA_ID, &val) != 0)
			nb_errs ++;
		if (uart_recv_nowait(0) != -1)
			break;

	}

	printf_P(PSTR("%ld errors / %ld\r\n"), nb_errs, i);
	t = (time_get_us2() - t) / 1000;
	printf_P(PSTR("Test done in %ld ms\r\n"), t);
}

prog_char str_ax12_stress_arg0[] = "ax12_stress";
parse_pgm_token_string_t cmd_ax12_stress_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_ax12_stress_result, arg0, str_ax12_stress_arg0);
parse_pgm_token_num_t cmd_ax12_stress_arg1 = TOKEN_NUM_INITIALIZER(struct cmd_ax12_stress_result, id, UINT8);
parse_pgm_token_num_t cmd_ax12_stress_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_ax12_stress_result, num, UINT32);

prog_char help_ax12_stress[] = "Ax12_Stress the board";
parse_pgm_inst_t cmd_ax12_stress = {
	.f = cmd_ax12_stress_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_ax12_stress,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_ax12_stress_arg0, 
		(prog_void *)&cmd_ax12_stress_arg1, 
		(prog_void *)&cmd_ax12_stress_arg2, 
		NULL,
	},
};


/**********************************************************/
/* Test */

/* this structure is filled when cmd_test is parsed successfully */
struct cmd_test_result {
	fixed_string_t arg0;
	uint16_t arg1;
	uint16_t arg2;
};

#define R_ELBOW_AX12 1
#define R_WRIST_AX12 2

#define L_ELBOW_AX12 4
#define L_WRIST_AX12 3


#define FINGER_AX12 5


void arm_l_goto(int32_t shoulder, uint16_t elbow, uint16_t wrist)
{
	uint8_t err;

	//printf("%ld %d %d\r\n", shoulder, elbow, wrist);
	//wait_ms(1);
	//cs_set_consign(&arm.cs_mot, shoulder);
	err = AX12_write_int(&ax12, L_ELBOW_AX12, AA_GOAL_POSITION_L, elbow);
	if (!err) 
 		err = AX12_write_int(&ax12, L_WRIST_AX12, AA_GOAL_POSITION_L, wrist); 
	if (err)
		printf_P(PSTR("AX12 error %x !\r\n"), err);
}


void arm_goto(Arm_Pos * arm_pos, int32_t shoulder, uint16_t elbow, uint16_t wrist)
{
	uint8_t err;

	//printf("%ld %d %d\r\n", shoulder, elbow, wrist);
	//wait_ms(1);
	cs_set_consign(&arm.cs_mot, shoulder);
	err = AX12_write_int(&ax12, arm_pos->ELBOW_AX12, AA_GOAL_POSITION_L, elbow);
	if (!err) 
 		err = AX12_write_int(&ax12, arm_pos->WRIST_AX12, AA_GOAL_POSITION_L, wrist); 
	if (err)
		printf_P(PSTR("AX12 error %x !\r\n"), err);
}

void arm_goto_dh(Arm_Pos* arm_pos, int32_t shoulder, uint16_t elbow)
{
	uint8_t err;
	cs_set_consign(&arm.cs_mot, shoulder);
	err = AX12_write_int(&ax12, arm_pos->ELBOW_AX12, AA_GOAL_POSITION_L, elbow);
	if (err)
		printf_P(PSTR("AX12 error %x !\r\n"), err);
}

void finger_right(void)
{
	AX12_write_int(&ax12, FINGER_AX12, AA_GOAL_POSITION_L, 666);
}

void finger_left(void)
{
	AX12_write_int(&ax12, FINGER_AX12, AA_GOAL_POSITION_L, 340);
}

void finger_center(void)
{
	AX12_write_int(&ax12, FINGER_AX12, AA_GOAL_POSITION_L, 490);
}

/*
#define arm_take_high_v1()  arm_goto(-18700, 204, 455)
#define arm_take_low_v1()   arm_goto(-11000, 273, 480)
#define arm_take_high_v2()  arm_goto(-18700, 204, 154)
#define arm_take_low_v2()   arm_goto(-11000, 273, 139)
#define arm_intermediate()  arm_goto(-35700, 297, 385)
#define arm_drop_v2()       arm_goto(-16810, 667, 564)
#define arm_drop_v1()       arm_goto(-16810, 667, 904)
*/

#define arm_take_high_v1()  arm_goto(-11533, 295, 477)
#define arm_take_low_v1()   arm_goto(-9835 , 311, 470)
#define arm_take_high_v2()  arm_goto(-11553, 295, 126)
#define arm_take_low_v2()   arm_goto(-9835,  311, 118)
#define arm_intermediate()  arm_goto(-25753, 299, 252)
#define arm_drop_v2()       arm_goto(-2427,  656, 396)
#define arm_drop_v1()       arm_goto(-11280, 547, 699)

#define arm_drop_v3()       arm_goto(-11280, 547, 396)
//#define arm_drop_v4()       arm_goto(-11000, 656, 699)



#define ARM_INTER_WAIT 
/* function called when cmd_test is parsed successfully */
static void cmd_test_parsed(void * parsed_result, void * data)
{
#if 0
	uint8_t i=0;
	struct cmd_test_result *res = parsed_result;
	uint16_t t_w = res->arg1;
	int16_t ppwm = res->arg2;

	AX12_write_int(&ax12, ELBOW_AX12, AA_MOVING_SPEED_L, 1000);
/* 	int8_t err; */

	/* Set some AX12 parameters */
/* 	err = AX12_write_int(&ax12,0xFE,AA_TORQUE_ENABLE,0x1); */
/* 	if (!err) */
/* 		AX12_write_int(&ax12,0xFE,AA_PUNCH_L,0x20); */
/* 	if (!err) */
/* 		AX12_write_int(&ax12,0xFE,AA_TORQUE_LIMIT_L,0x3FF); */
/* 	if (!err) */
/* 		AX12_write_int(&ax12,0xFE,AA_MOVING_SPEED_L,0x3FF); */
/* 	if (!err) */
/* 		AX12_write_byte(&ax12,0xFE,AA_ALARM_LED,0xEF); */
/* 	if (err) { */
/* 		printf_P(PSTR("AX12 error %x !\r\n"), err); */
/* 		return; */
/* 	} */

	for (i=0; i<1; i++) {
		arm_take_high_v1();
		wait_ms(t_w);
		pwm_ng_set(&arm.pwm1B, ppwm);
		arm_take_low_v1();
		wait_ms(t_w);
		arm_take_high_v1();
		wait_ms(t_w);


		arm_take_high_v2();
		wait_ms(t_w);
		pwm_ng_set(&arm.pwm3C, ppwm);
		arm_take_low_v2();
		wait_ms(t_w);
		arm_take_high_v2();
		wait_ms(t_w);


		arm_intermediate();
		wait_ms(t_w);
/* 		arm_intermediate2(); */
/* 		wait_ms(250); */
		arm_drop_v2();
		wait_ms(t_w);
		pwm_ng_set(&arm.pwm3C, -ppwm);
		wait_ms(50);
		pwm_ng_set(&arm.pwm3C, 0);

		arm_drop_v3();
		wait_ms(t_w);
		//arm_drop_v4();
		//wait_ms(t_w);
		arm_drop_v1();
		wait_ms(t_w);
		pwm_ng_set(&arm.pwm1B, -ppwm);
		wait_ms(50);
		pwm_ng_set(&arm.pwm1B, 0);

		arm_intermediate();
		wait_ms(t_w);
	}
#endif
}

prog_char str_test_arg0[] = "test";
parse_pgm_token_string_t cmd_test_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_test_result, arg0, str_test_arg0);
parse_pgm_token_num_t cmd_test_arg1 = TOKEN_NUM_INITIALIZER(struct cmd_test_result, arg1, UINT16);
parse_pgm_token_num_t cmd_test_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_test_result, arg2, UINT16);

prog_char help_test[] = "Test func timewait pump_pwm";
parse_pgm_inst_t cmd_test = {
	.f = cmd_test_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_test,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_test_arg0, 
		(prog_void *)&cmd_test_arg1, 
		(prog_void *)&cmd_test_arg2, 
		NULL,
	},
};


/**********************************************************/
/* armxy */

/* this structure is filled when cmd_armxy is parsed successfully */
struct cmd_armxy_result {
	fixed_string_t arg0;
	int16_t arg1;
	int16_t arg2;
	int16_t arg3;
};



Arm_Pos arm_pos_r;
Arm_Pos arm_pos_l;

#define ARM_STATE_MOV 1
#define ARM_STATE_IN_POS  2



/*
int32_t arm_h = 0;
int32_t arm_d = 260;
int32_t arm_w = 110;
*/

int8_t arm_do_xy_sched_update(Arm_Pos *arm_pos)
{
	
	int8_t ret;
	int32_t fin_h, fin_d;
	int32_t as;
	int32_t ae;
	int32_t aw;
	int32_t s_quad;
	int32_t e_quad;

	uint32_t next_time;
	double as_fin_rad, ae_fin_rad;
	int32_t as_deg, ae_deg;
	//double wrist_out;
	
	//printf("upt called\r\n");
	fin_h = arm_pos->goal_h;
	fin_d = arm_pos->goal_d;
	
	arm_pos->state = ARM_STATE_MOV;

	//quadramp_set_1st_order_vars(&arm.qr_mot, 800, 800); /* set speed */






		
	ret = arm_do_step(arm_pos, &arm_pos->h, &arm_pos->d, 
			  fin_h, fin_d, 
			  &as, &ae, &aw,
			  &as_fin_rad, &ae_fin_rad,
			  &next_time, &s_quad, &e_quad);
	//printf("ret: %d\r\n", ret);
	/*
	printf("do_step: %d arm_h %ld arm_d: %ld as: %ld ae:%ld\r\n",
	       ret,
	       arm_pos->h, arm_pos->d,
	       as, ae);
	printf("asdeg %f aedeg %f nextt:%ld squad: %ld equad: %ld\r\n", 
	       ((as_fin_rad*180)/M_PI), ((ae_fin_rad*180)/M_PI), 
	       next_time,
	       s_quad, e_quad);
	*/

	//printf("ret: %d time: %ld\r\n", ret, next_time);
	as_deg = (as_fin_rad*180.)/M_PI;
	ae_deg = (ae_fin_rad*180.)/M_PI;

	
	aw = -arm_pos->goal_w+as_deg+ae_deg;

	/* printf("as_deg: %ld ae_deg: %ld aw deg: %ld\r\n", as_deg, ae_deg, aw); */
	/*
	wrist_angle_deg2robot(arm_pos->goal_w,
			      &wrist_out);
	*/

	/* printf("w_out: %f\r\n", wrist_out); */
	
	if (ret<0)
		return arm_pos->state = ret; // XXX


	quadramp_set_1st_order_vars(&arm.qr_mot, s_quad, s_quad); /* set speed */
	AX12_write_int(&ax12, arm_pos->ELBOW_AX12, AA_MOVING_SPEED_L, e_quad);



	arm_goto_dh(arm_pos, as, ae);

	//wait_ms(next_time/1000);

	if (ret == 0){
		//printf("set arm end\r\n");
		arm_pos->state = ARM_STATE_IN_POS;
		return 0;
	}

	if (next_time<SCHEDULER_UNIT)
		next_time = SCHEDULER_UNIT;

	ret = scheduler_add_single_event_priority((void *)arm_do_xy_sched_update, 
						  arm_pos, 
						  next_time/SCHEDULER_UNIT, 
						  50);
	

	//printf("add event %ld\r\n", next_time/SCHEDULER_UNIT );
		


	return 0;

}



/*
#define SPEED_NEAR_STOP 15
#define CS_NEAR_STEP 50
*/

#define ARM_NEAR_POS 8
#define ARM_NEAR_SPEED0 100

#define CS_NEAR_POS 100




#define ARM_MAX_AX12_SS 1000


#define MOV_MAX_TIMEOUT 100L



int8_t arm_do_xy_nowait(Arm_Pos *arm_pos, int16_t armh, int16_t armd, int16_t w_angle, int32_t *wrist_wait_time)
{
	int8_t ret;


	double as_fin_rad, ae_fin_rad, wrist_out;
	int32_t as_deg, ae_deg, aw_deg;
	uint16_t wrist_cur;
	int32_t wrist_diff;


	arm_pos->goal_h = armh;
	arm_pos->goal_d = armd;
	arm_pos->goal_w = w_angle;

	arm_pos->state = ARM_STATE_MOV;

	if (wrist_wait_time)
		*wrist_wait_time = 0;

	/* calc for final pos */
	ret = cart2angle(armh, armd, &as_fin_rad, &ae_fin_rad);
	if (ret)
		return ret;


	
	/*give directly final wrist pos*/
	//AX12_write_int(&ax12, WRIST_AX12, AA_MOVING_SPEED_L, 0x3ff);

	as_deg = (as_fin_rad*180.)/M_PI;
	ae_deg = (ae_fin_rad*180.)/M_PI;	
	aw_deg = -arm_pos->goal_w+as_deg+ae_deg;
	arm_pos->wrist_angle_deg2robot(aw_deg,
				       &wrist_out);
	/* calc theorical wrist movement duration */
	ret = AX12_read_int(&ax12, arm_pos->WRIST_AX12, AA_PRESENT_POSITION_L, &wrist_cur);

	wrist_diff = wrist_out-wrist_cur;


	if (ret)
		printf_P(PSTR("AX12 read pos wrist error %x !\r\n"), ret);
	else{
		if  (wrist_diff  && wrist_wait_time)
			*wrist_wait_time = (1000*ABS((int32_t)wrist_diff))/ARM_MAX_AX12_SS;
	}

	ret = AX12_write_int(&ax12, arm_pos->WRIST_AX12, AA_GOAL_POSITION_L, wrist_out);	
	if (ret)
		printf_P(PSTR("AX12 wrist final pos error %x !\r\n"), ret);

	arm_do_xy_sched_update(arm_pos);

	while(arm_pos->state == ARM_STATE_MOV){
	}

	if (arm_pos->state != ARM_STATE_IN_POS)
		printf("error wait no speed ret: %d\r\n", arm_pos->state);
	return ret;
	

}

int8_t arm_do_xy_wait_nospeed(Arm_Pos *arm_pos, int16_t armh, int16_t armd, int16_t w_angle)
{
	int8_t ret;
	int16_t spd1, spd2;
	int16_t val1, val2;
	int16_t valp1, valp2;
	int32_t cs_err;

	microseconds t1,t2 ;

	int32_t wrist_wait_time;


	ret = arm_do_xy_nowait(arm_pos, armh, armd, w_angle, &wrist_wait_time);

	//printf("wwt %ld\r\n", wrist_wait_time);

	//ret = arm_do_xy(arm_pos, armh, armd, w_angle);
	//ret = arm_do_xy_sched(arm_pos, armh, armd, w_angle);

	t1 = time_get_us2();
	while (1){
		AX12_read_int(&ax12, arm_pos->ELBOW_AX12, AA_PRESENT_SPEED_L, (uint16_t*)&spd1);
		AX12_read_int(&ax12, arm_pos->WRIST_AX12, AA_PRESENT_SPEED_L, (uint16_t*)&spd2);
		
		AX12_read_int(&ax12, arm_pos->ELBOW_AX12, AA_GOAL_POSITION_L, (uint16_t*)&val1);
		AX12_read_int(&ax12, arm_pos->WRIST_AX12, AA_GOAL_POSITION_L, (uint16_t*)&val2);

		AX12_read_int(&ax12, arm_pos->ELBOW_AX12, AA_PRESENT_POSITION_L, (uint16_t*)&valp1);
		AX12_read_int(&ax12, arm_pos->WRIST_AX12, AA_PRESENT_POSITION_L, (uint16_t*)&valp2);

		
		val1-=valp1;
		val2-=valp2;
		

		cs_err = cs_get_error(&arm.cs_mot);

		if (ABS(val1)<ARM_NEAR_POS && ABS(val2)<ARM_NEAR_POS && ABS(cs_err)<CS_NEAR_POS &&
		    ABS(spd1)<ARM_NEAR_SPEED0 && ABS(spd2)<ARM_NEAR_SPEED0)
			break;

		t2 = time_get_us2();
		
		/* TIME OUT XXX ms */
		if (t2-t1>(MOV_MAX_TIMEOUT+wrist_wait_time)*1000L){
			printf("wait speed timeout: %d %d %ld spd %d %d\r\n", 
			       val1,val2,
			       ABS(cs_err),
			       spd1, spd2);
			break;
		}

	
	}

	return ret;
}


/* function called when cmd_armxy is parsed successfully */
static void cmd_armxy_parsed(void * parsed_result, void * data)
{
	struct cmd_armxy_result *res = parsed_result;
	int16_t armx = res->arg1;
	int16_t army = res->arg2;
	int16_t wrist_angle_deg = res->arg3;
	Arm_Pos *arm_pos;

	if (!strcmp_P(res->arg0, PSTR("rarmxy")))
		arm_pos = &arm_pos_r;
	else
		arm_pos = &arm_pos_l;


	arm_do_xy_wait_nospeed(arm_pos, armx,army, wrist_angle_deg);

	
	return ;





}

prog_char str_armxy_arg0[] = "larmxy#rarmxy";
parse_pgm_token_string_t cmd_armxy_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_armxy_result, arg0, str_armxy_arg0);
parse_pgm_token_num_t cmd_armxy_arg1 = TOKEN_NUM_INITIALIZER(struct cmd_armxy_result, arg1, INT16);
parse_pgm_token_num_t cmd_armxy_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_armxy_result, arg2, INT16);
parse_pgm_token_num_t cmd_armxy_arg3 = TOKEN_NUM_INITIALIZER(struct cmd_armxy_result, arg3, INT16);

prog_char help_armxy[] = "Armxy x y ";
parse_pgm_inst_t cmd_armxy = {
	.f = cmd_armxy_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_armxy,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_armxy_arg0, 
		(prog_void *)&cmd_armxy_arg1, 
		(prog_void *)&cmd_armxy_arg2, 
		(prog_void *)&cmd_armxy_arg3, 
		NULL,
	},
};





/**********************************************************/
/* arm_circ */

/* this structure is filled when cmd_arm_circ is parsed successfully */
struct cmd_arm_circ_result {
	fixed_string_t arg0;
	uint8_t step;
};

/* function called when cmd_arm_circ is parsed successfully */
static void cmd_arm_circ_parsed(void * parsed_result, void * data)
{
#if 0
	int32_t i;
	double a;
	struct cmd_arm_circ_result *res = parsed_result;
	//int32_t l = 90;
	int32_t add_h, add_d;

	add_h = 0;
	add_d = 170;

	/*
	for (i=0; i<360; i+=res->step) {
		a = (double)(i*2*M_PI/360.);
		arm_do_xy(&arm_pos_r, add_h+l*sin(a), add_d+l*cos(a) , 10);
			
	}
	*/
#endif
}

prog_char str_arm_circ_arg0[] = "arm_circ";
parse_pgm_token_string_t cmd_arm_circ_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_arm_circ_result, arg0, str_arm_circ_arg0);
parse_pgm_token_num_t cmd_arm_circ_arg1 = TOKEN_NUM_INITIALIZER(struct cmd_arm_circ_result, step, UINT8);

prog_char help_arm_circ[] = "Arm_Circ the board";
parse_pgm_inst_t cmd_arm_circ = {
	.f = cmd_arm_circ_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_arm_circ,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_arm_circ_arg0, 
		(prog_void *)&cmd_arm_circ_arg1, 
		NULL,
	},
};



/**********************************************************/
/* arm_harv */

/* this structure is filled when cmd_arm_harv is parsed successfully */
struct cmd_arm_harv_result {
	fixed_string_t arg0;
	uint16_t wait;
	int16_t hight;
};

#define ANGL1 2
#define ANGL2 110

#define PWM_PUMP -3000

#define ARM_GET_D 68

//#define ARM_LOW_H -150
#define ARM_LOW_D 210

#define FINGER_DELAY 200


#define ARM_GET_H -150
/* function called when cmd_arm_harv is parsed successfully */
static void cmd_arm_harv_parsed(void * parsed_result, void * data)
{
	struct cmd_arm_harv_result *res = parsed_result;

	/*get column*/
	finger_left();
        wait_ms(FINGER_DELAY);
	finger_center();
        wait_ms(FINGER_DELAY);

        arm_do_xy_nowait(&arm_pos_r, ARM_GET_H+10, ARM_GET_D, ANGL1, NULL);

	finger_right();
        wait_ms(FINGER_DELAY);

	/* TAKE COLUMN */
        //arm_do_xy_wait_nospeed(&arm_pos_r, 130, 130, ANGL1);

        arm_do_xy_wait_nospeed(&arm_pos_r, ARM_GET_H, ARM_GET_D, ANGL1);

	finger_left();

        wait_ms(res->wait);

	pwm_ng_set(&arm.pwm1B, PWM_PUMP);  
        wait_ms(40);
	
        //self.ser.write("pwm 1B -3000\n");
        wait_ms(res->wait);

        //arm_do_xy_wait_nospeed(&arm_pos_r, -120, ARM_GET_D, ANGL1);
        wait_ms(res->wait);
        arm_do_xy_wait_nospeed(&arm_pos_r, ARM_GET_H+10, ARM_GET_D, ANGL2);



	/* WAIT LOLO FOR COLUMN*/
	finger_right();
        wait_ms(600);

        wait_ms(res->wait);
        arm_do_xy_wait_nospeed(&arm_pos_r, ARM_GET_H, ARM_GET_D, ANGL2);
	pwm_ng_set(&arm.pwm3C, PWM_PUMP);

        wait_ms(res->wait);


	finger_left();
        wait_ms(40);
	

        arm_do_xy_wait_nospeed(&arm_pos_r, res->hight+80,ARM_LOW_D-140,ANGL2);
        wait_ms(res->wait);
        arm_do_xy_wait_nospeed(&arm_pos_r, res->hight+80, ARM_LOW_D, ANGL2);
        wait_ms(res->wait);





	/******** MAKE COLUMN *******/
        
        arm_do_xy_wait_nospeed(&arm_pos_r, res->hight+10, ARM_LOW_D, ANGL2);
        //plak arm_do_xy_wait_nospeed(&arm_pos_r, res->hight, ARM_LOW_D, ANGL2);
        wait_ms(res->wait);
        wait_ms(150);
	pwm_ng_set(&arm.pwm3C, -PWM_PUMP);
        wait_ms(res->wait);
        wait_ms(100);


        //arm_do_xy_nowait(&arm_pos_r, res->hight+65,ARM_LOW_D,ANGL2, NULL);
        wait_ms(res->wait);

        arm_do_xy_wait_nospeed(&arm_pos_r, res->hight+65,ARM_LOW_D,ANGL1);
        wait_ms(res->wait);
        arm_do_xy_wait_nospeed(&arm_pos_r, res->hight+40,ARM_LOW_D,ANGL1);
        //plak arm_do_xy_wait_nospeed(&arm_pos_r, res->hight+30,ARM_LOW_D,ANGL1);
        wait_ms(res->wait);
        wait_ms(150);
	pwm_ng_set(&arm.pwm1B, -PWM_PUMP);
        wait_ms(res->wait);
        wait_ms(100);

        arm_do_xy_nowait(&arm_pos_r, res->hight+70,ARM_LOW_D,ANGL1, NULL);


	pwm_ng_set(&arm.pwm1B, 0);
	pwm_ng_set(&arm.pwm3C, 0);


        arm_do_xy_nowait(&arm_pos_r, res->hight+80,ARM_LOW_D-140,ANGL1, NULL);

        arm_do_xy_nowait(&arm_pos_r, -100,80,ANGL1, NULL);

}

prog_char str_arm_harv_arg0[] = "arm_harv";
parse_pgm_token_string_t cmd_arm_harv_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_arm_harv_result, arg0, str_arm_harv_arg0);
parse_pgm_token_num_t cmd_arm_harv_arg1 = TOKEN_NUM_INITIALIZER(struct cmd_arm_harv_result, wait, UINT16);
parse_pgm_token_num_t cmd_arm_harv_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_arm_harv_result, hight, INT16);

prog_char help_arm_harv[] = "Arm_Harv the board";
parse_pgm_inst_t cmd_arm_harv = {
	.f = cmd_arm_harv_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_arm_harv,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_arm_harv_arg0, 
		(prog_void *)&cmd_arm_harv_arg1, 
		(prog_void *)&cmd_arm_harv_arg2, 
		NULL,
	},
};




/**********************************************************/
/* Arm_Straight */

/* this structure is filled when cmd_arm_straight is parsed successfully */
struct cmd_arm_straight_result {
	fixed_string_t arg0;
	uint32_t arg1;
	uint32_t arg2;
	fixed_string_t arg3;
};

/* function called when cmd_arm_straight is parsed successfully */
static void cmd_arm_straight_parsed(void * parsed_result, void * data)
{
	int32_t pos_shoulder[] = {
		-15510,
		-17181,
		-18852,
		-20524,
		-22470,
		-24416,
		-26363,
		-28309,
		-30255,
		-32464,
		-34673,
		-36881,
		-39090,
		-41299,
		-43080,
		-44861,
		-46642,
		-48423,
		-50204,
		-51026,
		-51849,
		-52671,
		-53493,
		-54316,
		-54443,
		-54370,
		-54398,
	};
	/* vitesse servo : 3ff = 114RPM = 233.2 pas/100ms */
	int32_t pos_elbow[] = {
		316,
		301,
		286,
		271,
		261,
		250,
		240,
		230,
		220,
		216,
		212,
		208,
		204,
		200,
		204,
		208,
		212,
		216,
		220,
		230,
		240,
		250,
		261,
		271,
		286,
		301,
		316,
	};

	int8_t i;
	int32_t speed_shoulder;
	int32_t speed_elbow;
 	int32_t arm_period;
	int8_t arm_step;
	int8_t arm_max_pos;
	int8_t arm_world;
	microseconds t_start;

	int8_t tab_len = sizeof(pos_elbow)/sizeof(int32_t);

	struct cmd_arm_straight_result *res = parsed_result;
 
	arm_period = res->arg1;
	arm_step = res->arg2;
	if (arm_step<0 || arm_step > tab_len){
		printf("bad step => reset to 1\r\n");
		arm_step = 1;
	}
	arm_world = strcmp("real", res->arg3);
	printf("speed: %ld step:%d\r\n", arm_period, arm_step);

	//arm_goto(pos_shoulder[0], pos_elbow[0], 500);
	printf("ready\r\n");
	while(uart_recv_nowait(0) == -1);	

	arm_max_pos = (tab_len/arm_step)*arm_step;

	printf("tstart %ld\r\n", time_get_us2());
	/* 50 ms per incr */
	for (i=0; i<tab_len-arm_step; i+=arm_step) {
		t_start = time_get_us2();

		speed_shoulder = pos_shoulder[i+arm_step] - pos_shoulder[i];
		speed_shoulder /= arm_period/CS_PERIOD; /* period is 5ms */
		if (speed_shoulder < 0)
			speed_shoulder = -speed_shoulder;
		
		speed_elbow = pos_elbow[i+arm_step] - pos_elbow[i];
		speed_elbow *= 0x3ff;
		speed_elbow /= (233*arm_period/100000);
		if (speed_elbow < 0)
			speed_elbow = -speed_elbow;

		if (arm_world){
			printf("shoulder : %ld, %ld / elbow : %ld, %ld\r\n",
			       pos_shoulder[i], speed_shoulder,
			       pos_elbow[i], speed_elbow);
		}
		else{
			quadramp_set_1st_order_vars(&arm.qr_mot, speed_shoulder, speed_shoulder);
			cs_set_consign(&arm.cs_mot, pos_shoulder[i]);
			
			AX12_write_int(&ax12, R_ELBOW_AX12, AA_MOVING_SPEED_L, speed_elbow);
			AX12_write_int(&ax12, R_ELBOW_AX12, AA_GOAL_POSITION_L, pos_elbow[i]);
			while(time_get_us2() - t_start < arm_period);

		}
	}
	printf("tstop %ld\r\n", time_get_us2());


	/* reset QUADRAMP */
	quadramp_set_1st_order_vars(&arm.qr_mot, 800, 800); /* set speed */
	quadramp_set_2nd_order_vars(&arm.qr_mot, 100, 100); /* set accel */

}

prog_char str_arm_straight_arg0[] = "arm_straight";
parse_pgm_token_string_t cmd_arm_straight_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_arm_straight_result, arg0, str_arm_straight_arg0);
parse_pgm_token_num_t cmd_arm_straight_arg1 = TOKEN_NUM_INITIALIZER(struct cmd_arm_straight_result, arg1, UINT32);
parse_pgm_token_num_t cmd_arm_straight_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_arm_straight_result, arg2, UINT32);
prog_char str_event_arg3[] = "sim#real";
parse_pgm_token_string_t cmd_arm_straight_arg3 = TOKEN_STRING_INITIALIZER(struct cmd_arm_straight_result, arg3, str_event_arg3);


prog_char help_arm_straight[] = "Arm_Straight func speed step";
parse_pgm_inst_t cmd_arm_straight = {
	.f = cmd_arm_straight_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_arm_straight,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_arm_straight_arg0, 
		(prog_void *)&cmd_arm_straight_arg1, 
		(prog_void *)&cmd_arm_straight_arg2, 
		(prog_void *)&cmd_arm_straight_arg3, 
		NULL,
	},
};

/**********************************************************/

/* this structure is filled when cmd_baudrate is parsed successfully */
struct cmd_baudrate_result {
	fixed_string_t arg0;
	uint32_t arg1;
};

/* function called when cmd_baudrate is parsed successfully */
static void cmd_baudrate_parsed(void * parsed_result, void * data)
{
	struct cmd_baudrate_result *res = parsed_result;
	struct uart_config c;

	printf_P(PSTR("%d %d\r\n"), UBRR1H, UBRR1L);
	uart_getconf(1, &c);
	c.baudrate = res->arg1;
	uart_setconf(1, &c);
	printf_P(PSTR("%d %d\r\n"), UBRR1H, UBRR1L);
}

prog_char str_baudrate_arg0[] = "baudrate";
parse_pgm_token_string_t cmd_baudrate_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_baudrate_result, arg0, str_baudrate_arg0);
parse_pgm_token_num_t cmd_baudrate_arg1 = TOKEN_NUM_INITIALIZER(struct cmd_baudrate_result, arg1, UINT32);

prog_char help_baudrate[] = "Change ax12 baudrate";
parse_pgm_inst_t cmd_baudrate = {
	.f = cmd_baudrate_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_baudrate,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_baudrate_arg0, 
		(prog_void *)&cmd_baudrate_arg1, 
		NULL,
	},
};

/**********************************************************/

/* this structure is filled when cmd_arm_goto is parsed successfully */
struct cmd_arm_goto_result {
	fixed_string_t arg0;
	int32_t arg1;
	uint16_t arg2;
	uint16_t arg3;
};

/* function called when cmd_arm_goto is parsed successfully */
static void cmd_arm_goto_parsed(void * parsed_result, void * data)
{
	struct cmd_arm_goto_result *res = parsed_result;
	Arm_Pos * arm_pos;
	if (!strcmp(res->arg0, "rarm_goto"))
		arm_pos = &arm_pos_r;
	else
		arm_pos = &arm_pos_l;
	arm_goto(arm_pos, res->arg1, res->arg2, res->arg3);
		
}

prog_char str_arm_goto_arg0[] = "larm_goto#rarm_goto";
parse_pgm_token_string_t cmd_arm_goto_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_arm_goto_result, arg0, str_arm_goto_arg0);
parse_pgm_token_num_t cmd_arm_goto_arg1 = TOKEN_NUM_INITIALIZER(struct cmd_arm_goto_result, arg1, INT32);
parse_pgm_token_num_t cmd_arm_goto_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_arm_goto_result, arg2, UINT16);
parse_pgm_token_num_t cmd_arm_goto_arg3 = TOKEN_NUM_INITIALIZER(struct cmd_arm_goto_result, arg3, UINT16);

prog_char help_arm_goto[] = "Change arm position (shoulder, elbow, wrist)";
parse_pgm_inst_t cmd_arm_goto = {
	.f = cmd_arm_goto_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_arm_goto,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_arm_goto_arg0, 
		(prog_void *)&cmd_arm_goto_arg1, 
		(prog_void *)&cmd_arm_goto_arg2, 
		(prog_void *)&cmd_arm_goto_arg3, 
		NULL,
	},
};

/**********************************************************/

/* this structure is filled when cmd_arm_capture is parsed successfully */
struct cmd_arm_capture_result {
	fixed_string_t arg0;
};

/* function called when cmd_arm_capture is parsed successfully */
static void cmd_arm_capture_parsed(void * parsed_result, void * data)
{
	struct cmd_arm_goto_result *res = parsed_result;
	uint16_t elbow, wrist;
	int32_t shoulder;
	uint8_t ret = 0;
	Arm_Pos *arm_pos;

	if (!strcmp_P(res->arg0, "rarm_capture"))
		arm_pos = &arm_pos_r;
	else
		arm_pos = &arm_pos_l;


	ret |= AX12_read_int(&ax12, arm_pos->ELBOW_AX12, AA_PRESENT_POSITION_L, &elbow);
	ret |= AX12_read_int(&ax12, arm_pos->WRIST_AX12, AA_PRESENT_POSITION_L, &wrist);
	shoulder = encoders_microb_get_value((void *)ARM_ENC);
	if (ret)
		printf_P(PSTR("AX12 error %.2x!\r\n"), ret);
	printf_P(PSTR("%ld %d %d\r\n"), shoulder, elbow, wrist);
}

prog_char str_arm_capture_arg0[] = "rarm_capture#larm_capture";
parse_pgm_token_string_t cmd_arm_capture_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_arm_capture_result, arg0, str_arm_capture_arg0);

prog_char help_arm_capture[] = "Change arm position (shoulder, elbow, wrist)";
parse_pgm_inst_t cmd_arm_capture = {
	.f = cmd_arm_capture_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_arm_capture,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_arm_capture_arg0, 
		NULL,
	},
};
/**********************************************************/
/* Uint16 */


/* this structure is filled when cmd_uint16_read is parsed successfully */
struct cmd_uint16_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	uint8_t num;
	uint16_t val;
};

/* function called when cmd_uint16_read is parsed successfully */
static void cmd_uint16_read_parsed(void * parsed_result, void * data)
{
	struct cmd_uint16_result *res = parsed_result;
	uint8_t ret;
	uint16_t val;
	uint8_t addr = addr_from_string(res->arg1);
	ret = AX12_read_int(&ax12, res->num, addr, &val);
	if (ret)
		printf_P(PSTR("AX12 error %.2x!\r\n"), ret);
	printf_P(PSTR("%s: %d [0x%.4x]\r\n"), res->arg1, val, val);
}

prog_char str_uint16_arg0[] = "read";
parse_pgm_token_string_t cmd_uint16_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_uint16_result, arg0, str_uint16_arg0);
prog_char str_uint16_arg1[] = "moving_speed#model#goal_pos#cw_angle_limit#ccw_angle_limit#"
		"max_torque#down_calibration#up_calibration#torque_limit#"
		"position#speed#load#punch";
parse_pgm_token_string_t cmd_uint16_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_uint16_result, arg1, str_uint16_arg1);
parse_pgm_token_num_t cmd_uint16_num = TOKEN_NUM_INITIALIZER(struct cmd_uint16_result, num, UINT8);

prog_char help_uint16_read[] = "Read uint16 value (type, num)";
parse_pgm_inst_t cmd_uint16_read = {
	.f = cmd_uint16_read_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_uint16_read,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_uint16_arg0,
		(prog_void *)&cmd_uint16_arg1,
		(prog_void *)&cmd_uint16_num,
		NULL,
	},
};

/* function called when cmd_uint16_write is parsed successfully */
static void cmd_uint16_write_parsed(void * parsed_result, void * data)
{
	struct cmd_uint16_result *res = parsed_result;
	uint8_t ret;
	uint8_t addr = addr_from_string(res->arg1);
	printf_P(PSTR("writing %s: %d [0x%.4x]\r\n"), res->arg1,
		 res->val, res->val);
	ret = AX12_write_int(&ax12, res->num, addr, res->val);
	if (ret)
		printf_P(PSTR("AX12 error %.2x!\r\n"), ret);
}

prog_char str_uint16_arg0_w[] = "write";
parse_pgm_token_string_t cmd_uint16_arg0_w = TOKEN_STRING_INITIALIZER(struct cmd_uint16_result, arg0, str_uint16_arg0_w);
prog_char str_uint16_arg1_w[] = "moving_speed#goal_pos#cw_angle_limit#ccw_angle_limit#"
		"max_torque#torque_limit#punch";
parse_pgm_token_string_t cmd_uint16_arg1_w = TOKEN_STRING_INITIALIZER(struct cmd_uint16_result, arg1, str_uint16_arg1_w);
parse_pgm_token_num_t cmd_uint16_val = TOKEN_NUM_INITIALIZER(struct cmd_uint16_result, val, UINT16);

prog_char help_uint16_write[] = "Write uint16 value (write, num, val)";
parse_pgm_inst_t cmd_uint16_write = {
	.f = cmd_uint16_write_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_uint16_write,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_uint16_arg0_w,
		(prog_void *)&cmd_uint16_arg1_w,
		(prog_void *)&cmd_uint16_num,
		(prog_void *)&cmd_uint16_val,
		NULL,
	},
};

/**********************************************************/
/* Uint8 */


/* this structure is filled when cmd_uint8_read is parsed successfully */
struct cmd_uint8_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	uint8_t num;
	uint8_t val;
};

/* function called when cmd_uint8_read is parsed successfully */
static void cmd_uint8_read_parsed(void * parsed_result, void * data)
{
	struct cmd_uint8_result *res = parsed_result;
	uint8_t ret;
	uint8_t val;
	uint8_t addr = addr_from_string(res->arg1);

	ret = AX12_read_byte(&ax12, res->num, addr, &val);
	if (ret)
		printf_P(PSTR("AX12 error %.2x!\r\n"), ret);
	printf_P(PSTR("%s: %d [0x%.2x]\r\n"), res->arg1, val, val);
}

prog_char str_uint8_arg0[] = "read";
parse_pgm_token_string_t cmd_uint8_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_uint8_result, arg0, str_uint8_arg0);
prog_char str_uint8_arg1[] = "id#firmware#baudrate#delay#high_lim_temp#"
		"low_lim_volt#high_lim_volt#status_return#alarm_led#"
		"alarm_shutdown#torque_enable#led#cw_comp_margin#"
		"ccw_comp_margin#cw_comp_slope#ccw_comp_slope#"
		"voltage#temp#reginst#moving#lock";
parse_pgm_token_string_t cmd_uint8_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_uint8_result, arg1, str_uint8_arg1);
parse_pgm_token_num_t cmd_uint8_num = TOKEN_NUM_INITIALIZER(struct cmd_uint8_result, num, UINT8);

prog_char help_uint8_read[] = "Read uint8 value (type, num)";
parse_pgm_inst_t cmd_uint8_read = {
	.f = cmd_uint8_read_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_uint8_read,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_uint8_arg0,
		(prog_void *)&cmd_uint8_arg1,
		(prog_void *)&cmd_uint8_num,
		NULL,
	},
};

/* function called when cmd_uint8_write is parsed successfully */
static void cmd_uint8_write_parsed(void * parsed_result, void * data)
{
	struct cmd_uint8_result *res = parsed_result;
	uint8_t addr = addr_from_string(res->arg1);
	uint8_t ret;
	printf_P(PSTR("writing %s: %d [0x%.2x]\r\n"), res->arg1, 
		 res->val, res->val);
	ret = AX12_write_byte(&ax12, res->num, addr, res->val);
	if (ret)
		printf_P(PSTR("AX12 error %.2x!\r\n"), ret);
}

prog_char str_uint8_arg0_w[] = "write";
parse_pgm_token_string_t cmd_uint8_arg0_w = TOKEN_STRING_INITIALIZER(struct cmd_uint8_result, arg0, str_uint8_arg0_w);
prog_char str_uint8_arg1_w[] = "id#baudrate#delay#high_lim_temp#"
		"low_lim_volt#high_lim_volt#status_return#alarm_led#"
		"alarm_shutdown#torque_enable#led#cw_comp_margin#"
		"ccw_comp_margin#cw_comp_slope#ccw_comp_slope#"
		"reginst#lock";
parse_pgm_token_string_t cmd_uint8_arg1_w = TOKEN_STRING_INITIALIZER(struct cmd_uint8_result, arg1, str_uint8_arg1_w);
parse_pgm_token_num_t cmd_uint8_val = TOKEN_NUM_INITIALIZER(struct cmd_uint8_result, val, UINT8);

prog_char help_uint8_write[] = "Write uint8 value (write, num, val)";
parse_pgm_inst_t cmd_uint8_write = {
	.f = cmd_uint8_write_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_uint8_write,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_uint8_arg0_w,
		(prog_void *)&cmd_uint8_arg1_w,
		(prog_void *)&cmd_uint8_num,
		(prog_void *)&cmd_uint8_val,
		NULL,
	},
};

/**********************************************************/
/* Encoders tests */

/* this structure is filled when cmd_encoders is parsed successfully */
struct cmd_encoders_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_encoders is parsed successfully */
static void cmd_encoders_parsed(void * parsed_result, void * data)
{
	while(uart_recv_nowait(0) == -1) {
		printf_P(PSTR("% .8ld % .8ld % .8ld % .8ld\r\n"), 
			 encoders_microb_get_value((void *)0),
			 encoders_microb_get_value((void *)1),
			 encoders_microb_get_value((void *)2),
			 encoders_microb_get_value((void *)3));
		wait_ms(100);
	}
}

prog_char str_encoders_arg0[] = "encoders";
parse_pgm_token_string_t cmd_encoders_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_encoders_result, arg0, str_encoders_arg0);
prog_char str_encoders_arg1[] = "show";
parse_pgm_token_string_t cmd_encoders_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_encoders_result, arg1, str_encoders_arg1);

prog_char help_encoders[] = "Show encoders values";
parse_pgm_inst_t cmd_encoders = {
	.f = cmd_encoders_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_encoders,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_encoders_arg0, 
		(prog_void *)&cmd_encoders_arg1, 
		NULL,
	},
};

/**********************************************************/
/* Pwms tests */

/* this structure is filled when cmd_pwm is parsed successfully */
struct cmd_pwm_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	int16_t arg2;
};

/* function called when cmd_pwm is parsed successfully */
static void cmd_pwm_parsed(void * parsed_result, void * data)
{
	void * pwm_ptr = NULL;
	struct cmd_pwm_result * res = parsed_result;
	
	DDRB |= 0x84;
	
	if (!strcmp_P(res->arg1, PSTR("1A")))
		pwm_ptr = &arm.pwm1A;
	else if (!strcmp_P(res->arg1, PSTR("1B")))
		pwm_ptr = &arm.pwm1B;
	else if (!strcmp_P(res->arg1, PSTR("3C")))
		pwm_ptr = &arm.pwm3C;
	else if (!strcmp_P(res->arg1, PSTR("2"))) {
		if (res->arg2 == 0)
			PORTB &= ~0x84;
		if (res->arg2 == 1) {
			PORTB &= ~0x80;
			PORTB |= 0x4;
		}
		if (res->arg2 == 0) {
			PORTB &= ~0x04;
			PORTB |= 0x80;
		}
		if (res->arg2 == 0)
			PORTB |= 0x84;
		
		//pwm_ptr = &arm.pwm2;

	}
	
	if (pwm_ptr)
		pwm_ng_set(pwm_ptr, res->arg2);

	printf_P(PSTR("done\r\n"));
}

prog_char str_pwm_arg0[] = "pwm";
parse_pgm_token_string_t cmd_pwm_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_pwm_result, arg0, str_pwm_arg0);
prog_char str_pwm_arg1[] = "1A#1B#3C#2";
parse_pgm_token_string_t cmd_pwm_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_pwm_result, arg1, str_pwm_arg1);
parse_pgm_token_num_t cmd_pwm_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_pwm_result, arg2, INT16);

prog_char help_pwm[] = "Set pwm values [-4096 ; 4095]";
parse_pgm_inst_t cmd_pwm = {
	.f = cmd_pwm_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_pwm,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_pwm_arg0, 
		(prog_void *)&cmd_pwm_arg1, 
		(prog_void *)&cmd_pwm_arg2, 
		NULL,
	},
};

/**********************************************************/
/* Gains for control system */

/* this structure is filled when cmd_gain is parsed successfully */
struct cmd_gain_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	int16_t p;
	int16_t i;
	int16_t d;
};

/* function called when cmd_gain is parsed successfully */
static void cmd_gain_parsed(void * parsed_result, void * data)
{
	struct cmd_gain_result * res = parsed_result;
	
	if (!strcmp_P(res->arg1, PSTR("arm"))) {
		pid_set_gains(&arm.pid_mot, res->p, res->i, res->d);
	}
	/* else it is a "show" */

	printf_P(PSTR("arm %d %d %d\r\n"), 
		 pid_get_gain_P(&arm.pid_mot),
		 pid_get_gain_I(&arm.pid_mot),
		 pid_get_gain_D(&arm.pid_mot));
}

prog_char str_gain_arg0[] = "gain";
parse_pgm_token_string_t cmd_gain_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_gain_result, arg0, str_gain_arg0);
prog_char str_gain_arg1[] = "arm";
parse_pgm_token_string_t cmd_gain_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_gain_result, arg1, str_gain_arg1);
parse_pgm_token_num_t cmd_gain_p = TOKEN_NUM_INITIALIZER(struct cmd_gain_result, p, INT16);
parse_pgm_token_num_t cmd_gain_i = TOKEN_NUM_INITIALIZER(struct cmd_gain_result, i, INT16);
parse_pgm_token_num_t cmd_gain_d = TOKEN_NUM_INITIALIZER(struct cmd_gain_result, d, INT16);

prog_char help_gain[] = "Set gain values for PID";
parse_pgm_inst_t cmd_gain = {
	.f = cmd_gain_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_gain,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_gain_arg0, 
		(prog_void *)&cmd_gain_arg1, 
		(prog_void *)&cmd_gain_p, 
		(prog_void *)&cmd_gain_i, 
		(prog_void *)&cmd_gain_d, 
		NULL,
	},
};

/* show */

prog_char str_gain_show_arg[] = "show";
parse_pgm_token_string_t cmd_gain_show_arg = TOKEN_STRING_INITIALIZER(struct cmd_gain_result, arg1, str_gain_show_arg);

prog_char help_gain_show[] = "Show gain values for PID";
parse_pgm_inst_t cmd_gain_show = {
	.f = cmd_gain_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_gain_show,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_gain_arg0, 
		(prog_void *)&cmd_gain_show_arg,
		NULL,
	},
};


/**********************************************************/
/* Speeds for control system */

/* this structure is filled when cmd_speed is parsed successfully */
struct cmd_speed_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	uint16_t s;
};

/* function called when cmd_speed is parsed successfully */
static void cmd_speed_parsed(void * parsed_result, void * data)
{
#if 0
	struct cmd_speed_result * res = parsed_result;
	
	if (!strcmp_P(res->arg1, PSTR("arm"))) {
		ramp_set_vars(&ext.r_b, res->s, res->s); /* set speed */
	}

	printf_P(PSTR("arm %lu\r\n"), 
		 ext.r_b.var_pos);
#else
	printf_P(PSTR("DISABLED FOR NOW\r\n"));
#endif
}

prog_char str_speed_arg0[] = "speed";
parse_pgm_token_string_t cmd_speed_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_speed_result, arg0, str_speed_arg0);
prog_char str_speed_arg1[] = "arm#show";
parse_pgm_token_string_t cmd_speed_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_speed_result, arg1, str_speed_arg1);
parse_pgm_token_num_t cmd_speed_s = TOKEN_NUM_INITIALIZER(struct cmd_speed_result, s, UINT16);

prog_char help_speed[] = "Set speed values for ramp filter";
parse_pgm_inst_t cmd_speed = {
	.f = cmd_speed_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_speed,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_speed_arg0, 
		(prog_void *)&cmd_speed_arg1, 
		(prog_void *)&cmd_speed_s, 
		NULL,
	},
};

/* show */

prog_char str_speed_show_arg[] = "show";
parse_pgm_token_string_t cmd_speed_show_arg = TOKEN_STRING_INITIALIZER(struct cmd_speed_result, arg1, str_speed_show_arg);

prog_char help_speed_show[] = "Show speed values for ramp filter";
parse_pgm_inst_t cmd_speed_show = {
	.f = cmd_speed_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_speed_show,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_speed_arg0, 
		(prog_void *)&cmd_speed_show_arg,
		NULL,
	},
};


/**********************************************************/
/* Pos for control system */

/* this structure is filled when cmd_pos is parsed successfully */
struct cmd_pos_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	int32_t p;
};

/* function called when cmd_pos is parsed successfully */
static void cmd_pos_parsed(void * parsed_result, void * data)
{
	struct cmd_pos_result * res = parsed_result;
	//	uint8_t i;

	if (!strcmp_P(res->arg1, PSTR("arm"))) {
		cs_set_consign(&arm.cs_mot, res->p);
	}
	
#if 0
	for (i=0; i<50; i++) {
		printf("%ld %ld %ld\r\n",
		       pid_get_value_in(&arm.pid_mot),
		       pid_get_value_out(&arm.pid_mot),
		       pid_get_value_D(&arm.pid_mot)
		       );
	}
#endif
}

prog_char str_pos_arg0[] = "pos";
parse_pgm_token_string_t cmd_pos_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_pos_result, arg0, str_pos_arg0);
prog_char str_pos_arg1[] = "arm";
parse_pgm_token_string_t cmd_pos_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_pos_result, arg1, str_pos_arg1);
parse_pgm_token_num_t cmd_pos_p = TOKEN_NUM_INITIALIZER(struct cmd_pos_result, p, INT32);

prog_char help_pos[] = "Set pos value";
parse_pgm_inst_t cmd_pos = {
	.f = cmd_pos_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_pos,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_pos_arg0, 
		(prog_void *)&cmd_pos_arg1, 
		(prog_void *)&cmd_pos_p, 
		NULL,
	},
};


/**********************************************************/
/* Events on/off */

/* this structure is filled when cmd_event is parsed successfully */
struct cmd_event_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	fixed_string_t arg2;
};


void init_arm(void)
{
	uint32_t shoulder_robot;
	uint16_t elbow_robot, wrist_robot;
	double shoulder_rad, elbow_rad;
	int32_t h, d;

	/* init arm xy pos */
	shoulder_robot = encoders_microb_get_value(ARM_ENC);
	AX12_read_int(&ax12, R_ELBOW_AX12, AA_PRESENT_POSITION_L, &elbow_robot);
	AX12_read_int(&ax12, R_WRIST_AX12, AA_PRESENT_POSITION_L, &wrist_robot);
	
	AX12_write_int(&ax12, R_ELBOW_AX12, AA_MOVING_SPEED_L, 0x3ff);
	AX12_write_int(&ax12, R_WRIST_AX12, AA_MOVING_SPEED_L, 0x3ff);


	arm_pos_r.wrist_angle_deg2robot = wrist_angle_deg2robot_r;
	arm_pos_l.wrist_angle_deg2robot = wrist_angle_deg2robot_l;

	arm_pos_r.angle_rad2robot = angle_rad2robot_r;
	arm_pos_l.angle_rad2robot = angle_rad2robot_l;

	arm_pos_r.angle_robot2rad = angle_robot2rad_r;
	arm_pos_l.angle_robot2rad = angle_robot2rad_l;

	arm_pos_r.ELBOW_AX12 = R_ELBOW_AX12;
	arm_pos_l.ELBOW_AX12 = L_ELBOW_AX12;

	arm_pos_r.WRIST_AX12 = R_WRIST_AX12;
	arm_pos_l.WRIST_AX12 = L_WRIST_AX12;


	arm_pos_r.angle_robot2rad(shoulder_robot, elbow_robot,
				   &shoulder_rad, &elbow_rad);
	angle2cart(shoulder_rad, elbow_rad, &h, &d);
	printf("init: h:%ld d:%ld w:%d\r\n", h, d, wrist_robot);
	
	arm_pos_r.h = h;
	arm_pos_r.d = d;
	arm_pos_r.w = wrist_robot;
	arm_pos_r.goal_h = h;
	arm_pos_r.goal_d = d;
	arm_pos_r.goal_w = wrist_robot;
	arm_pos_r.state = ARM_STATE_IN_POS;


	arm_pos_l.h = h;
	arm_pos_l.d = d;
	arm_pos_l.w = wrist_robot;
	arm_pos_l.goal_h = h;
	arm_pos_l.goal_d = d;
	arm_pos_l.goal_w = wrist_robot;
	arm_pos_l.state = ARM_STATE_IN_POS;


		
}


/* function called when cmd_event is parsed successfully */
static void cmd_event_parsed(void * parsed_result, void * data)
{
	u08 bit=0;
	struct cmd_event_result * res = parsed_result;


	
	if (!strcmp_P(res->arg1, PSTR("cs"))) {
		if (!strcmp_P(res->arg2, PSTR("on"))) {
			pwm_ng_set(ARM_MOT_PWM, 0);
			printf_P(PSTR("ax12 will start\r\n"));
			while(uart_recv_nowait(0) == -1);
			AX12_write_int(&ax12,0xFE,AA_TORQUE_ENABLE,0x1);
			AX12_write_int(&ax12, R_ELBOW_AX12, AA_GOAL_POSITION_L, 660);
			AX12_write_int(&ax12, R_WRIST_AX12, AA_GOAL_POSITION_L, 613);
			printf_P(PSTR("Set the arm to 0\r\n"));
			while(uart_recv_nowait(0) == -1);
			encoders_microb_set_value(ARM_ENC, 0);


			printf_P(PSTR("Set scanner to 0\r\n"));
			while(uart_recv_nowait(0) == -1);
			encoders_microb_set_value(SCANNER_ENC, 0);
			scanner.flags |= CS_ON; 

			init_arm();


		}
		bit = CS_ON;
	}
/* 	else if (!strcmp_P(res->arg1, PSTR("catapult"))) */
/* 		bit = CATAPULT_CS_ON; */
	
	if (!strcmp_P(res->arg2, PSTR("on")))
		arm.flags |= bit;
	else if (!strcmp_P(res->arg2, PSTR("off")))
		arm.flags &= (~bit);
	
	printf_P(PSTR("%s is %s\r\n"), res->arg1, 
		      (bit & arm.flags) ? "on":"off");
}

prog_char str_event_arg0[] = "event";
parse_pgm_token_string_t cmd_event_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_event_result, arg1, str_event_arg0);
prog_char str_event_arg1[] = "cs";
parse_pgm_token_string_t cmd_event_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_event_result, arg1, str_event_arg1);
prog_char str_event_arg2[] = "on#off#show";
parse_pgm_token_string_t cmd_event_arg2 = TOKEN_STRING_INITIALIZER(struct cmd_event_result, arg2, str_event_arg2);

prog_char help_event[] = "Enable/disable events";
parse_pgm_inst_t cmd_event = {
	.f = cmd_event_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_event,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_event_arg0, 
		(prog_void *)&cmd_event_arg1, 
		(prog_void *)&cmd_event_arg2, 
		NULL,
	},
};

/**********************************************************/
/* Maximums for control system */

/* this structure is filled when cmd_maximum is parsed successfully */
struct cmd_maximum_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	uint32_t in;
	uint32_t i;
	uint32_t out;
};

/* function called when cmd_maximum is parsed successfully */
static void cmd_maximum_parsed(void * parsed_result, void * data)
{
	struct cmd_maximum_result * res = parsed_result;
	
	if (!strcmp_P(res->arg1, PSTR("arm"))) {
		pid_set_maximums(&arm.pid_mot, res->in, res->i, res->out);
	}
	/* else it is a "show" */

	printf_P(PSTR("maximum arm %lu %lu %lu\r\n"), 
		 pid_get_max_in(&arm.pid_mot),
		 pid_get_max_I(&arm.pid_mot),
		 pid_get_max_out(&arm.pid_mot));
}

prog_char str_maximum_arg0[] = "maximum";
parse_pgm_token_string_t cmd_maximum_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_maximum_result, arg0, str_maximum_arg0);
prog_char str_maximum_arg1[] = "arm";
parse_pgm_token_string_t cmd_maximum_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_maximum_result, arg1, str_maximum_arg1);
parse_pgm_token_num_t cmd_maximum_in = TOKEN_NUM_INITIALIZER(struct cmd_maximum_result, in, UINT32);
parse_pgm_token_num_t cmd_maximum_i = TOKEN_NUM_INITIALIZER(struct cmd_maximum_result, i, UINT32);
parse_pgm_token_num_t cmd_maximum_out = TOKEN_NUM_INITIALIZER(struct cmd_maximum_result, out, UINT32);

prog_char help_maximum[] = "Set maximum values for PID (in, I, out)";
parse_pgm_inst_t cmd_maximum = {
	.f = cmd_maximum_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_maximum,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_maximum_arg0, 
		(prog_void *)&cmd_maximum_arg1, 
		(prog_void *)&cmd_maximum_in, 
		(prog_void *)&cmd_maximum_i, 
		(prog_void *)&cmd_maximum_out, 
		NULL,
	},
};

/* show */

prog_char str_maximum_show_arg[] = "show";
parse_pgm_token_string_t cmd_maximum_show_arg = TOKEN_STRING_INITIALIZER(struct cmd_maximum_result, arg1, str_maximum_show_arg);

prog_char help_maximum_show[] = "Show maximum values for PID";
parse_pgm_inst_t cmd_maximum_show = {
	.f = cmd_maximum_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_maximum_show,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_maximum_arg0, 
		(prog_void *)&cmd_maximum_show_arg,
		NULL,
	},
};

/**********************************************************/
/* Quadramp for control system */

/* this structure is filled when cmd_quadramp is parsed successfully */
struct cmd_quadramp_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	uint32_t ap;
	uint32_t an;
	uint32_t sp;
	uint32_t sn;
};

/* function called when cmd_quadramp is parsed successfully */
static void cmd_quadramp_parsed(void * parsed_result, void * data)
{
	struct cmd_quadramp_result * res = parsed_result;
	
	if (!strcmp_P(res->arg1, PSTR("arm"))) {
		quadramp_set_1st_order_vars(&arm.qr_mot, res->sp, res->sn);
		quadramp_set_2nd_order_vars(&arm.qr_mot, res->ap, res->an);
	}
/* 	else if (!strcmp_P(res->arg1, PSTR("distance"))) { */
/* 		quadramp_set_1st_order_vars(&arm.qr_d, res->sp, res->sn); */
/* 		quadramp_set_2nd_order_vars(&arm.qr_d, res->ap, res->an); */
/* 	} */
	/* else it's a "show" */

	printf_P(PSTR("quadramp arm %ld %ld %ld %ld\r\n"), 
		 arm.qr_mot.var_2nd_ord_pos,
		 arm.qr_mot.var_2nd_ord_neg,
		 arm.qr_mot.var_1st_ord_pos,
		 arm.qr_mot.var_1st_ord_neg);
}

prog_char str_quadramp_arg0[] = "quadramp";
parse_pgm_token_string_t cmd_quadramp_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_quadramp_result, arg0, str_quadramp_arg0);
prog_char str_quadramp_arg1[] = "arm";
parse_pgm_token_string_t cmd_quadramp_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_quadramp_result, arg1, str_quadramp_arg1);
parse_pgm_token_num_t cmd_quadramp_ap = TOKEN_NUM_INITIALIZER(struct cmd_quadramp_result, ap, UINT32);
parse_pgm_token_num_t cmd_quadramp_an = TOKEN_NUM_INITIALIZER(struct cmd_quadramp_result, an, UINT32);
parse_pgm_token_num_t cmd_quadramp_sp = TOKEN_NUM_INITIALIZER(struct cmd_quadramp_result, sp, UINT32);
parse_pgm_token_num_t cmd_quadramp_sn = TOKEN_NUM_INITIALIZER(struct cmd_quadramp_result, sn, UINT32);

prog_char help_quadramp[] = "Set quadramp values (acc+, acc-, speed+, speed-)";
parse_pgm_inst_t cmd_quadramp = {
	.f = cmd_quadramp_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_quadramp,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_quadramp_arg0, 
		(prog_void *)&cmd_quadramp_arg1, 
		(prog_void *)&cmd_quadramp_ap, 
		(prog_void *)&cmd_quadramp_an, 
		(prog_void *)&cmd_quadramp_sp, 
		(prog_void *)&cmd_quadramp_sn, 
		
		NULL,
	},
};

/* show */

prog_char str_quadramp_show_arg[] = "show";
parse_pgm_token_string_t cmd_quadramp_show_arg = TOKEN_STRING_INITIALIZER(struct cmd_quadramp_result, arg1, str_quadramp_show_arg);

prog_char help_quadramp_show[] = "Get quadramp values for control system";
parse_pgm_inst_t cmd_quadramp_show = {
	.f = cmd_quadramp_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_quadramp_show,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_quadramp_arg0, 
		(prog_void *)&cmd_quadramp_show_arg, 
		NULL,
	},
};


/**********************************************************/
/* sample ADC */
extern uint16_t sample_i;
//extern uint16_t sample_tab[MAX_SAMPLE];
/* this structure is filled when cmd_sample is parsed successfully */
struct cmd_sample_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	uint16_t offset_a;
};

extern int32_t pos_start_scan;
int32_t scan_frwd = 0;
/* function called when cmd_sample is parsed successfully */
static void cmd_sample_parsed(void * parsed_result, void * data)
{
	struct cmd_sample_result * res = parsed_result;
	uint16_t i;
	scan_frwd++;
	//int32_t cs_err, cs_out, cs_in;

	printf_P(PSTR("cmd sample called!\r\n"));
	printf_P(PSTR("arg %s %d!\r\n"), res->arg1, res->offset_a);

	offset_a = (((float)res->offset_a)*M_PI/180.);

	if (!strcmp_P(res->arg1, PSTR("start"))) {
		sample_i = MAX_SAMPLE;
		pos_start_scan = encoders_microb_get_value(SCANNER_ENC);

		memset(sample_tab, 0xff, MAX_SAMPLE*sizeof(uint8_t));
	
		
		//encoders_microb_set_value(SCANNER_ENC, 0);
		cs_set_consign(&scanner.cs_mot, pos_start_scan+SCANNER_STEP_TOUR*200L);

		last_tour_n = 0;
		last_tour_pos = 0;

		/*
		while (uart_recv_nowait(0)==-1){
			wait_ms(200);
			cs_err = cs_get_error(&scanner.cs_mot);
			cs_out = cs_get_out(&scanner.cs_mot);
			cs_in = cs_get_filtered_feedback(&scanner.cs_mot);
			printf_P(PSTR("err: %ld out: %ld in: %ld\r\n"), cs_err, cs_out, cs_in);

		}
		*/
	
	}


	

	else if (!strcmp_P(res->arg1, PSTR("dump"))) {
		printf_P(PSTR("start dumping\r\n"));
		
		for (i=0;i<MAX_SAMPLE;i++)
			printf_P(PSTR("%d %d \r\n"),sample_tab[i]&0x1ff, sample_tab[i]&0x200?1:0);
		printf_P(PSTR("end dumping  (pos: %ld)\r\n"), 
			 encoders_microb_get_value((void *)SCANNER_ENC));
	}

}

prog_char str_sample_arg0[] = "sample";
parse_pgm_token_string_t cmd_sample_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_sample_result, arg0, str_sample_arg0);
prog_char str_sample_arg1[] = "start#dump";
parse_pgm_token_string_t cmd_sample_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_sample_result, arg1, str_sample_arg1);
parse_pgm_token_num_t cmd_sample_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_sample_result, offset_a, UINT16);

prog_char help_sample[] = "Sample func";
parse_pgm_inst_t cmd_sample = {
	.f = cmd_sample_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_sample,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_sample_arg0, 
		(prog_void *)&cmd_sample_arg1, 
		(prog_void *)&cmd_sample_arg2, 
		NULL,
	},
};


/**********************************************************/


/* in progmem */
parse_pgm_ctx_t main_ctx[] = {
	(parse_pgm_inst_t *)&cmd_reset,
	(parse_pgm_inst_t *)&cmd_spi_test,
	(parse_pgm_inst_t *)&cmd_bootloader,
	(parse_pgm_inst_t *)&cmd_ax12_stress,
	(parse_pgm_inst_t *)&cmd_armxy,
	(parse_pgm_inst_t *)&cmd_arm_circ,
	(parse_pgm_inst_t *)&cmd_arm_harv,
	(parse_pgm_inst_t *)&cmd_test,
	(parse_pgm_inst_t *)&cmd_arm_straight,
	(parse_pgm_inst_t *)&cmd_baudrate,
	(parse_pgm_inst_t *)&cmd_arm_goto,
	(parse_pgm_inst_t *)&cmd_arm_capture,
	(parse_pgm_inst_t *)&cmd_sample,
	(parse_pgm_inst_t *)&cmd_uint16_read,
	(parse_pgm_inst_t *)&cmd_uint16_write,
	(parse_pgm_inst_t *)&cmd_uint8_read,
	(parse_pgm_inst_t *)&cmd_uint8_write,
	(parse_pgm_inst_t *)&cmd_encoders,
	(parse_pgm_inst_t *)&cmd_pwm,
	(parse_pgm_inst_t *)&cmd_gain,
	(parse_pgm_inst_t *)&cmd_gain_show,
	(parse_pgm_inst_t *)&cmd_speed,
	(parse_pgm_inst_t *)&cmd_speed_show,
	(parse_pgm_inst_t *)&cmd_pos,
	(parse_pgm_inst_t *)&cmd_event,
	(parse_pgm_inst_t *)&cmd_maximum,
	(parse_pgm_inst_t *)&cmd_maximum_show,
	(parse_pgm_inst_t *)&cmd_quadramp,
	(parse_pgm_inst_t *)&cmd_quadramp_show,

	NULL,
};
