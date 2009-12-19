/*
 *  Copyright Droids Corporation (2009)
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
 *  Revision : $Id: commands_scan.c,v 1.1 2009-05-27 20:04:07 zer0 Exp $
 *
 *  Olivier MATZ <zer0@droids-corp.org> 
 */

#include <stdio.h>
#include <string.h>

#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>
#include <time.h>

#include <ax12.h>
#include <uart.h>
#include <pwm_ng.h>
#include <time.h>
#include <spi.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <blocking_detection_manager.h>

#include <rdline.h>
#include <parse.h>
#include <parse_string.h>
#include <parse_num.h>

#include <adc.h>

#include <math.h>

#include "main.h"
#include "cmdline.h"
#include "../common/i2c_commands.h"
#include "i2c_protocol.h"
#include "actuator.h"

#include <vect_base.h>
#include <lines.h>
#include <polygon.h>

#include "img_processing.h"
#include "scanner.h"



/**********************************************************/
/* sample ADC */
/*
extern uint16_t sample_i;
extern float scan_offset_a;
extern float scan_offset_b;
*/
extern struct scan_params scan_params;

//extern uint16_t sample_tab[MAX_SAMPLE];
/* this structure is filled when cmd_sample is parsed successfully */
struct cmd_sample_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	uint16_t offset_b;
	float offset_a;
	uint16_t dump_speed;
	uint8_t filter;
};

extern int32_t pos_start_scan;
/* function called when cmd_sample is parsed successfully */

#define MAX_OBJECTS 10
Object_bb sac_obj[MAX_OBJECTS];
static void cmd_sample_parsed(void * parsed_result, void * data)
{
	struct cmd_sample_result * res = parsed_result;
	uint16_t i;

	printf_P(PSTR("cmd sample called\r\n"));
	printf_P(PSTR("arg %s %d\r\n"), res->arg1, res->offset_b);

	quadramp_set_1st_order_vars(&sensorboard.scanner.qr, res->dump_speed, res->dump_speed); /* set speed */


	scan_params.offset_b = (((float)res->offset_b)*M_PI/180.);
	scan_params.offset_a = (((float)res->offset_a)*M_PI/180.);
	scan_params.filter = res->filter;

	if (!strcmp_P(res->arg1, PSTR("start"))) {
		scan_params.sample_i = MAX_SAMPLE;
		scan_params.pos_start_scan = encoders_spi_get_value_scanner(SCANNER_ENC);
		//printf_P(PSTR("start scan at pos %ld\r\n"), scan_params.pos_start_scan);

		memset(scan_params.sample_tab, 0xff, MAX_SAMPLE*sizeof(uint8_t));
	
		
		cs_set_consign(&sensorboard.scanner.cs, scan_params.pos_start_scan+SCANNER_STEP_TOUR*200L);
		//printf_P(PSTR("scan dst %ld\r\n"), scan_params.pos_start_scan+SCANNER_STEP_TOUR*200L);

		scan_params.last_col_n = 0;
		scan_params.last_row_n = 0;
		scan_params.last_sample = 0;
	
	}
	else if (!strcmp_P(res->arg1, PSTR("dump"))) {
		printf_P(PSTR("start object detection\r\n"));
		for (i=0;i<MAX_OBJECTS;i++){
			sac_obj[i].x_min = 0;
			sac_obj[i].x_max = 0;
			sac_obj[i].y_min = 0;
			sac_obj[i].y_max = 0;
		}

		//parcour_img(sample_tab, PIX_PER_SCAN, MAX_SAMPLE/PIX_PER_SCAN, sac_obj, MAX_OBJECTS);
		/*
		process_img(scan_params.sample_tab, PIX_PER_SCAN, MAX_SAMPLE/PIX_PER_SCAN,
			    4, 1,
			    0, 15, 15, 
			    1);
		*/

		for (i=0;i<MAX_OBJECTS;i++){
			printf_P(PSTR("obj: %d %d %d %d\r\n"), 
				 sac_obj[i].x_min, 
				 sac_obj[i].y_min, 
				 sac_obj[i].x_max, 
				 sac_obj[i].y_max);
		}

		printf_P(PSTR("start dumping %ld\r\n"), PIX_PER_SCAN);
		
		for (i=0;i<MAX_SAMPLE;i++)
			printf_P(PSTR("%d\r\n"),scan_params.sample_tab[i]);

		printf_P(PSTR("end dumping  (pos: %ld)\r\n"), 
			 encoders_spi_get_value_scanner((void *)SCANNER_ENC));
	}

}

prog_char str_sample_arg0[] = "sample";
parse_pgm_token_string_t cmd_sample_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_sample_result, arg0, str_sample_arg0);
prog_char str_sample_arg1[] = "start#dump";
parse_pgm_token_string_t cmd_sample_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_sample_result, arg1, str_sample_arg1);
parse_pgm_token_num_t cmd_sample_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_sample_result, offset_b, UINT16);
parse_pgm_token_num_t cmd_sample_arg3 = TOKEN_NUM_INITIALIZER(struct cmd_sample_result, offset_a, FLOAT);
parse_pgm_token_num_t cmd_sample_arg4 = TOKEN_NUM_INITIALIZER(struct cmd_sample_result, dump_speed, UINT16);
parse_pgm_token_num_t cmd_sample_arg5 = TOKEN_NUM_INITIALIZER(struct cmd_sample_result, filter, UINT8);

prog_char help_sample[] = "Sample func off_a_mot, off_a_angl, dump_speed";
parse_pgm_inst_t cmd_sample = {
	.f = cmd_sample_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_sample,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_sample_arg0, 
		(prog_void *)&cmd_sample_arg1, 
		(prog_void *)&cmd_sample_arg2, 
		(prog_void *)&cmd_sample_arg3, 
		(prog_void *)&cmd_sample_arg4, 
		(prog_void *)&cmd_sample_arg5, 
		NULL,
	},
};




/**********************************************************/
/* sadc tests */

/* this structure is filled when cmd_sadc is parsed successfully */
struct cmd_sadc_result {
	fixed_string_t arg0;
};

/* function called when cmd_sadc is parsed successfully */
static void cmd_sadc_parsed(void *parsed_result, void *data)
{

	printf_P(PSTR("Scan ADC values:\r\n"));
	do {
	  printf_P(PSTR("%.4d "), adc_get_value( ADC_REF_AVCC | MUX_ADC13 ));
	  printf_P(PSTR("\r\n"));
	  wait_ms(100);
	} while (!cmdline_keypressed());
}

prog_char str_sadc_arg0[] = "sadc";
parse_pgm_token_string_t cmd_sadc_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_sadc_result, arg0, str_sadc_arg0);

prog_char help_sadc[] = "Show sadc values";
parse_pgm_inst_t cmd_sadc = {
	.f = cmd_sadc_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_sadc,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_sadc_arg0, 
		NULL,
	},
};




/**********************************************************/
/* set scanner params */

/* this structure is filled when cmd_scan_params is parsed successfully */
struct cmd_scan_params_result {
	fixed_string_t arg0;
	int speed;
	uint8_t debug;
};

/* function called when cmd_scan_params is parsed successfully */
static void cmd_scan_params_parsed(void * parsed_result, void * data)
{
	struct cmd_scan_params_result * res = parsed_result;
	
	scan_params.speed = res->speed;
	scan_params.debug = res->debug;

}

prog_char str_scan_params_arg0[] = "scan_params";
parse_pgm_token_string_t cmd_scan_params_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_scan_params_result, arg0, str_scan_params_arg0);
parse_pgm_token_num_t cmd_scan_params_arg1 = TOKEN_NUM_INITIALIZER(struct cmd_scan_params_result, speed, INT16);
parse_pgm_token_num_t cmd_scan_params_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_scan_params_result, debug, UINT8);

prog_char help_scan_params[] = "Set scanner params (speed, debug)";
parse_pgm_inst_t cmd_scan_params = {
	.f = cmd_scan_params_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_sample,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_scan_params_arg0,
		(prog_void *)&cmd_scan_params_arg1,
		(prog_void *)&cmd_scan_params_arg2,
		NULL,
	},
};



/**********************************************************/
/* set scanner calibration */

/* this structure is filled when cmd_scan_calibre is parsed successfully */
struct cmd_scan_calibre_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_scan_calibre is parsed successfully */
static void cmd_scan_calibre_parsed(void * parsed_result, void * data)
{
	struct cmd_scan_calibre_result * res = parsed_result;

	printf_P(PSTR("Starting scanner autocalibration\r\n"));
	  

	if (!strcmp_P(res->arg1, PSTR("mirror"))) {
		scanner_calibre_mirror();
	}
	else{
		scanner_calibre_laser();
	}
}

prog_char str_scan_calibre_arg0[] = "scan_calibre";

prog_char str_scan_calibre_what_arg1[] = "mirror#laser";
parse_pgm_token_string_t cmd_scan_calibre_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_scan_calibre_result, arg0, str_scan_calibre_arg0);
parse_pgm_token_string_t cmd_scan_calibre_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_scan_calibre_result, arg1, str_scan_calibre_what_arg1);


prog_char help_scan_calibre[] = "Scanner auto calibration (mirror|laser)";
parse_pgm_inst_t cmd_scan_calibre = {
	.f = cmd_scan_calibre_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_scan_calibre,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_scan_calibre_arg0, 
		(prog_void *)&cmd_scan_calibre_arg1, 
		NULL,
	},
};



/**********************************************************/
/* start scan */

/* this structure is filled when cmd_scan_do is parsed successfully */
struct cmd_scan_do_result {
	fixed_string_t arg0;
};

/* function called when cmd_scan_do is parsed successfully */
static void cmd_scan_do_parsed(void * parsed_result, void * data)
{
	  printf_P(PSTR("Starting scan\r\n"));
	  scanner_scan_autonomous();	
}

prog_char str_scan_do_arg0[] = "scan_do";
parse_pgm_token_string_t cmd_scan_do_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_scan_do_result, arg0, str_scan_do_arg0);

prog_char help_scan_do[] = "Scan zone";
parse_pgm_inst_t cmd_scan_do = {
	.f = cmd_scan_do_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_scan_do,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_scan_do_arg0, 
		NULL,
	},
};




/**********************************************************/
/* set scanner img params */

/* this structure is filled when cmd_scan_img is parsed successfully */
struct cmd_scan_img_result {
	fixed_string_t arg0;
	uint8_t algo;
	uint8_t h;
	int16_t x;
	int16_t y;

};

/* function called when cmd_scan_img is parsed successfully */
static void cmd_scan_img_parsed(void * parsed_result, void * data)
{
	struct cmd_scan_img_result * res = parsed_result;

	scan_params.algo = res->algo;
	if (res->algo == I2C_SCANNER_ALGO_COLUMN_DROPZONE) {
		scan_params.drop_zone.working_zone = res->h;
		scan_params.drop_zone.center_x = res->x;
		scan_params.drop_zone.center_y = res->y;
	} else if (res->algo == I2C_SCANNER_ALGO_CHECK_TEMPLE) {
		scan_params.check_temple.level = res->h;
		scan_params.check_temple.temple_x = res->x;
		scan_params.check_temple.temple_y = res->y;
	} else 	if (res->algo == I2C_SCANNER_ALGO_TEMPLE_DROPZONE) {
		scan_params.drop_zone.working_zone = res->h;
		scan_params.drop_zone.center_x = res->x;
		scan_params.drop_zone.center_y = res->y;
	} 



}

prog_char str_scan_img_arg0[] = "scan_img";
parse_pgm_token_string_t cmd_scan_img_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_scan_img_result, arg0, str_scan_img_arg0);
parse_pgm_token_num_t cmd_scan_img_arg1 = TOKEN_NUM_INITIALIZER(struct cmd_scan_img_result, algo,  UINT8);
parse_pgm_token_num_t cmd_scan_img_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_scan_img_result, h,  UINT8);
parse_pgm_token_num_t cmd_scan_img_arg3 = TOKEN_NUM_INITIALIZER(struct cmd_scan_img_result, x, INT16);
parse_pgm_token_num_t cmd_scan_img_arg4 = TOKEN_NUM_INITIALIZER(struct cmd_scan_img_result, y, INT16);


prog_char help_scan_img[] = "Set scanner img processing params (algo, H, x, y)";
parse_pgm_inst_t cmd_scan_img = {
	.f = cmd_scan_img_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_scan_img,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_scan_img_arg0,
		(prog_void *)&cmd_scan_img_arg1,
		(prog_void *)&cmd_scan_img_arg2,
		(prog_void *)&cmd_scan_img_arg3,
		(prog_void *)&cmd_scan_img_arg4,
		NULL,
	},
};


