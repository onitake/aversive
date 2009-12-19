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
 *  Revision : $Id: commands.c,v 1.1 2009-01-30 20:42:17 zer0 Exp $
 *
 *  Olivier MATZ <zer0@droids-corp.org> 
 */


#include <stdio.h>
#include <string.h>

#include <aversive/pgmspace.h>
#include <aversive/wait.h>

#include <parse.h>
#include <parse_num.h>
#include <parse_string.h>
#include <uart.h>
#include <spi.h>

#include "main.h"


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
#define BOOTLOADER_ADDR 0x1e000
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

	__asm__ __volatile__ ("ldi r30,0x00\n");
	__asm__ __volatile__ ("ldi r31,0xf0\n");
	__asm__ __volatile__ ("ijmp\n");
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

#ifdef notyet
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
#endif

/**********************************************************/


/* in progmem */
parse_pgm_ctx_t main_ctx[] = {
	(parse_pgm_inst_t *)&cmd_reset,
	(parse_pgm_inst_t *)&cmd_spi_test,
	(parse_pgm_inst_t *)&cmd_bootloader,
#ifdef notyet
	(parse_pgm_inst_t *)&cmd_encoders,
#endif

	NULL,
};
