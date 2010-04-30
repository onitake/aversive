/*
 *  Copyright Droids Corporation
 *  Olivier Matz <zer0@droids-corp.org>
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
 *  Revision : $Id: main.c,v 1.10 2009-11-08 17:24:33 zer0 Exp $
 *
 */

#include <stdio.h>
#include <string.h>

#include <aversive.h>
#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>
#include <aversive/eeprom.h>

#include <ax12.h>
#include <uart.h>
#include <spi.h>
#include <i2c.h>
#include <encoders_spi.h>
#include <pwm_ng.h>
#include <timer.h>
#include <scheduler.h>
#include <clock_time.h>
#include <adc.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <trajectory_manager.h>
#include <vect_base.h>
#include <lines.h>
#include <polygon.h>
#include <obstacle_avoidance.h>
#include <blocking_detection_manager.h>
#include <robot_system.h>
#include <position_manager.h>

#include <parse.h>
#include <rdline.h>

#include "../common/eeprom_mapping.h"
#include "../common/i2c_commands.h"

#include "main.h"
#include "robotsim.h"
#include "ax12_user.h"
#include "strat.h"
#include "cmdline.h"
#include "sensor.h"
#include "actuator.h"
#include "cs.h"
#include "strat_base.h"
#include "strat_db.h"
#include "i2c_protocol.h"

/* 0 means "programmed"
 * ---- with 16 Mhz quartz
 * CKSEL 3-0 : 0111
 * SUT 1-0 : 10
 * CKDIV8 : 1
 * ---- bootloader
 * BOOTZ 1-0 : 01 (4K bootloader)
 * BOOTRST : 0 (reset on bootloader)
 * ---- jtag
 * jtagen : 0
 */

struct genboard gen;
struct mainboard mainboard;
volatile struct cobboard cobboard;
volatile struct ballboard ballboard;

#ifndef HOST_VERSION
/***********************/

void bootloader(void)
{
#define BOOTLOADER_ADDR 0x3f000
	if (pgm_read_byte_far(BOOTLOADER_ADDR) == 0xff) {
		printf_P(PSTR("Bootloader is not present\r\n"));
		return;
	}
	cli();
	BRAKE_ON();
	/* ... very specific :( */
	TIMSK0 = 0;
	TIMSK1 = 0;
	TIMSK2 = 0;
	TIMSK3 = 0;
	TIMSK4 = 0;
	TIMSK5 = 0;
	EIMSK = 0;
	UCSR0B = 0;
	UCSR1B = 0;
	UCSR2B = 0;
	UCSR3B = 0;
	SPCR = 0;
	TWCR = 0;
	ACSR = 0;
	ADCSRA = 0;

	EIND = 1;
	__asm__ __volatile__ ("ldi r31,0xf8\n");
	__asm__ __volatile__ ("ldi r30,0x00\n");
	__asm__ __volatile__ ("eijmp\n");

	/* never returns */
}

void do_time_monitor(void *dummy)
{
	uint16_t seconds;
	seconds = eeprom_read_word(EEPROM_TIME_ADDRESS);
	seconds ++;
	eeprom_write_word(EEPROM_TIME_ADDRESS, seconds);
}

void do_led_blink(void *dummy)
{
	static uint8_t a = 0;

	if (mainboard.flags & DO_ERRBLOCKING) {
		if (a & 1)
			LED1_ON();
		else
			LED1_OFF();
	}
	else {
		if (a & 4)
			LED1_ON();
		else
			LED1_OFF();
	}
	a++;
}

static void main_timer_interrupt(void)
{
	static uint8_t cpt = 0;
	cpt++;
	sei();
	if ((cpt & 0x3) == 0)
		scheduler_interrupt();
}
#endif

int main(void)
{
#ifndef HOST_VERSION
	/* brake */
	BRAKE_DDR();
	BRAKE_OFF();

	/* CPLD reset on PG3 */
	DDRG |= 1<<3;
	PORTG &= ~(1<<3); /* implicit */

	/* LEDS */
	DDRJ |= 0x0c;
	DDRL = 0xc0;
	LED1_OFF();
	LED2_OFF();
	LED3_OFF();
	LED4_OFF();
#endif

	memset(&gen, 0, sizeof(gen));
	memset(&mainboard, 0, sizeof(mainboard));
	mainboard.flags = DO_ENCODERS | DO_CS | DO_RS |
		DO_POS | DO_POWER | DO_BD | DO_ERRBLOCKING;
	ballboard.lcob = I2C_COB_NONE;
	ballboard.rcob = I2C_COB_NONE;

	/* UART */
	uart_init();
	uart_register_rx_event(CMDLINE_UART, emergency);
#ifndef HOST_VERSION
#if CMDLINE_UART == 3
 	fdevopen(uart3_dev_send, uart3_dev_recv);
#elif CMDLINE_UART == 1
 	fdevopen(uart1_dev_send, uart1_dev_recv);
#endif

	/* check eeprom to avoid to run the bad program */
	if (eeprom_read_byte(EEPROM_MAGIC_ADDRESS) !=
	    EEPROM_MAGIC_MAINBOARD) {
		int c;
		sei();
		printf_P(PSTR("Bad eeprom value ('f' to force)\r\n"));
		c = uart_recv(CMDLINE_UART);
		if (c == 'f')
			eeprom_write_byte(EEPROM_MAGIC_ADDRESS, EEPROM_MAGIC_MAINBOARD);
		wait_ms(100);
		bootloader();
	}
#endif /* ! HOST_VERSION */

	/* LOGS */
	error_register_emerg(mylog);
	error_register_error(mylog);
	error_register_warning(mylog);
	error_register_notice(mylog);
	error_register_debug(mylog);

#ifndef HOST_VERSION
	/* SPI + ENCODERS */
	encoders_spi_init(); /* this will also init spi hardware */

	/* I2C */
	i2c_init(I2C_MODE_MASTER, I2C_MAINBOARD_ADDR);
	i2c_protocol_init();
	i2c_register_recv_event(i2c_recvevent);
	i2c_register_send_event(i2c_sendevent);

	/* TIMER */
	timer_init();
	timer0_register_OV_intr(main_timer_interrupt);

	/* PWM */
	PWM_NG_TIMER_16BITS_INIT(1, TIMER_16_MODE_PWM_10,
				 TIMER1_PRESCALER_DIV_1);
	PWM_NG_TIMER_16BITS_INIT(4, TIMER_16_MODE_PWM_10,
				 TIMER4_PRESCALER_DIV_1);

	PWM_NG_INIT16(&gen.pwm1_4A, 4, A, 10, PWM_NG_MODE_SIGNED,
		      &PORTD, 4);
	PWM_NG_INIT16(&gen.pwm2_4B, 4, B, 10, PWM_NG_MODE_SIGNED |
		      PWM_NG_MODE_SIGN_INVERTED, &PORTD, 5);
	PWM_NG_INIT16(&gen.pwm3_1A, 1, A, 10, PWM_NG_MODE_SIGNED,
		      &PORTD, 6);
	PWM_NG_INIT16(&gen.pwm4_1B, 1, B, 10, PWM_NG_MODE_SIGNED,
		      &PORTD, 7);


	/* servos */
	PWM_NG_TIMER_16BITS_INIT(3, TIMER_16_MODE_PWM_10,
				 TIMER1_PRESCALER_DIV_256);
	PWM_NG_INIT16(&gen.servo1, 3, C, 10, PWM_NG_MODE_NORMAL,
		      NULL, 0);
	PWM_NG_TIMER_16BITS_INIT(5, TIMER_16_MODE_PWM_10,
				 TIMER1_PRESCALER_DIV_256);
	PWM_NG_INIT16(&gen.servo2, 5, A, 10, PWM_NG_MODE_NORMAL,
		      NULL, 0);
	PWM_NG_INIT16(&gen.servo3, 5, B, 10, PWM_NG_MODE_NORMAL,
		      NULL, 0);
	PWM_NG_INIT16(&gen.servo4, 5, C, 10, PWM_NG_MODE_NORMAL,
		      NULL, 0);
	support_balls_deploy(); /* init pwm for servos */
#endif /* !HOST_VERSION */

	/* SCHEDULER */
	scheduler_init();
#ifdef HOST_VERSION
	hostsim_init();
	robotsim_init();
#endif

#ifndef HOST_VERSION
	scheduler_add_periodical_event_priority(do_led_blink, NULL,
						100000L / SCHEDULER_UNIT,
						LED_PRIO);
#endif /* !HOST_VERSION */

	/* all cs management */
	microb_cs_init();

	/* TIME */
	time_init(TIME_PRIO);

#ifndef HOST_VERSION
	/* sensors, will also init hardware adc */
	sensor_init();

	/* start i2c slave polling */
	scheduler_add_periodical_event_priority(i2c_poll_slaves, NULL,
						8000L / SCHEDULER_UNIT, I2C_POLL_PRIO);
#endif /* !HOST_VERSION */

	/* strat */
 	gen.logs[0] = E_USER_STRAT;
 	gen.log_level = 5;
	strat_db_init();

	/* strat-related event */
	scheduler_add_periodical_event_priority(strat_event, NULL,
						25000L / SCHEDULER_UNIT,
						STRAT_PRIO);

#ifndef HOST_VERSION
	/* eeprom time monitor */
	scheduler_add_periodical_event_priority(do_time_monitor, NULL,
						1000000L / SCHEDULER_UNIT,
						EEPROM_TIME_PRIO);
#endif /* !HOST_VERSION */

	sei();

	printf_P(PSTR("\r\n"));
	printf_P(PSTR("Respect et robustesse.\r\n"));
#ifndef HOST_VERSION
	{
		uint16_t seconds;
		seconds = eeprom_read_word(EEPROM_TIME_ADDRESS);
		printf_P(PSTR("Running since %d mn %d\r\n"), seconds/60, seconds%60);
	}
#endif

#ifdef HOST_VERSION
	strat_reset_pos(400, COLOR_Y(400), COLOR_A(-90));
#endif

	cmdline_interact();

	return 0;
}
