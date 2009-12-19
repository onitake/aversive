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
 *  Revision : $Id: main.c,v 1.8 2009-05-02 10:08:09 zer0 Exp $
 *
 */

#include <stdio.h>
#include <string.h>

#include <aversive.h>
#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>
#include <avr/eeprom.h>

#include <uart.h>
#include <pwm_ng.h>
#include <timer.h>
#include <scheduler.h>
#include <time.h>
#include <adc.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <trajectory_manager.h>
#include <blocking_detection_manager.h>
#include <robot_system.h>
#include <position_manager.h>
#include <encoders_microb.h>

#include <parse.h>
#include <rdline.h>

#include "main.h"
#include "cmdline.h"
#include "sensor.h"
#include "actuator.h"
#include "cs.h"

#if __AVR_LIBC_VERSION__ == 10602UL
//#error "won't work with this version"
#endif

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

void do_led_blink(void *dummy)
{
#if 1 /* simple blink */
	LED1_TOGGLE();
#endif
}

static void main_timer_interrupt(void)
{
	static uint8_t cpt = 0;
	static uint8_t encoder_running = 0;

	cpt++;

	/* BAAAAd */
	if (encoder_running)
		return;

	encoder_running = 1;
	sei();

	encoders_microb_manage(NULL);
	encoder_running = 0;

	if ((cpt & 0x3) == 0)
		scheduler_interrupt();
}

int main(void)
{
	/* LEDs */
	DDRG = 0x18;
	DDRB = 0x10;

	/* brake */
	BRAKE_DDR();
	BRAKE_OFF();

	LED1_ON();
	LED2_ON();
	LED3_ON();

	memset(&gen, 0, sizeof(gen));
	memset(&mainboard, 0, sizeof(mainboard));
	mainboard.flags = DO_ENCODERS | DO_RS |
		DO_POS | DO_POWER | DO_BD;

	/* UART */
	uart_init();
#if CMDLINE_UART == 0
 	fdevopen(uart0_dev_send, uart0_dev_recv);
	uart_register_rx_event(0, emergency);
#elif CMDLINE_UART == 1
 	fdevopen(uart1_dev_send, uart1_dev_recv);
	uart_register_rx_event(1, emergency);
#else
#  error not supported
#endif

	/* LOGS */
	error_register_emerg(mylog);
	error_register_error(mylog);
	error_register_warning(mylog);
	error_register_notice(mylog);
	error_register_debug(mylog);

	encoders_microb_init(); 

	/* TIMER */
	timer_init();
	timer0_register_OV_intr(main_timer_interrupt);

	/* PWM */
	PWM_NG_TIMER_16BITS_INIT(1, TIMER_16_MODE_PWM_10, 
				 TIMER1_PRESCALER_DIV_1);
	PWM_NG_INIT16(&gen.pwm1_1A, 1, A, 10, PWM_NG_MODE_SIGNED,
		      &PORTG, 0);
	PWM_NG_INIT16(&gen.pwm2_1B, 1, B, 10, PWM_NG_MODE_SIGNED,
		      &PORTG, 1);


	/* servos */
	PWM_NG_TIMER_16BITS_INIT(3, TIMER_16_MODE_PWM_10, 
				 TIMER1_PRESCALER_DIV_1);
	PWM_NG_INIT16(&gen.pwm3_3A, 3, A, 10, PWM_NG_MODE_SIGNED,
		      &PORTG, 2);
	PWM_NG_INIT16(&gen.pwm4_3B, 3, B, 10, PWM_NG_MODE_SIGNED | PWM_NG_MODE_SIGN_INVERTED,
		      &PORTD, 7);

	/* SCHEDULER */
	scheduler_init();

	scheduler_add_periodical_event_priority(do_led_blink, NULL, 
						100000L / SCHEDULER_UNIT, 
						LED_PRIO);
	/* all cs management */
	microb_cs_init();

	/* sensors, will also init hardware adc */
	sensor_init();
	fessor_init();
	elevator_init();

	/* TIME */
	time_init(TIME_PRIO);

	/* strat */
 	gen.logs[0] = E_USER_STRAT;
 	gen.log_level = 5;

	sei();

	printf_P(PSTR("\r\n"));
	printf_P(PSTR("Respekt Robustheit!\r\n"));
	cmdline_interact();

	return 0;
}
