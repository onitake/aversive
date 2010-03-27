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
 *  Revision : $Id: main.c,v 1.6 2009-11-08 17:25:00 zer0 Exp $
 *
 */

#include <stdio.h>
#include <string.h>
#include <avr/eeprom.h>

#include <aversive.h>
#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>

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
#include <blocking_detection_manager.h>

#include <parse.h>
#include <rdline.h>

#include "../common/eeprom_mapping.h"
#include "../common/i2c_commands.h"

#include "main.h"
#include "ax12_user.h"
#include "cmdline.h"
#include "sensor.h"
#include "state.h"
#include "actuator.h"
#include "spickle.h"
#include "shovel.h"
#include "cs.h"
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
struct cobboard cobboard;

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

void do_led_blink(__attribute__((unused)) void *dummy)
{
#if 1 /* simple blink */
	static uint8_t a=0;

	if(a)
		LED1_ON();
	else
		LED1_OFF();
	
	a = !a;
#endif
}

static void main_timer_interrupt(void)
{
	static uint8_t cpt = 0;
	cpt++;
	sei();
	if ((cpt & 0x3) == 0)
		scheduler_interrupt();
}

int main(void)
{
	/* brake */
	BRAKE_ON();
	BRAKE_DDR();

	/* CPLD reset on PG3 */
	DDRG |= 1<<3;
	PORTG &= ~(1<<3); /* implicit */

	/* LEDS */
	DDRJ |= 0x0c;
	DDRL = 0xc0;
	LED1_OFF();
	memset(&gen, 0, sizeof(gen));
	memset(&cobboard, 0, sizeof(cobboard));
	/* cs is enabled after arm_calibrate() */
	cobboard.flags = DO_ENCODERS | DO_POWER; // DO_BD

	/* UART */
	uart_init();
#if CMDLINE_UART == 3
 	fdevopen(uart3_dev_send, uart3_dev_recv);
	uart_register_rx_event(3, emergency);
#elif CMDLINE_UART == 1
 	fdevopen(uart1_dev_send, uart1_dev_recv);
	uart_register_rx_event(1, emergency);
#else
#  error not supported
#endif

	//eeprom_write_byte(EEPROM_MAGIC_ADDRESS, EEPROM_MAGIC_COBBOARD);
	/* check eeprom to avoid to run the bad program */
	if (eeprom_read_byte(EEPROM_MAGIC_ADDRESS) !=
	    EEPROM_MAGIC_COBBOARD) {
		sei();
		printf_P(PSTR("Bad eeprom value\r\n"));
		while(1);
	}

	/* LOGS */
	error_register_emerg(mylog);
	error_register_error(mylog);
	error_register_warning(mylog);
	error_register_notice(mylog);
	error_register_debug(mylog);

	/* SPI + ENCODERS */
	encoders_spi_init(); /* this will also init spi hardware */

	/* I2C */
	i2c_protocol_init();
	i2c_init(I2C_MODE_SLAVE, I2C_COBBOARD_ADDR);
	i2c_register_recv_event(i2c_recvevent);

	/* TIMER */
	timer_init();
	timer0_register_OV_intr(main_timer_interrupt);

	/* PWM */
	PWM_NG_TIMER_16BITS_INIT(1, TIMER_16_MODE_PWM_10, 
				 TIMER1_PRESCALER_DIV_1);
	PWM_NG_TIMER_16BITS_INIT(4, TIMER_16_MODE_PWM_10, 
				 TIMER4_PRESCALER_DIV_1);
	
	PWM_NG_INIT16(&gen.pwm1_4A, 4, A, 10, PWM_NG_MODE_SIGNED |
		      PWM_NG_MODE_SIGN_INVERTED, &PORTD, 4);
	PWM_NG_INIT16(&gen.pwm2_4B, 4, B, 10, PWM_NG_MODE_SIGNED,
		      &PORTD, 5);
	PWM_NG_INIT16(&gen.pwm3_1A, 1, A, 10, PWM_NG_MODE_SIGNED,
		      &PORTD, 6);
	PWM_NG_INIT16(&gen.pwm4_1B, 1, B, 10, PWM_NG_MODE_SIGNED |
		      PWM_NG_MODE_SIGN_INVERTED,
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
	
	/* SCHEDULER */
	scheduler_init();

	scheduler_add_periodical_event_priority(do_led_blink, NULL, 
						100000L / SCHEDULER_UNIT, 
						LED_PRIO);
	/* all cs management */
	microb_cs_init();

	/* sensors, will also init hardware adc */
	sensor_init();

	/* TIME */
	time_init(TIME_PRIO);

	/* ax12 */
	ax12_user_init();

	sei();

	/* actuators */
	actuator_init();

	/* spickle, shovel */
	spickle_init();
	shovel_init();

/* 	state_init(); */

	printf_P(PSTR("\r\n"));
	printf_P(PSTR("Dass das Gluck deinen Haus setzt.\r\n"));

	/* arm management */
 	gen.logs[0] = E_USER_ST_MACH;
 	gen.log_level = 5;
	cobboard.flags |= DO_CS;

/* 	state_machine(); */
	cmdline_interact();

	return 0;
}
