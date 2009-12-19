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
 *  Revision : $Id: main.c,v 1.6 2009-03-15 20:08:51 zer0 Exp $
 *
 */

/*
 * Cmdline interface for AX12. Use the PC to command a daisy-chain of
 * AX12 actuators with a nice command line interface.
 * 
 * The circuit should be as following:
 *
 *    |----------|
 *    |	    uart0|------->--- PC (baudrate=57600)
 *    |		 |-------<---
 *    |	atmega128|
 *    |		 |
 *    |	    uart1|---->---+-- AX12 (baudrate 1 000 000)
 *    |		 |----<---| 
 *    |----------|
 *
 * Note that RX and TX pins of UART1 are connected together to provide
 * a half-duplex UART emulation.
 *
 */

#include <stdio.h>
#include <string.h>

#include <aversive.h>
#include <aversive/pgmspace.h>
#include <aversive/wait.h>

#include <uart.h>
#include <i2c.h>
#include <ax12.h>
#include <parse.h>
#include <rdline.h>
#include <pwm_ng.h>
#include <encoders_microb.h>
#include <timer.h>
#include <scheduler.h>
#include <pid.h>
#include <time.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <adc.h>
#include <spi.h>

#include "main.h"

#include "arm_xy.h"

/* for cmdline interface */
struct rdline rdl;
char prompt[RDLINE_PROMPT_SIZE];
extern parse_pgm_ctx_t main_ctx[];

/* structure defining the AX12 servo */
AX12 ax12;
struct arm arm;

struct arm scanner;

/* for storing mesures*/
uint8_t sample_tab[MAX_SAMPLE];
uint16_t sample_i = 0;

/******** For cmdline. See in commands.c for the list of commands. */
static void write_char(char c) 
{
	uart_send(0, c);
}

static void 
valid_buffer(const char * buf, uint8_t size) 
{
	int8_t ret;
	ret = parse(main_ctx, buf);
	if (ret == PARSE_AMBIGUOUS)
		printf_P(PSTR("Ambiguous command\r\n"));
	else if (ret == PARSE_NOMATCH)
		printf_P(PSTR("Command not found\r\n"));
	else if (ret == PARSE_BAD_ARGS)
		printf_P(PSTR("Bad arguments\r\n"));
}

static int8_t 
complete_buffer(const char * buf, char * dstbuf, uint8_t dstsize,
		int16_t * state)
{
	return complete(main_ctx, buf, state, dstbuf, dstsize);
}

/********************************* AX12 commands */

/*
 * --- use uart1 
 *
 * We use synchronous access (not interrupt driven) to the hardware
 * UART, because we have to be sure that the transmission/reception is
 * really finished when we return from the functions.
 *
 * We don't use the CM-5 circuit as described in the AX12
 * documentation, we simply connect TX and RX and use TXEN + RXEN +
 * DDR to manage the port directions.
 */

static volatile uint8_t ax12_state = AX12_STATE_READ;
extern volatile struct cirbuf g_tx_fifo[]; /* uart fifo */
static volatile uint8_t ax12_nsent = 0;

/* Called by ax12 module to send a character on serial line. Count the
 * number of transmitted bytes. It will be used in ax12_recv_char() to
 * drop the bytes that we transmitted. */
static int8_t ax12_send_char(uint8_t c)
{
	uart_send(1, c);
	ax12_nsent++;
	return 0;
}

/* for atmega256 */
#ifndef TXEN
#define TXEN TXEN0
#endif

/* called by uart module when the character has been written in
 * UDR. It does not mean that the byte is physically transmitted. */
static void ax12_send_callback(char c)
{
	if (ax12_state == AX12_STATE_READ) {
		/* disable TX when last byte is pushed. */
		if (CIRBUF_IS_EMPTY(&g_tx_fifo[1]))
			UCSR1B &= ~(1<<TXEN);
	}
}

/* Called by ax12 module when we want to receive a char. Note that we
 * also receive the bytes we sent ! So we need to drop them. */
static int16_t ax12_recv_char(void)
{
	microseconds t = time_get_us2();
	int c;
	while (1) {
		c = uart_recv_nowait(1);
		if (c != -1) {
			if (ax12_nsent == 0)
				return c;
			ax12_nsent --;
		}

		/* 50 ms timeout */
		if ((time_get_us2() - t) > 50000)
			return -1;
	}
	return c;
}

/* called by ax12 module when we want to switch serial line. As we
 * work in interruption mode, this function can be called to switch
 * back in read mode even if the bytes are not really transmitted on
 * the line. That's why in this case we do nothing, we will fall back
 * in read mode in any case when xmit is finished -- see in
 * ax12_send_callback() -- */
static void ax12_switch_uart(uint8_t state)
{
	uint8_t flags;

	if (state == AX12_STATE_WRITE) {
		IRQ_LOCK(flags);
		ax12_nsent=0;
		while (uart_recv_nowait(1) != -1);
		UCSR1B |= (1<<TXEN);
		ax12_state = AX12_STATE_WRITE;
		IRQ_UNLOCK(flags);
	}
	else {
		IRQ_LOCK(flags);
		if (CIRBUF_IS_EMPTY(&g_tx_fifo[1]))
			UCSR1B &= ~(1<<TXEN);
		ax12_state = AX12_STATE_READ;
		IRQ_UNLOCK(flags);
	}
}

/***********************/

void do_led_blink(void * dummy)
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

/* called every 5 ms */
static void do_cs(void * dummy) 
{
	if (arm.flags & CS_ON)
		cs_manage(&arm.cs_mot);
	
	if (scanner.flags & CS_ON){
		cs_manage(&scanner.cs_mot);

	}
	
}

static void main_timer_interrupt(void)
{
	static uint8_t cpt = 0;
	static uint8_t encoder_running = 0;

	cpt++;

	/* log ? */
	if (encoder_running)
		return;

	encoder_running = 1;
	sei();

	encoders_microb_manage(NULL);
	encoder_running = 0;

	if ((cpt & 0x3) == 0)
		scheduler_interrupt();
}

/* sending "pop" on uart0 resets the robot */
static void emergency(char c) {
	static uint8_t i = 0;
	
	if( (i == 0 && c == 'p') ||
	    (i == 1 && c == 'o') ||
	    (i == 2 && c == 'p') )
		i++;
	else if ( !(i == 1 && c == 'p') )
		i = 0;
	if(i == 3){
		reset();
		//PORTG|=0x3;

	}
}

//#define SCANNER_STEP_TOUR (3525L)

/* called every 1 ms */
#define STEP_PER_POS 64L
#define PIX_PER_SCAN 80L

int32_t pos_start_scan;
int32_t last_tour_n;
int32_t last_tour_pos;




#define X  45.
#define Y  -11.
#define l1  9.
#define l2  21.13
#define l3  47.14
#define l_mirror  235.
#define h_mirror  15.


//#define offset_a (75.*M_PI/180.)
float offset_a;

/* get motor angle in radian; return mirror angle in radian, cos a sin a */
void ang2_a_mirror(float b, float * c_a, float* s_a, float* a)
{
	float x2, y2;
	float A, DELTA, B, D;

	b+=offset_a;
	x2 = X + l1*cos(b);
	y2 = Y + l1*sin(b);

	A = (l3*l3+(x2)*(x2)+(y2)*(y2)-l2*l2)/(2*l3);

	DELTA = -(A*A-(x2)*(x2)-(y2)*(y2));
	B = sqrt(DELTA);

	D = (x2)*(x2)+(y2)*(y2);

	*c_a = (x2*A+y2*B)/D;
	*s_a = -(x2*B-y2*A)/D;

	*a = atan2(*s_a, *c_a);
}

/* get telemeter dist , cos a, sin a, a and return H, L of scanned point */
void ang2_H_L(float l_telemetre, float c_a, float s_a, float a, float *H, float *L)
{
	float d;
	d = h_mirror*c_a/s_a;
	*H = (l_telemetre - l_mirror - d)*sin(2*a);
	*L = l_mirror + d + *H/tan(2*a);
}

// d_telemetre = a * cm + b
#define TELEMETRE_A (16.76)
#define TELEMETRE_B (-476.)

static void do_adc(void * dummy) 
{

	int16_t a;
	int32_t tour_n;
	int32_t tour_pos;
	int32_t pos, pos_tmp, last_pos;
	int32_t mot_pos;
	float dist;

	float b, c_a, s_a, H, L, m_a;


	if (sample_i==0)
		return;

	mot_pos = encoders_microb_get_value((void *)SCANNER_ENC);
	mot_pos-=pos_start_scan;

	if (sample_i==1){
		printf_P(PSTR("dump end enc %ld %d \r\n"), mot_pos, PIX_PER_SCAN);
		//scanner.flags &= (~CS_ON);
		
		mot_pos = SCANNER_STEP_TOUR*(encoders_microb_get_value((void *)SCANNER_ENC)/SCANNER_STEP_TOUR+1L);
		cs_set_consign(&scanner.cs_mot, mot_pos);
		//pwm_ng_set(SCANNER_MOT_PWM, 0);

		
	}
	a = adc_get_value( ADC_REF_AVCC | MUX_ADC0 );
	//printf_P(PSTR("polling : ADC0 = %i\n"),a);
	dist = (a-TELEMETRE_B)/TELEMETRE_A;

	//printf_P(PSTR("enc val = %ld\r\n"),	encoders_microb_get_value((void *)SCANNER_ENC));


	//sample_tab[MAX_SAMPLE-sample_i] = a>0x1ff?0x1FF:a;
	//sample_tab[MAX_SAMPLE-sample_i] |= PINF&2?0x200:0;


	tour_n = (mot_pos)/(SCANNER_STEP_TOUR);
	tour_pos = (mot_pos)%(SCANNER_STEP_TOUR);

	b = (2.*M_PI)*(float)tour_pos/(float)(SCANNER_STEP_TOUR);
	ang2_a_mirror(b, &c_a, &s_a, &m_a);
	ang2_H_L(dist, c_a, s_a, m_a, &H, &L);

	H = H;//m_a*180/M_PI;
	L = L;//(L-100)*PIX_PER_SCAN;


	//printf_P(PSTR("%f %f\r\n"), dist, m_a*180/M_PI);
	//printf_P(PSTR("%f %f\r\n"), m_a*180/M_PI, b*180/M_PI);

	//printf_P(PSTR("%d %f\r\n"), a, b*180/M_PI);
	//printf_P(PSTR("%f %f\r\n"), H, L);
	printf_P(PSTR("%f %f %f\r\n"), H, m_a*180/M_PI, offset_a);
	if (tour_n%2){
		//tour_pos = ((SCANNER_STEP_TOUR/2)-tour_pos);
		tour_pos = (tour_pos*PIX_PER_SCAN)/(SCANNER_STEP_TOUR);
	}
	else{
		tour_pos = ((SCANNER_STEP_TOUR-tour_pos)*PIX_PER_SCAN)/(SCANNER_STEP_TOUR);
	}
	
	

	//pos = (tour_n*SCANNER_STEP_TOUR + tour_pos)/STEP_PER_POS;
	pos= tour_n*PIX_PER_SCAN+tour_pos;
	last_pos= last_tour_n*PIX_PER_SCAN+last_tour_pos;
	
	//a-=0x100;
	a-=200;
	//a/=10;

	if (pos <MAX_SAMPLE)// && tour_n%2)
		//sample_tab[pos] =  a>0xff?0xFF:a;
		//sample_tab[(int)L] = H ;
		sample_tab[pos] = H;
		nop();

	if (last_tour_n == tour_n){
		if (pos > last_pos){
			pos_tmp = pos;
			pos = last_pos;
			last_pos = pos_tmp;
		}
		for (pos_tmp=pos;pos_tmp< last_pos;pos_tmp++){
			if (pos_tmp <MAX_SAMPLE)// && tour_n%2)
				//sample_tab[pos_tmp] =  a>0xff?0xFF:a;
				nop();
				
		}

	}



	last_tour_n = tour_n;
	last_tour_pos = tour_pos;

	
	//printf("pos : %ld\r\n", pos);
	//sample_tab[sample_i] =  a>0x1ff?0x1FF:a;

	//sample_ok_tab[MAX_SAMPLE-sample_i] = PORTF&2;

	/*
	if (((pos <MAX_SAMPLE)) && (tour_pos<=(SCANNER_STEP_TOUR/2)))
		sample_tab[pos] = 0xffff;
	*/
	sample_i--;
}

int main(void)
{
	int c;
	const char * history;
	int8_t ret;

	/* brake */
	DDRG=0x3;
	PORTG=0x0;

	/* LEDS */
	DDRE=0x0C;

	LED1_OFF();
	memset(&arm, 0, sizeof(struct arm));

	/* PID */
	pid_init(&arm.pid_mot);
	pid_set_gains(&arm.pid_mot, 80, 5, 250);
	pid_set_maximums(&arm.pid_mot, 0, 10000, 4095);
	pid_set_out_shift(&arm.pid_mot, 6);
	pid_set_derivate_filter(&arm.pid_mot, 6);


	/* QUADRAMP */
	quadramp_init(&arm.qr_mot);
	quadramp_set_1st_order_vars(&arm.qr_mot, 200, 200); /* set speed */
	quadramp_set_2nd_order_vars(&arm.qr_mot, 20, 20); /* set accel */

	/* CS */
	memset(&scanner, 0, sizeof(struct arm));

	cs_init(&arm.cs_mot);
	cs_set_consign_filter(&arm.cs_mot, quadramp_do_filter, &arm.qr_mot);
	cs_set_correct_filter(&arm.cs_mot, pid_do_filter, &arm.pid_mot);
	cs_set_process_in(&arm.cs_mot, pwm_ng_set, ARM_MOT_PWM);
	cs_set_process_out(&arm.cs_mot, encoders_microb_get_value, ARM_ENC);
	cs_set_consign(&arm.cs_mot, 0);



	pid_init(&scanner.pid_mot);
	pid_set_gains(&scanner.pid_mot, 80, 5, 250);
	pid_set_maximums(&scanner.pid_mot, 0, 10000, 2047);
	pid_set_out_shift(&scanner.pid_mot, 6);
	pid_set_derivate_filter(&scanner.pid_mot, 6);
	
	
	quadramp_init(&scanner.qr_mot);
	quadramp_set_1st_order_vars(&scanner.qr_mot, 40, 40); /* set speed */
	quadramp_set_2nd_order_vars(&scanner.qr_mot, 20, 20); /* set accel */
	
	
	cs_init(&scanner.cs_mot);
	cs_set_consign_filter(&scanner.cs_mot, quadramp_do_filter, &scanner.qr_mot);
	cs_set_correct_filter(&scanner.cs_mot, pid_do_filter, &scanner.pid_mot);
	cs_set_process_in(&scanner.cs_mot, pwm_ng_set, SCANNER_MOT_PWM);
	cs_set_process_out(&scanner.cs_mot, encoders_microb_get_value, SCANNER_ENC);
	cs_set_consign(&scanner.cs_mot, 0);
	
	//scanner.flags |= CS_ON; 
	


#if 0
	/* SPI */
	spi_init(SPI_MODE_MASTER, SPI_FORMAT_2, SPI_CLK_RATE_16);
	spi_set_data_order(SPI_MSB_FIRST);
	spi_register_ss_line(&SS_PORT, SS_BIT);
#endif

	/* UART */
	/* Initialize full duplex uart direction port */
	sbi(PORTD,3); /* pullup */
	uart_init();
	/* disable rx intr, needed for AX12 !! */
	//UCSRnB &= ~( (1 << RXCIE) | (1 << UDRIE) | (1 << TXCIE) );

	ax12_switch_uart(AX12_STATE_READ);
 	fdevopen(uart0_dev_send, uart0_dev_recv);
	uart_register_rx_event(0, emergency);
 
	/* I2C */
	wait_ms(50);
/* 	i2c_protocol_init(); */
	i2c_init(I2C_MODE_MASTER, 0/* I2C_MAIN_ADDR */);
/* 	i2c_register_recv_event(i2c_recvevent); */
/* 	i2c_register_send_event(i2c_sendevent); */
/* 	scheduler_add_periodical_event_priority(i2c_poll_slaves, NULL, */
/* 						8000L / SCHEDULER_UNIT, I2C_POLL_PRIO); */

	/* AX12 */
	AX12_init(&ax12);
	AX12_set_hardware_send(&ax12, ax12_send_char);
	AX12_set_hardware_recv(&ax12, ax12_recv_char);
	AX12_set_hardware_switch(&ax12, ax12_switch_uart);
	uart_register_tx_event(1, ax12_send_callback);

	/* ENCODERS */
	encoders_microb_init();

	/* TIMER */
	timer_init();
	timer0_register_OV_intr(main_timer_interrupt);

	/* SCHEDULER */
	scheduler_init();
	scheduler_add_periodical_event_priority(do_led_blink, NULL, 
						100000L / SCHEDULER_UNIT, 
						LED_PRIO);
	/* PWM */
	PWM_NG_TIMER_16BITS_INIT(1, TIMER_16_MODE_PWM_10, 
				 TIMER1_PRESCALER_DIV_1);
	
	PWM_NG_TIMER_16BITS_INIT(3, TIMER_16_MODE_PWM_10, 
				 TIMER1_PRESCALER_DIV_1);
	
/* 	pwm_ng_timer_8bits_init(2, TIMER_8_MODE_PWM,  */
/* 				 TIMER1_PRESCALER_DIV_1); */
	PWM_NG_INIT16(&arm.pwm1A, 1, A, 10, PWM_NG_MODE_SIGNED | 
		      PWM_NG_MODE_SIGN_INVERTED, &PORTB, 0);
	PWM_NG_INIT16(&arm.pwm1B, 1, B, 10, PWM_NG_MODE_SIGNED,
		      &PORTB, 1);
	PWM_NG_INIT16(&arm.pwm3C, 3, C, 10, PWM_NG_MODE_SIGNED,
		      &PORTE, 4);
/* 	PWM_NG_INIT8(&arm.pwm2, 2, 10, PWM_NG_MODE_SIGNED, */
/* 		      &PORTB, 2); */

	/* CS EVENT */
	scheduler_add_periodical_event_priority(do_cs, NULL, 
						CS_PERIOD / SCHEDULER_UNIT, 
						CS_PRIO);
	
	/* ADC EVENT */
	
	adc_init();
	scheduler_add_periodical_event_priority(do_adc, NULL, 
						2000L / SCHEDULER_UNIT, 
						CS_PRIO-1);
	
	wait_ms(200);

	/* arm xy init matrix */
	init_arm_matrix();

	/* TIME */
	time_init(TIME_PRIO);

	wait_ms(200);

	sei();

	printf_P(PSTR("Coucou\r\n"));
	
	/* set status return level to 2 and torque to 0 */
	AX12_write_int(&ax12,0xFE, AA_TORQUE_ENABLE, 0x00);
	AX12_write_byte(&ax12, 0xFE, AA_STATUS_RETURN_LEVEL, 2);

	rdline_init(&rdl, write_char, valid_buffer, complete_buffer);
	snprintf(prompt, sizeof(prompt), "ax12 > ");	
	rdline_newline(&rdl, prompt);


	while (1) {
		c = uart_recv_nowait(0);
		if (c == -1) 
			continue;
		ret = rdline_char_in(&rdl, c);
		if (ret != 2 && ret != 0) {
			history = rdline_get_buffer(&rdl);
			if (strlen(history) > 1)
				rdline_add_history(&rdl, history);
			rdline_newline(&rdl, prompt);
		}
	}

	return 0;
}
