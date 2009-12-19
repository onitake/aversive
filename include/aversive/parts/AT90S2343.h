/*  
 *  Copyright Droids Corporation, Microb Technology, Eirbot (2009)
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
 *  Revision : $Id $
 *
 */

/* WARNING : this file is automatically generated by scripts.
 * You should not edit it. If you find something wrong in it,
 * write to zer0@droids-corp.org */


/* prescalers timer 0 */
#define TIMER0_PRESCALER_DIV_0          0
#define TIMER0_PRESCALER_DIV_1          1
#define TIMER0_PRESCALER_DIV_8          2
#define TIMER0_PRESCALER_DIV_64         3
#define TIMER0_PRESCALER_DIV_256        4
#define TIMER0_PRESCALER_DIV_1024       5
#define TIMER0_PRESCALER_DIV_FALL       6
#define TIMER0_PRESCALER_DIV_RISE       7

#define TIMER0_PRESCALER_REG_0          0
#define TIMER0_PRESCALER_REG_1          1
#define TIMER0_PRESCALER_REG_2          8
#define TIMER0_PRESCALER_REG_3          64
#define TIMER0_PRESCALER_REG_4          256
#define TIMER0_PRESCALER_REG_5          1024
#define TIMER0_PRESCALER_REG_6          -1
#define TIMER0_PRESCALER_REG_7          -2


/* available timers */
#define TIMER0_AVAILABLE

/* overflow interrupt number */
#define SIG_OVERFLOW0_NUM 0
#define SIG_OVERFLOW_TOTAL_NUM 1

/* output compare interrupt number */
#define SIG_OUTPUT_COMPARE_TOTAL_NUM 0

/* Pwm nums */
#define PWM_TOTAL_NUM 0

/* input capture interrupt number */
#define SIG_INPUT_CAPTURE_TOTAL_NUM 0


/* GIFR */
#define INTF0_REG            GIFR

/* TIMSK */
#define TOIE0_REG            TIMSK

/* WDTCR */
#define WDP0_REG             WDTCR
#define WDP1_REG             WDTCR
#define WDP2_REG             WDTCR
#define WDE_REG              WDTCR
#define WDTOE_REG            WDTCR

/* GIMSK */
#define INT0_REG             GIMSK

/* EECR */
#define EERE_REG             EECR
#define EEWE_REG             EECR
#define EEMWE_REG            EECR

/* PINB */
#define PINB0_REG            PINB
#define PINB1_REG            PINB
#define PINB2_REG            PINB
#define PINB3_REG            PINB
#define PINB4_REG            PINB

/* EEAR */
#define EEAR0_REG            EEAR
#define EEAR1_REG            EEAR
#define EEAR2_REG            EEAR
#define EEAR3_REG            EEAR
#define EEAR4_REG            EEAR
#define EEAR5_REG            EEAR
#define EEAR6_REG            EEAR

/* PORTB */
#define PORTB0_REG           PORTB
#define PORTB1_REG           PORTB
#define PORTB2_REG           PORTB
#define PORTB3_REG           PORTB
#define PORTB4_REG           PORTB

/* TCCR0 */
#define CS00_REG             TCCR0
#define CS01_REG             TCCR0
#define CS02_REG             TCCR0

/* SREG */
#define C_REG                SREG
#define Z_REG                SREG
#define N_REG                SREG
#define V_REG                SREG
#define S_REG                SREG
#define H_REG                SREG
#define T_REG                SREG
#define I_REG                SREG

/* DDRB */
#define DDB0_REG             DDRB
#define DDB1_REG             DDRB
#define DDB2_REG             DDRB
#define DDB3_REG             DDRB
#define DDB4_REG             DDRB

/* TCNT0 */
#define TCNT00_REG           TCNT0
#define TCNT01_REG           TCNT0
#define TCNT02_REG           TCNT0
#define TCNT03_REG           TCNT0
#define TCNT04_REG           TCNT0
#define TCNT05_REG           TCNT0
#define TCNT06_REG           TCNT0
#define TCNT07_REG           TCNT0

/* SPL */
#define SP0_REG              SPL
#define SP1_REG              SPL
#define SP2_REG              SPL
#define SP3_REG              SPL
#define SP4_REG              SPL
#define SP5_REG              SPL
#define SP6_REG              SPL
#define SP7_REG              SPL

/* EEDR */
#define EEDR0_REG            EEDR
#define EEDR1_REG            EEDR
#define EEDR2_REG            EEDR
#define EEDR3_REG            EEDR
#define EEDR4_REG            EEDR
#define EEDR5_REG            EEDR
#define EEDR6_REG            EEDR
#define EEDR7_REG            EEDR

/* MCUCR */
#define ISC00_REG            MCUCR
#define ISC01_REG            MCUCR
#define SM_REG               MCUCR
#define SE_REG               MCUCR

/* TIFR */
#define TOV0_REG             TIFR

/* MCUSR */
#define PORF_REG             MCUSR
#define EXTRF_REG            MCUSR

/* pins mapping */
#define MOSI_PORT PORTB
#define MOSI_BIT 0

#define MISO_PORT PORTB
#define MISO_BIT 1
#define INT0_PORT PORTB
#define INT0_BIT 1

#define SCK_PORT PORTB
#define SCK_BIT 2
#define T0_PORT PORTB
#define T0_BIT 2

#define CLOCK_PORT PORTB
#define CLOCK_BIT 3



