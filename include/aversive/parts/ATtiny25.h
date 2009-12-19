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

/* prescalers timer 1 */
#define TIMER1_PRESCALER_DIV_0          0
#define TIMER1_PRESCALER_DIV_1          1
#define TIMER1_PRESCALER_DIV_2          2
#define TIMER1_PRESCALER_DIV_4          3
#define TIMER1_PRESCALER_DIV_8          4
#define TIMER1_PRESCALER_DIV_16         5
#define TIMER1_PRESCALER_DIV_32         6
#define TIMER1_PRESCALER_DIV_64         7
#define TIMER1_PRESCALER_DIV_128        8
#define TIMER1_PRESCALER_DIV_256        9
#define TIMER1_PRESCALER_DIV_512        10
#define TIMER1_PRESCALER_DIV_1024       11
#define TIMER1_PRESCALER_DIV_2048       12
#define TIMER1_PRESCALER_DIV_4096       13
#define TIMER1_PRESCALER_DIV_8192       14
#define TIMER1_PRESCALER_DIV_16384      15

#define TIMER1_PRESCALER_REG_0          0
#define TIMER1_PRESCALER_REG_1          1
#define TIMER1_PRESCALER_REG_2          2
#define TIMER1_PRESCALER_REG_3          4
#define TIMER1_PRESCALER_REG_4          8
#define TIMER1_PRESCALER_REG_5          16
#define TIMER1_PRESCALER_REG_6          32
#define TIMER1_PRESCALER_REG_7          64
#define TIMER1_PRESCALER_REG_8          128
#define TIMER1_PRESCALER_REG_9          256
#define TIMER1_PRESCALER_REG_10         512
#define TIMER1_PRESCALER_REG_11         1024
#define TIMER1_PRESCALER_REG_12         2048
#define TIMER1_PRESCALER_REG_13         4096
#define TIMER1_PRESCALER_REG_14         8192
#define TIMER1_PRESCALER_REG_15         16384


/* available timers */
#define TIMER0_AVAILABLE
#define TIMER0A_AVAILABLE
#define TIMER0B_AVAILABLE
#define TIMER1_AVAILABLE
#define TIMER1A_AVAILABLE
#define TIMER1B_AVAILABLE

/* overflow interrupt number */
#define SIG_OVERFLOW0_NUM 0
#define SIG_OVERFLOW1_NUM 1
#define SIG_OVERFLOW_TOTAL_NUM 2

/* output compare interrupt number */
#define SIG_OUTPUT_COMPARE0A_NUM 0
#define SIG_OUTPUT_COMPARE0B_NUM 1
#define SIG_OUTPUT_COMPARE1A_NUM 2
#define SIG_OUTPUT_COMPARE1B_NUM 3
#define SIG_OUTPUT_COMPARE_TOTAL_NUM 4

/* Pwm nums */
#define PWM0A_NUM 0
#define PWM0B_NUM 1
#define PWM1A_NUM 2
#define PWM1B_NUM 3
#define PWM_TOTAL_NUM 4

/* input capture interrupt number */
#define SIG_INPUT_CAPTURE_TOTAL_NUM 0


/* CLKPR */
#define CLKPS0_REG           CLKPR
#define CLKPS1_REG           CLKPR
#define CLKPS2_REG           CLKPR
#define CLKPS3_REG           CLKPR
#define CLKPCE_REG           CLKPR

/* WDTCR */
#define WDP0_REG             WDTCR
#define WDP1_REG             WDTCR
#define WDP2_REG             WDTCR
#define WDE_REG              WDTCR
#define WDCE_REG             WDTCR
#define WDP3_REG             WDTCR
#define WDIE_REG             WDTCR
#define WDIF_REG             WDTCR

/* GIMSK */
#define PCIE_REG             GIMSK
#define INT0_REG             GIMSK

/* DIDR0 */
#define AIN0D_REG            DIDR0
#define AIN1D_REG            DIDR0
#define ADC1D_REG            DIDR0
#define ADC3D_REG            DIDR0
#define ADC2D_REG            DIDR0
#define ADC0D_REG            DIDR0

/* ADMUX */
#define MUX0_REG             ADMUX
#define MUX1_REG             ADMUX
#define MUX2_REG             ADMUX
#define MUX3_REG             ADMUX
#define REFS2_REG            ADMUX
#define ADLAR_REG            ADMUX
#define REFS0_REG            ADMUX
#define REFS1_REG            ADMUX

/* TCCR1 */
#define CS10_REG             TCCR1
#define CS11_REG             TCCR1
#define CS12_REG             TCCR1
#define CS13_REG             TCCR1
#define COM1A0_REG           TCCR1
#define COM1A1_REG           TCCR1
#define PWM1A_REG            TCCR1
#define CTC1_REG             TCCR1

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
#define DDB5_REG             DDRB

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
#define SM0_REG              MCUCR
#define SM1_REG              MCUCR
#define SE_REG               MCUCR
#define PUD_REG              MCUCR

/* GTCCR */
#define PSR0_REG             GTCCR
#define TSM_REG              GTCCR
#define PSR1_REG             GTCCR
#define FOC1A_REG            GTCCR
#define FOC1B_REG            GTCCR
#define COM1B0_REG           GTCCR
#define COM1B1_REG           GTCCR
#define PWM1B_REG            GTCCR

/* DTPS */
#define DTPS0_REG            DTPS
#define DTPS1_REG            DTPS

/* GIFR */
#define PCIF_REG             GIFR
#define INTF0_REG            GIFR

/* TIMSK */
#define TOIE0_REG            TIMSK
#define OCIE0B_REG           TIMSK
#define OCIE0A_REG           TIMSK
#define TOIE1_REG            TIMSK
#define OCIE1B_REG           TIMSK
#define OCIE1A_REG           TIMSK

/* ADCSRA */
#define ADPS0_REG            ADCSRA
#define ADPS1_REG            ADCSRA
#define ADPS2_REG            ADCSRA
#define ADIE_REG             ADCSRA
#define ADIF_REG             ADCSRA
#define ADATE_REG            ADCSRA
#define ADSC_REG             ADCSRA
#define ADEN_REG             ADCSRA

/* DT1B */
/* #define DTVL0_REG            DT1B */ /* dup in DT1A */
/* #define DTVL1_REG            DT1B */ /* dup in DT1A */
/* #define DTVL2_REG            DT1B */ /* dup in DT1A */
/* #define DTVL3_REG            DT1B */ /* dup in DT1A */
/* #define DTVH0_REG            DT1B */ /* dup in DT1A */
/* #define DTVH1_REG            DT1B */ /* dup in DT1A */
/* #define DTVH2_REG            DT1B */ /* dup in DT1A */
/* #define DTVH3_REG            DT1B */ /* dup in DT1A */

/* OCR0A */
/* #define OCR0_0_REG           OCR0A */ /* dup in OCR0B */
/* #define OCR0_1_REG           OCR0A */ /* dup in OCR0B */
/* #define OCR0_2_REG           OCR0A */ /* dup in OCR0B */
/* #define OCR0_3_REG           OCR0A */ /* dup in OCR0B */
/* #define OCR0_4_REG           OCR0A */ /* dup in OCR0B */
/* #define OCR0_5_REG           OCR0A */ /* dup in OCR0B */
/* #define OCR0_6_REG           OCR0A */ /* dup in OCR0B */
/* #define OCR0_7_REG           OCR0A */ /* dup in OCR0B */

/* OCR0B */
/* #define OCR0_0_REG           OCR0B */ /* dup in OCR0A */
/* #define OCR0_1_REG           OCR0B */ /* dup in OCR0A */
/* #define OCR0_2_REG           OCR0B */ /* dup in OCR0A */
/* #define OCR0_3_REG           OCR0B */ /* dup in OCR0A */
/* #define OCR0_4_REG           OCR0B */ /* dup in OCR0A */
/* #define OCR0_5_REG           OCR0B */ /* dup in OCR0A */
/* #define OCR0_6_REG           OCR0B */ /* dup in OCR0A */
/* #define OCR0_7_REG           OCR0B */ /* dup in OCR0A */

/* SPL */
#define SP0_REG              SPL
#define SP1_REG              SPL
#define SP2_REG              SPL
#define SP3_REG              SPL
#define SP4_REG              SPL
#define SP5_REG              SPL
#define SP6_REG              SPL
#define SP7_REG              SPL

/* PRR */
#define PRADC_REG            PRR
#define PRUSI_REG            PRR
#define PRTIM0_REG           PRR
#define PRTIM1_REG           PRR

/* GPIOR1 */
#define GPIOR10_REG          GPIOR1
#define GPIOR11_REG          GPIOR1
#define GPIOR12_REG          GPIOR1
#define GPIOR13_REG          GPIOR1
#define GPIOR14_REG          GPIOR1
#define GPIOR15_REG          GPIOR1
#define GPIOR16_REG          GPIOR1
#define GPIOR17_REG          GPIOR1

/* GPIOR0 */
#define GPIOR00_REG          GPIOR0
#define GPIOR01_REG          GPIOR0
#define GPIOR02_REG          GPIOR0
#define GPIOR03_REG          GPIOR0
#define GPIOR04_REG          GPIOR0
#define GPIOR05_REG          GPIOR0
#define GPIOR06_REG          GPIOR0
#define GPIOR07_REG          GPIOR0

/* GPIOR2 */
#define GPIOR20_REG          GPIOR2
#define GPIOR21_REG          GPIOR2
#define GPIOR22_REG          GPIOR2
#define GPIOR23_REG          GPIOR2
#define GPIOR24_REG          GPIOR2
#define GPIOR25_REG          GPIOR2
#define GPIOR26_REG          GPIOR2
#define GPIOR27_REG          GPIOR2

/* MCUSR */
#define PORF_REG             MCUSR
#define EXTRF_REG            MCUSR
#define BORF_REG             MCUSR
#define WDRF_REG             MCUSR

/* EECR */
#define EERE_REG             EECR
#define EEPE_REG             EECR
#define EEMPE_REG            EECR
#define EERIE_REG            EECR
#define EEPM0_REG            EECR
#define EEPM1_REG            EECR

/* PCMSK */
#define PCINT0_REG           PCMSK
#define PCINT1_REG           PCMSK
#define PCINT2_REG           PCMSK
#define PCINT3_REG           PCMSK
#define PCINT4_REG           PCMSK
#define PCINT5_REG           PCMSK

/* SPMCSR */
#define SPMEN_REG            SPMCSR
#define PGERS_REG            SPMCSR
#define PGWRT_REG            SPMCSR
#define RFLB_REG             SPMCSR
#define CTPB_REG             SPMCSR

/* OSCCAL */
#define CAL0_REG             OSCCAL
#define CAL1_REG             OSCCAL
#define CAL2_REG             OSCCAL
#define CAL3_REG             OSCCAL
#define CAL4_REG             OSCCAL
#define CAL5_REG             OSCCAL
#define CAL6_REG             OSCCAL
#define CAL7_REG             OSCCAL

/* ADCL */
#define ADCL0_REG            ADCL
#define ADCL1_REG            ADCL
#define ADCL2_REG            ADCL
#define ADCL3_REG            ADCL
#define ADCL4_REG            ADCL
#define ADCL5_REG            ADCL
#define ADCL6_REG            ADCL
#define ADCL7_REG            ADCL

/* USISR */
#define USICNT0_REG          USISR
#define USICNT1_REG          USISR
#define USICNT2_REG          USISR
#define USICNT3_REG          USISR
#define USIDC_REG            USISR
#define USIPF_REG            USISR
#define USIOIF_REG           USISR
#define USISIF_REG           USISR

/* PORTB */
#define PORTB0_REG           PORTB
#define PORTB1_REG           PORTB
#define PORTB2_REG           PORTB
#define PORTB3_REG           PORTB
#define PORTB4_REG           PORTB
#define PORTB5_REG           PORTB

/* ADCH */
#define ADCH0_REG            ADCH
#define ADCH1_REG            ADCH
#define ADCH2_REG            ADCH
#define ADCH3_REG            ADCH
#define ADCH4_REG            ADCH
#define ADCH5_REG            ADCH
#define ADCH6_REG            ADCH
#define ADCH7_REG            ADCH

/* TCNT0 */
#define TCNT0_0_REG          TCNT0
#define TCNT0_1_REG          TCNT0
#define TCNT0_2_REG          TCNT0
#define TCNT0_3_REG          TCNT0
#define TCNT0_4_REG          TCNT0
#define TCNT0_5_REG          TCNT0
#define TCNT0_6_REG          TCNT0
#define TCNT0_7_REG          TCNT0

/* TCNT1 */
#define TCNT1_0_REG          TCNT1
#define TCNT1_1_REG          TCNT1
#define TCNT1_2_REG          TCNT1
#define TCNT1_3_REG          TCNT1
#define TCNT1_4_REG          TCNT1
#define TCNT1_5_REG          TCNT1
#define TCNT1_6_REG          TCNT1
#define TCNT1_7_REG          TCNT1

/* TCCR0B */
#define CS00_REG             TCCR0B
#define CS01_REG             TCCR0B
#define CS02_REG             TCCR0B
#define WGM02_REG            TCCR0B
#define FOC0B_REG            TCCR0B
#define FOC0A_REG            TCCR0B

/* TIFR */
#define TOV0_REG             TIFR
#define OCF0B_REG            TIFR
#define OCF0A_REG            TIFR
#define TOV1_REG             TIFR
#define OCF1B_REG            TIFR
#define OCF1A_REG            TIFR

/* TCCR0A */
#define WGM00_REG            TCCR0A
#define WGM01_REG            TCCR0A
#define COM0B0_REG           TCCR0A
#define COM0B1_REG           TCCR0A
#define COM0A0_REG           TCCR0A
#define COM0A1_REG           TCCR0A

/* EEARH */
#define EEAR8_REG            EEARH

/* PLLCSR */
#define PLOCK_REG            PLLCSR
#define PLLE_REG             PLLCSR
#define PCKE_REG             PLLCSR
#define LSM_REG              PLLCSR

/* USICR */
#define USITC_REG            USICR
#define USICLK_REG           USICR
#define USICS0_REG           USICR
#define USICS1_REG           USICR
#define USIWM0_REG           USICR
#define USIWM1_REG           USICR
#define USIOIE_REG           USICR
#define USISIE_REG           USICR

/* EEARL */
#define EEAR0_REG            EEARL
#define EEAR1_REG            EEARL
#define EEAR2_REG            EEARL
#define EEAR3_REG            EEARL
#define EEAR4_REG            EEARL
#define EEAR5_REG            EEARL
#define EEAR6_REG            EEARL
#define EEAR7_REG            EEARL

/* DWDR */
#define DWDR0_REG            DWDR
#define DWDR1_REG            DWDR
#define DWDR2_REG            DWDR
#define DWDR3_REG            DWDR
#define DWDR4_REG            DWDR
#define DWDR5_REG            DWDR
#define DWDR6_REG            DWDR
#define DWDR7_REG            DWDR

/* ADCSRB */
#define ACME_REG             ADCSRB
#define ADTS0_REG            ADCSRB
#define ADTS1_REG            ADCSRB
#define ADTS2_REG            ADCSRB
#define IPR_REG              ADCSRB
#define BIN_REG              ADCSRB

/* OCR1B */
#define OCR1B0_REG           OCR1B
#define OCR1B1_REG           OCR1B
#define OCR1B2_REG           OCR1B
#define OCR1B3_REG           OCR1B
#define OCR1B4_REG           OCR1B
#define OCR1B5_REG           OCR1B
#define OCR1B6_REG           OCR1B
#define OCR1B7_REG           OCR1B

/* OCR1C */
#define OCR1C0_REG           OCR1C
#define OCR1C1_REG           OCR1C
#define OCR1C2_REG           OCR1C
#define OCR1C3_REG           OCR1C
#define OCR1C4_REG           OCR1C
#define OCR1C5_REG           OCR1C
#define OCR1C6_REG           OCR1C
#define OCR1C7_REG           OCR1C

/* DT1A */
/* #define DTVL0_REG            DT1A */ /* dup in DT1B */
/* #define DTVL1_REG            DT1A */ /* dup in DT1B */
/* #define DTVL2_REG            DT1A */ /* dup in DT1B */
/* #define DTVL3_REG            DT1A */ /* dup in DT1B */
/* #define DTVH0_REG            DT1A */ /* dup in DT1B */
/* #define DTVH1_REG            DT1A */ /* dup in DT1B */
/* #define DTVH2_REG            DT1A */ /* dup in DT1B */
/* #define DTVH3_REG            DT1A */ /* dup in DT1B */

/* OCR1A */
#define OCR1A0_REG           OCR1A
#define OCR1A1_REG           OCR1A
#define OCR1A2_REG           OCR1A
#define OCR1A3_REG           OCR1A
#define OCR1A4_REG           OCR1A
#define OCR1A5_REG           OCR1A
#define OCR1A6_REG           OCR1A
#define OCR1A7_REG           OCR1A

/* ACSR */
#define ACIS0_REG            ACSR
#define ACIS1_REG            ACSR
#define ACIE_REG             ACSR
#define ACI_REG              ACSR
#define ACO_REG              ACSR
#define ACBG_REG             ACSR
#define ACD_REG              ACSR

/* PINB */
#define PINB0_REG            PINB
#define PINB1_REG            PINB
#define PINB2_REG            PINB
#define PINB3_REG            PINB
#define PINB4_REG            PINB
#define PINB5_REG            PINB

/* USIBR */
#define USIBR0_REG           USIBR
#define USIBR1_REG           USIBR
#define USIBR2_REG           USIBR
#define USIBR3_REG           USIBR
#define USIBR4_REG           USIBR
#define USIBR5_REG           USIBR
#define USIBR6_REG           USIBR
#define USIBR7_REG           USIBR

/* USIDR */
#define USIDR0_REG           USIDR
#define USIDR1_REG           USIDR
#define USIDR2_REG           USIDR
#define USIDR3_REG           USIDR
#define USIDR4_REG           USIDR
#define USIDR5_REG           USIDR
#define USIDR6_REG           USIDR
#define USIDR7_REG           USIDR

/* pins mapping */
#define MOSI_PORT PORTB
#define MOSI_BIT 0
#define DI_PORT PORTB
#define DI_BIT 0
#define SDA_PORT PORTB
#define SDA_BIT 0
#define AIN0_PORT PORTB
#define AIN0_BIT 0
#define OC0A_PORT PORTB
#define OC0A_BIT 0
#define OC1A_PORT PORTB
#define OC1A_BIT 0
#define AREF_PORT PORTB
#define AREF_BIT 0
#define PCINT0_PORT PORTB
#define PCINT0_BIT 0

#define MISO_PORT PORTB
#define MISO_BIT 1
#define DO_PORT PORTB
#define DO_BIT 1
#define AIN1_PORT PORTB
#define AIN1_BIT 1
#define OC0B_PORT PORTB
#define OC0B_BIT 1
#define OC1A_PORT PORTB
#define OC1A_BIT 1
#define PCINT1_PORT PORTB
#define PCINT1_BIT 1

#define SCK_PORT PORTB
#define SCK_BIT 2
#define USCK_PORT PORTB
#define USCK_BIT 2
#define SCL_PORT PORTB
#define SCL_BIT 2
#define ADC1_PORT PORTB
#define ADC1_BIT 2
#define T0_PORT PORTB
#define T0_BIT 2
#define INT0_PORT PORTB
#define INT0_BIT 2
#define PCINT2_PORT PORTB
#define PCINT2_BIT 2

#define ADC3_PORT PORTB
#define ADC3_BIT 3
#define OC1B_PORT PORTB
#define OC1B_BIT 3
#define XTAL1_PORT PORTB
#define XTAL1_BIT 3
#define PCINT4_PORT PORTB
#define PCINT4_BIT 3

#define ADC2_PORT PORTB
#define ADC2_BIT 4
#define OC1B_PORT PORTB
#define OC1B_BIT 4
#define XTAL2_PORT PORTB
#define XTAL2_BIT 4
#define PCINT3_PORT PORTB
#define PCINT3_BIT 4

#define RESET_PORT PORTB
#define RESET_BIT 5
#define ADC0_PORT PORTB
#define ADC0_BIT 5
#define PCINT5_PORT PORTB
#define PCINT5_BIT 5
#define dW_PORT PORTB
#define dW_BIT 5


