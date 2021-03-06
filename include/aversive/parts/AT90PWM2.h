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
#define TIMER1_PRESCALER_DIV_8          2
#define TIMER1_PRESCALER_DIV_64         3
#define TIMER1_PRESCALER_DIV_256        4
#define TIMER1_PRESCALER_DIV_1024       5
#define TIMER1_PRESCALER_DIV_FALL       6
#define TIMER1_PRESCALER_DIV_RISE       7

#define TIMER1_PRESCALER_REG_0          0
#define TIMER1_PRESCALER_REG_1          1
#define TIMER1_PRESCALER_REG_2          8
#define TIMER1_PRESCALER_REG_3          64
#define TIMER1_PRESCALER_REG_4          256
#define TIMER1_PRESCALER_REG_5          1024
#define TIMER1_PRESCALER_REG_6          -1
#define TIMER1_PRESCALER_REG_7          -2


/* available timers */
#define TIMER0_AVAILABLE
#define TIMER0B_AVAILABLE
#define TIMER1_AVAILABLE
#define TIMER1A_AVAILABLE
#define TIMER1B_AVAILABLE

/* overflow interrupt number */
#define SIG_OVERFLOW0_NUM 0
#define SIG_OVERFLOW1_NUM 1
#define SIG_OVERFLOW_TOTAL_NUM 2

/* output compare interrupt number */
#define SIG_OUTPUT_COMPARE0_NUM 0
#define SIG_OUTPUT_COMPARE0B_NUM 1
#define SIG_OUTPUT_COMPARE1A_NUM 2
#define SIG_OUTPUT_COMPARE1B_NUM 3
#define SIG_OUTPUT_COMPARE_TOTAL_NUM 4

/* Pwm nums */
#define PWM0_NUM 0
#define PWM0B_NUM 1
#define PWM1A_NUM 2
#define PWM1B_NUM 3
#define PWM_TOTAL_NUM 4

/* input capture interrupt number */
#define SIG_INPUT_CAPTURE1_NUM 0
#define SIG_INPUT_CAPTURE_TOTAL_NUM 1


/* EUDR */
#define EUDR0_REG            EUDR
#define EUDR1_REG            EUDR
#define EUDR2_REG            EUDR
#define EUDR3_REG            EUDR
#define EUDR4_REG            EUDR
#define EUDR5_REG            EUDR
#define EUDR6_REG            EUDR
#define EUDR7_REG            EUDR

/* ADMUX */
#define MUX0_REG             ADMUX
#define MUX1_REG             ADMUX
#define MUX2_REG             ADMUX
#define MUX3_REG             ADMUX
#define ADLAR_REG            ADMUX
#define REFS0_REG            ADMUX
#define REFS1_REG            ADMUX

/* OCR2SBH */
#define OCR2SB_8_REG         OCR2SBH
#define OCR2SB_9_REG         OCR2SBH
#define OCR2SB_10_REG        OCR2SBH
#define OCR2SB_11_REG        OCR2SBH

/* OCR2SBL */
#define OCR2SB_0_REG         OCR2SBL
#define OCR2SB_1_REG         OCR2SBL
#define OCR2SB_2_REG         OCR2SBL
#define OCR2SB_3_REG         OCR2SBL
#define OCR2SB_4_REG         OCR2SBL
#define OCR2SB_5_REG         OCR2SBL
#define OCR2SB_6_REG         OCR2SBL
#define OCR2SB_7_REG         OCR2SBL

/* WDTCSR */
#define WDP0_REG             WDTCSR
#define WDP1_REG             WDTCSR
#define WDP2_REG             WDTCSR
#define WDE_REG              WDTCSR
#define WDCE_REG             WDTCSR
#define WDP3_REG             WDTCSR
#define WDIE_REG             WDTCSR
#define WDIF_REG             WDTCSR

/* EEDR */
#define EEDR0_REG            EEDR
#define EEDR1_REG            EEDR
#define EEDR2_REG            EEDR
#define EEDR3_REG            EEDR
#define EEDR4_REG            EEDR
#define EEDR5_REG            EEDR
#define EEDR6_REG            EEDR
#define EEDR7_REG            EEDR

/* OCR0B */
/* #define OCR0_0_REG           OCR0B */ /* dup in OCR0A */
/* #define OCR0_1_REG           OCR0B */ /* dup in OCR0A */
/* #define OCR0_2_REG           OCR0B */ /* dup in OCR0A */
/* #define OCR0_3_REG           OCR0B */ /* dup in OCR0A */
/* #define OCR0_4_REG           OCR0B */ /* dup in OCR0A */
/* #define OCR0_5_REG           OCR0B */ /* dup in OCR0A */
/* #define OCR0_6_REG           OCR0B */ /* dup in OCR0A */
/* #define OCR0_7_REG           OCR0B */ /* dup in OCR0A */

/* OCR0SAL */
#define OCR0SA_0_REG         OCR0SAL
#define OCR0SA_1_REG         OCR0SAL
#define OCR0SA_2_REG         OCR0SAL
#define OCR0SA_3_REG         OCR0SAL
#define OCR0SA_4_REG         OCR0SAL
#define OCR0SA_5_REG         OCR0SAL
#define OCR0SA_6_REG         OCR0SAL
#define OCR0SA_7_REG         OCR0SAL

/* SPDR */
#define SPDR0_REG            SPDR
#define SPDR1_REG            SPDR
#define SPDR2_REG            SPDR
#define SPDR3_REG            SPDR
#define SPDR4_REG            SPDR
#define SPDR5_REG            SPDR
#define SPDR6_REG            SPDR
#define SPDR7_REG            SPDR

/* SPSR */
#define SPI2X_REG            SPSR
#define WCOL_REG             SPSR
#define SPIF_REG             SPSR

/* SPH */
#define SP8_REG              SPH
#define SP9_REG              SPH
#define SP10_REG             SPH
#define SP11_REG             SPH
#define SP12_REG             SPH
#define SP13_REG             SPH
#define SP14_REG             SPH
#define SP15_REG             SPH

/* UCSRA */
#define MPCM_REG             UCSRA
#define U2X_REG              UCSRA
#define UPE_REG              UCSRA
#define DOR_REG              UCSRA
#define FE_REG               UCSRA
#define UDRE_REG             UCSRA
#define TXC_REG              UCSRA
#define RXC_REG              UCSRA

/* UCSRB */
#define TXB8_REG             UCSRB
#define RXB8_REG             UCSRB
#define UCSZ2_REG            UCSRB
#define TXEN_REG             UCSRB
#define RXEN_REG             UCSRB
#define UDRIE_REG            UCSRB
#define TXCIE_REG            UCSRB
#define RXCIE_REG            UCSRB

/* UCSRC */
#define UCPOL_REG            UCSRC
#define UCSZ0_REG            UCSRC
#define UCSZ1_REG            UCSRC
#define USBS_REG             UCSRC
#define UPM0_REG             UCSRC
#define UPM1_REG             UCSRC
#define UMSEL0_REG           UCSRC

/* SPL */
#define SP0_REG              SPL
#define SP1_REG              SPL
#define SP2_REG              SPL
#define SP3_REG              SPL
#define SP4_REG              SPL
#define SP5_REG              SPL
#define SP6_REG              SPL
#define SP7_REG              SPL

/* AC1CON */
#define AC1M0_REG            AC1CON
#define AC1M1_REG            AC1CON
#define AC1M2_REG            AC1CON
#define AC1ICE_REG           AC1CON
#define AC1IS0_REG           AC1CON
#define AC1IS1_REG           AC1CON
#define AC1IE_REG            AC1CON
#define AC1EN_REG            AC1CON

/* PRR */
#define PRADC_REG            PRR
#define PRUSART0_REG         PRR
#define PRSPI_REG            PRR
#define PRTIM0_REG           PRR
#define PRTIM1_REG           PRR
#define PRPSC0_REG           PRR
#define PRPSC1_REG           PRR
#define PRPSC2_REG           PRR

/* PCNF0 */
#define PCLKSEL0_REG         PCNF0
#define POP0_REG             PCNF0
#define PMODE00_REG          PCNF0
#define PMODE01_REG          PCNF0
#define PLOCK0_REG           PCNF0
#define PALOCK0_REG          PCNF0
#define PFIFTY0_REG          PCNF0

/* PCNF2 */
#define POME2_REG            PCNF2
#define PCLKSEL2_REG         PCNF2
#define POP2_REG             PCNF2
#define PMODE20_REG          PCNF2
#define PMODE21_REG          PCNF2
#define PLOCK2_REG           PCNF2
#define PALOCK2_REG          PCNF2
#define PFIFTY2_REG          PCNF2

/* TCNT1L */
#define TCNT1L0_REG          TCNT1L
#define TCNT1L1_REG          TCNT1L
#define TCNT1L2_REG          TCNT1L
#define TCNT1L3_REG          TCNT1L
#define TCNT1L4_REG          TCNT1L
#define TCNT1L5_REG          TCNT1L
#define TCNT1L6_REG          TCNT1L
#define TCNT1L7_REG          TCNT1L

/* PORTD */
#define PORTD0_REG           PORTD
#define PORTD1_REG           PORTD
#define PORTD2_REG           PORTD
#define PORTD3_REG           PORTD
#define PORTD4_REG           PORTD
#define PORTD5_REG           PORTD
#define PORTD6_REG           PORTD
#define PORTD7_REG           PORTD

/* PORTE */
#define PORTE0_REG           PORTE
#define PORTE1_REG           PORTE
#define PORTE2_REG           PORTE

/* TCNT1H */
#define TCNT1H0_REG          TCNT1H
#define TCNT1H1_REG          TCNT1H
#define TCNT1H2_REG          TCNT1H
#define TCNT1H3_REG          TCNT1H
#define TCNT1H4_REG          TCNT1H
#define TCNT1H5_REG          TCNT1H
#define TCNT1H6_REG          TCNT1H
#define TCNT1H7_REG          TCNT1H

/* AMP1CSR */
#define AMP1TS0_REG          AMP1CSR
#define AMP1TS1_REG          AMP1CSR
#define AMP1G0_REG           AMP1CSR
#define AMP1G1_REG           AMP1CSR
#define AMP1IS_REG           AMP1CSR
#define AMP1EN_REG           AMP1CSR

/* AC2CON */
#define AC2M0_REG            AC2CON
#define AC2M1_REG            AC2CON
#define AC2M2_REG            AC2CON
#define AC2IS0_REG           AC2CON
#define AC2IS1_REG           AC2CON
#define AC2IE_REG            AC2CON
#define AC2EN_REG            AC2CON

/* EIMSK */
#define INT0_REG             EIMSK
#define INT1_REG             EIMSK
#define INT2_REG             EIMSK

/* PFRC0A */
#define PRFM0A0_REG          PFRC0A
#define PRFM0A1_REG          PFRC0A
#define PRFM0A2_REG          PFRC0A
#define PRFM0A3_REG          PFRC0A
#define PFLTE0A_REG          PFRC0A
#define PELEV0A_REG          PFRC0A
#define PISEL0A_REG          PFRC0A
#define PCAE0A_REG           PFRC0A

/* PFRC0B */
#define PRFM0B0_REG          PFRC0B
#define PRFM0B1_REG          PFRC0B
#define PRFM0B2_REG          PFRC0B
#define PRFM0B3_REG          PFRC0B
#define PFLTE0B_REG          PFRC0B
#define PELEV0B_REG          PFRC0B
#define PISEL0B_REG          PFRC0B
#define PCAE0B_REG           PFRC0B

/* EICRA */
#define ISC00_REG            EICRA
#define ISC01_REG            EICRA
#define ISC10_REG            EICRA
#define ISC11_REG            EICRA
#define ISC20_REG            EICRA
#define ISC21_REG            EICRA

/* DIDR0 */
#define ADC0D_REG            DIDR0
#define ADC1D_REG            DIDR0
#define ADC2D_REG            DIDR0
#define ADC3D_REG            DIDR0
#define ADC4D_REG            DIDR0
#define ADC5D_REG            DIDR0
#define ADC6D_REG            DIDR0
#define ADC7D_REG            DIDR0

/* DIDR1 */
#define ADC8D_REG            DIDR1
#define ADC9D_REG            DIDR1
#define ADC10D_REG           DIDR1
#define AMP0ND_REG           DIDR1
#define AMP0PD_REG           DIDR1
#define ACMP0D_REG           DIDR1

/* CLKPR */
#define CLKPS0_REG           CLKPR
#define CLKPS1_REG           CLKPR
#define CLKPS2_REG           CLKPR
#define CLKPS3_REG           CLKPR
#define CLKPCE_REG           CLKPR

/* OCR0RBH */
#define OCR0RB_8_REG         OCR0RBH
#define OCR0RB_9_REG         OCR0RBH
#define OCR0RB_00_REG        OCR0RBH
#define OCR0RB_01_REG        OCR0RBH
#define OCR0RB_02_REG        OCR0RBH
#define OCR0RB_03_REG        OCR0RBH
#define OCR0RB_04_REG        OCR0RBH
#define OCR0RB_05_REG        OCR0RBH

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
#define DDB6_REG             DDRB
#define DDB7_REG             DDRB

/* PIM2 */
#define PEOPE2_REG           PIM2
#define PEVE2A_REG           PIM2
#define PEVE2B_REG           PIM2
#define PSEIE2_REG           PIM2

/* TCCR1A */
#define WGM10_REG            TCCR1A
#define WGM11_REG            TCCR1A
#define COM1B0_REG           TCCR1A
#define COM1B1_REG           TCCR1A
#define COM1A0_REG           TCCR1A
#define COM1A1_REG           TCCR1A

/* TCCR1C */
#define FOC1B_REG            TCCR1C
#define FOC1A_REG            TCCR1C

/* TCCR1B */
#define CS10_REG             TCCR1B
#define CS11_REG             TCCR1B
#define CS12_REG             TCCR1B
#define WGM12_REG            TCCR1B
#define WGM13_REG            TCCR1B
#define ICES1_REG            TCCR1B
#define ICNC1_REG            TCCR1B

/* OSCCAL */
#define CAL0_REG             OSCCAL
#define CAL1_REG             OSCCAL
#define CAL2_REG             OSCCAL
#define CAL3_REG             OSCCAL
#define CAL4_REG             OSCCAL
#define CAL5_REG             OSCCAL
#define CAL6_REG             OSCCAL

/* OCR0RAL */
#define OCR0RA_0_REG         OCR0RAL
#define OCR0RA_1_REG         OCR0RAL
#define OCR0RA_2_REG         OCR0RAL
#define OCR0RA_3_REG         OCR0RAL
#define OCR0RA_4_REG         OCR0RAL
#define OCR0RA_5_REG         OCR0RAL
#define OCR0RA_6_REG         OCR0RAL
#define OCR0RA_7_REG         OCR0RAL

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

/* GPIOR3 */
#define GPIOR30_REG          GPIOR3
#define GPIOR31_REG          GPIOR3
#define GPIOR32_REG          GPIOR3
#define GPIOR33_REG          GPIOR3
#define GPIOR34_REG          GPIOR3
#define GPIOR35_REG          GPIOR3
#define GPIOR36_REG          GPIOR3
#define GPIOR37_REG          GPIOR3

/* GPIOR2 */
#define GPIOR20_REG          GPIOR2
#define GPIOR21_REG          GPIOR2
#define GPIOR22_REG          GPIOR2
#define GPIOR23_REG          GPIOR2
#define GPIOR24_REG          GPIOR2
#define GPIOR25_REG          GPIOR2
#define GPIOR26_REG          GPIOR2
#define GPIOR27_REG          GPIOR2

/* PIFR0 */
#define PEOP0_REG            PIFR0
#define PRN00_REG            PIFR0
#define PRN01_REG            PIFR0
#define PEV0A_REG            PIFR0
#define PEV0B_REG            PIFR0
#define PSEI0_REG            PIFR0

/* DDRE */
#define DDE0_REG             DDRE
#define DDE1_REG             DDRE
#define DDE2_REG             DDRE

/* TCNT0 */
#define TCNT0_0_REG          TCNT0
#define TCNT0_1_REG          TCNT0
#define TCNT0_2_REG          TCNT0
#define TCNT0_3_REG          TCNT0
#define TCNT0_4_REG          TCNT0
#define TCNT0_5_REG          TCNT0
#define TCNT0_6_REG          TCNT0
#define TCNT0_7_REG          TCNT0

/* TCCR0B */
#define CS00_REG             TCCR0B
#define CS01_REG             TCCR0B
#define CS02_REG             TCCR0B
#define WGM02_REG            TCCR0B
#define FOC0B_REG            TCCR0B
#define FOC0A_REG            TCCR0B

/* TCCR0A */
#define WGM00_REG            TCCR0A
#define WGM01_REG            TCCR0A
#define COM0B0_REG           TCCR0A
#define COM0B1_REG           TCCR0A
#define COM0A0_REG           TCCR0A
#define COM0A1_REG           TCCR0A

/* PFRC2B */
#define PRFM2B0_REG          PFRC2B
#define PRFM2B1_REG          PFRC2B
#define PRFM2B2_REG          PFRC2B
#define PRFM2B3_REG          PFRC2B
#define PFLTE2B_REG          PFRC2B
#define PELEV2B_REG          PFRC2B
#define PISEL2B_REG          PFRC2B
#define PCAE2B_REG           PFRC2B

/* PFRC2A */
#define PRFM2A0_REG          PFRC2A
#define PRFM2A1_REG          PFRC2A
#define PRFM2A2_REG          PFRC2A
#define PRFM2A3_REG          PFRC2A
#define PFLTE2A_REG          PFRC2A
#define PELEV2A_REG          PFRC2A
#define PISEL2A_REG          PFRC2A
#define PCAE2A_REG           PFRC2A

/* OCR2SAL */
#define OCR2SA_0_REG         OCR2SAL
#define OCR2SA_1_REG         OCR2SAL
#define OCR2SA_2_REG         OCR2SAL
#define OCR2SA_3_REG         OCR2SAL
#define OCR2SA_4_REG         OCR2SAL
#define OCR2SA_5_REG         OCR2SAL
#define OCR2SA_6_REG         OCR2SAL
#define OCR2SA_7_REG         OCR2SAL

/* EUCSRA */
#define URxS0_REG            EUCSRA
#define URxS1_REG            EUCSRA
#define URxS2_REG            EUCSRA
#define URxS3_REG            EUCSRA
#define UTxS0_REG            EUCSRA
#define UTxS1_REG            EUCSRA
#define UTxS2_REG            EUCSRA
#define UTxS3_REG            EUCSRA

/* EUCSRB */
#define BODR_REG             EUCSRB
#define EMCH_REG             EUCSRB
#define EUSBS_REG            EUCSRB
#define EUSART_REG           EUCSRB

/* EUCSRC */
#define STP0_REG             EUCSRC
#define STP1_REG             EUCSRC
#define F1617_REG            EUCSRC
#define FEM_REG              EUCSRC

/* PCTL0 */
#define PRUN0_REG            PCTL0
#define PCCYC0_REG           PCTL0
#define PARUN0_REG           PCTL0
#define PAOC0A_REG           PCTL0
#define PAOC0B_REG           PCTL0
#define PBFM0_REG            PCTL0
#define PPRE00_REG           PCTL0
#define PPRE01_REG           PCTL0

/* PCTL2 */
#define PRUN2_REG            PCTL2
#define PCCYC2_REG           PCTL2
#define PARUN2_REG           PCTL2
#define PAOC2A_REG           PCTL2
#define PAOC2B_REG           PCTL2
#define PBFM2_REG            PCTL2
#define PPRE20_REG           PCTL2
#define PPRE21_REG           PCTL2

/* SPCR */
#define SPR0_REG             SPCR
#define SPR1_REG             SPCR
#define CPHA_REG             SPCR
#define CPOL_REG             SPCR
#define MSTR_REG             SPCR
#define DORD_REG             SPCR
#define SPE_REG              SPCR
#define SPIE_REG             SPCR

/* TIFR1 */
#define TOV1_REG             TIFR1
#define OCF1A_REG            TIFR1
#define OCF1B_REG            TIFR1
#define ICF1_REG             TIFR1

/* GTCCR */
#define PSR10_REG            GTCCR
#define ICPSEL1_REG          GTCCR
#define TSM_REG              GTCCR
#define PSRSYNC_REG          GTCCR

/* ICR1H */
#define ICR1H0_REG           ICR1H
#define ICR1H1_REG           ICR1H
#define ICR1H2_REG           ICR1H
#define ICR1H3_REG           ICR1H
#define ICR1H4_REG           ICR1H
#define ICR1H5_REG           ICR1H
#define ICR1H6_REG           ICR1H
#define ICR1H7_REG           ICR1H

/* POM2 */
#define POMV2A0_REG          POM2
#define POMV2A1_REG          POM2
#define POMV2A2_REG          POM2
#define POMV2A3_REG          POM2
#define POMV2B0_REG          POM2
#define POMV2B1_REG          POM2
#define POMV2B2_REG          POM2
#define POMV2B3_REG          POM2

/* OCR2RBL */
#define OCR2RB_0_REG         OCR2RBL
#define OCR2RB_1_REG         OCR2RBL
#define OCR2RB_2_REG         OCR2RBL
#define OCR2RB_3_REG         OCR2RBL
#define OCR2RB_4_REG         OCR2RBL
#define OCR2RB_5_REG         OCR2RBL
#define OCR2RB_6_REG         OCR2RBL
#define OCR2RB_7_REG         OCR2RBL

/* PICR2H */
#define PICR2_8_REG          PICR2H
#define PICR2_9_REG          PICR2H
#define PICR2_10_REG         PICR2H
#define PICR2_11_REG         PICR2H

/* OCR2RBH */
#define OCR2RB_8_REG         OCR2RBH
#define OCR2RB_9_REG         OCR2RBH
#define OCR2RB_10_REG        OCR2RBH
#define OCR2RB_11_REG        OCR2RBH
#define OCR2RB_12_REG        OCR2RBH
#define OCR2RB_13_REG        OCR2RBH
#define OCR2RB_14_REG        OCR2RBH
#define OCR2RB_15_REG        OCR2RBH

/* PICR2L */
#define PICR2_0_REG          PICR2L
#define PICR2_1_REG          PICR2L
#define PICR2_2_REG          PICR2L
#define PICR2_3_REG          PICR2L
#define PICR2_4_REG          PICR2L
#define PICR2_5_REG          PICR2L
#define PICR2_6_REG          PICR2L
#define PICR2_7_REG          PICR2L

/* OCR1BL */
#define OCR1BL0_REG          OCR1BL
#define OCR1BL1_REG          OCR1BL
#define OCR1BL2_REG          OCR1BL
#define OCR1BL3_REG          OCR1BL
#define OCR1BL4_REG          OCR1BL
#define OCR1BL5_REG          OCR1BL
#define OCR1BL6_REG          OCR1BL
#define OCR1BL7_REG          OCR1BL

/* OCR1BH */
#define OCR1BH0_REG          OCR1BH
#define OCR1BH1_REG          OCR1BH
#define OCR1BH2_REG          OCR1BH
#define OCR1BH3_REG          OCR1BH
#define OCR1BH4_REG          OCR1BH
#define OCR1BH5_REG          OCR1BH
#define OCR1BH6_REG          OCR1BH
#define OCR1BH7_REG          OCR1BH

/* ICR1L */
#define ICR1L0_REG           ICR1L
#define ICR1L1_REG           ICR1L
#define ICR1L2_REG           ICR1L
#define ICR1L3_REG           ICR1L
#define ICR1L4_REG           ICR1L
#define ICR1L5_REG           ICR1L
#define ICR1L6_REG           ICR1L
#define ICR1L7_REG           ICR1L

/* MCUSR */
#define PORF_REG             MCUSR
#define EXTRF_REG            MCUSR
#define BORF_REG             MCUSR
#define WDRF_REG             MCUSR

/* EECR */
#define EERE_REG             EECR
#define EEWE_REG             EECR
#define EEMWE_REG            EECR
#define EERIE_REG            EECR

/* SMCR */
#define SE_REG               SMCR
#define SM0_REG              SMCR
#define SM1_REG              SMCR
#define SM2_REG              SMCR

/* PLLCSR */
#define PLOCK_REG            PLLCSR
#define PLLE_REG             PLLCSR
#define PLLF_REG             PLLCSR

/* OCR2RAH */
#define OCR2RA_8_REG         OCR2RAH
#define OCR2RA_9_REG         OCR2RAH
#define OCR2RA_10_REG        OCR2RAH
#define OCR2RA_11_REG        OCR2RAH

/* OCR2RAL */
#define OCR2RA_0_REG         OCR2RAL
#define OCR2RA_1_REG         OCR2RAL
#define OCR2RA_2_REG         OCR2RAL
#define OCR2RA_3_REG         OCR2RAL
#define OCR2RA_4_REG         OCR2RAL
#define OCR2RA_5_REG         OCR2RAL
#define OCR2RA_6_REG         OCR2RAL
#define OCR2RA_7_REG         OCR2RAL

/* OCR0RBL */
#define OCR0RB_0_REG         OCR0RBL
#define OCR0RB_1_REG         OCR0RBL
#define OCR0RB_2_REG         OCR0RBL
#define OCR0RB_3_REG         OCR0RBL
#define OCR0RB_4_REG         OCR0RBL
#define OCR0RB_5_REG         OCR0RBL
#define OCR0RB_6_REG         OCR0RBL
#define OCR0RB_7_REG         OCR0RBL

/* OCR0SAH */
#define OCR0SA_8_REG         OCR0SAH
#define OCR0SA_9_REG         OCR0SAH
#define OCR0SA_00_REG        OCR0SAH
#define OCR0SA_01_REG        OCR0SAH

/* EEARH */
#define EEAR8_REG            EEARH
#define EEAR9_REG            EEARH
#define EEAR10_REG           EEARH
#define EEAR11_REG           EEARH

/* EEARL */
#define EEARL0_REG           EEARL
#define EEARL1_REG           EEARL
#define EEARL2_REG           EEARL
#define EEARL3_REG           EEARL
#define EEARL4_REG           EEARL
#define EEARL5_REG           EEARL
#define EEARL6_REG           EEARL
#define EEARL7_REG           EEARL

/* MCUCR */
#define IVCE_REG             MCUCR
#define IVSEL_REG            MCUCR
#define PUD_REG              MCUCR
#define SPIPS_REG            MCUCR

/* PICR0H */
#define PICR0_8_REG          PICR0H
#define PICR0_9_REG          PICR0H
#define PICR0_10_REG         PICR0H
#define PICR0_11_REG         PICR0H

/* EIFR */
#define INTF0_REG            EIFR
#define INTF1_REG            EIFR
#define INTF2_REG            EIFR

/* MUBRRL */
#define MUBRR0_REG           MUBRRL
#define MUBRR1_REG           MUBRRL
#define MUBRR2_REG           MUBRRL
#define MUBRR3_REG           MUBRRL
#define MUBRR4_REG           MUBRRL
#define MUBRR5_REG           MUBRRL
#define MUBRR6_REG           MUBRRL
#define MUBRR7_REG           MUBRRL

/* MUBRRH */
#define MUBRR8_REG           MUBRRH
#define MUBRR9_REG           MUBRRH
#define MUBRR10_REG          MUBRRH
#define MUBRR11_REG          MUBRRH
#define MUBRR12_REG          MUBRRH
#define MUBRR13_REG          MUBRRH
#define MUBRR14_REG          MUBRRH
#define MUBRR15_REG          MUBRRH

/* OCR2SAH */
#define OCR2SA_8_REG         OCR2SAH
#define OCR2SA_9_REG         OCR2SAH
#define OCR2SA_10_REG        OCR2SAH
#define OCR2SA_11_REG        OCR2SAH

/* OCR0SBL */
#define OCR0SB_0_REG         OCR0SBL
#define OCR0SB_1_REG         OCR0SBL
#define OCR0SB_2_REG         OCR0SBL
#define OCR0SB_3_REG         OCR0SBL
#define OCR0SB_4_REG         OCR0SBL
#define OCR0SB_5_REG         OCR0SBL
#define OCR0SB_6_REG         OCR0SBL
#define OCR0SB_7_REG         OCR0SBL

/* OCR0SBH */
#define OCR0SB_8_REG         OCR0SBH
#define OCR0SB_9_REG         OCR0SBH
#define OCR0SB_00_REG        OCR0SBH
#define OCR0SB_01_REG        OCR0SBH

/* PICR0L */
#define PICR0_0_REG          PICR0L
#define PICR0_1_REG          PICR0L
#define PICR0_2_REG          PICR0L
#define PICR0_3_REG          PICR0L
#define PICR0_4_REG          PICR0L
#define PICR0_5_REG          PICR0L
#define PICR0_6_REG          PICR0L
#define PICR0_7_REG          PICR0L

/* ADCSRA */
#define ADPS0_REG            ADCSRA
#define ADPS1_REG            ADCSRA
#define ADPS2_REG            ADCSRA
#define ADIE_REG             ADCSRA
#define ADIF_REG             ADCSRA
#define ADATE_REG            ADCSRA
#define ADSC_REG             ADCSRA
#define ADEN_REG             ADCSRA

/* PSOC0 */
#define POEN0A_REG           PSOC0
#define POEN0B_REG           PSOC0
#define PSYNC00_REG          PSOC0
#define PSYNC01_REG          PSOC0

/* ADCSRB */
#define ADTS0_REG            ADCSRB
#define ADTS1_REG            ADCSRB
#define ADTS2_REG            ADCSRB
#define ADTS3_REG            ADCSRB
#define ADASCR_REG           ADCSRB
#define ADHSM_REG            ADCSRB

/* OCR0A */
/* #define OCR0_0_REG           OCR0A */ /* dup in OCR0B */
/* #define OCR0_1_REG           OCR0A */ /* dup in OCR0B */
/* #define OCR0_2_REG           OCR0A */ /* dup in OCR0B */
/* #define OCR0_3_REG           OCR0A */ /* dup in OCR0B */
/* #define OCR0_4_REG           OCR0A */ /* dup in OCR0B */
/* #define OCR0_5_REG           OCR0A */ /* dup in OCR0B */
/* #define OCR0_6_REG           OCR0A */ /* dup in OCR0B */
/* #define OCR0_7_REG           OCR0A */ /* dup in OCR0B */

/* ACSR */
#define AC0O_REG             ACSR
#define AC1O_REG             ACSR
#define AC2O_REG             ACSR
#define AC0IF_REG            ACSR
#define AC1IF_REG            ACSR
#define AC2IF_REG            ACSR
#define ACCKDIV_REG          ACSR

/* DDRD */
#define DDD0_REG             DDRD
#define DDD1_REG             DDRD
#define DDD2_REG             DDRD
#define DDD3_REG             DDRD
#define DDD4_REG             DDRD
#define DDD5_REG             DDRD
#define DDD6_REG             DDRD
#define DDD7_REG             DDRD

/* UBRRH */
#define UBRR8_REG            UBRRH
#define UBRR9_REG            UBRRH
#define UBRR10_REG           UBRRH
#define UBRR11_REG           UBRRH

/* DACL */
#define DACL0_REG            DACL
#define DACL1_REG            DACL
#define DACL2_REG            DACL
#define DACL3_REG            DACL
#define DACL4_REG            DACL
#define DACL5_REG            DACL
#define DACL6_REG            DACL
#define DACL7_REG            DACL

/* UBRRL */
#define UBRR0_REG            UBRRL
#define UBRR1_REG            UBRRL
#define UBRR2_REG            UBRRL
#define UBRR3_REG            UBRRL
#define UBRR4_REG            UBRRL
#define UBRR5_REG            UBRRL
#define UBRR6_REG            UBRRL
#define UBRR7_REG            UBRRL

/* DACH */
#define DACH0_REG            DACH
#define DACH1_REG            DACH
#define DACH2_REG            DACH
#define DACH3_REG            DACH
#define DACH4_REG            DACH
#define DACH5_REG            DACH
#define DACH6_REG            DACH
#define DACH7_REG            DACH

/* OCR0RAH */
#define OCR0RA_8_REG         OCR0RAH
#define OCR0RA_9_REG         OCR0RAH
#define OCR0RA_00_REG        OCR0RAH
#define OCR0RA_01_REG        OCR0RAH

/* SPMCSR */
#define SPMEN_REG            SPMCSR
#define PGERS_REG            SPMCSR
#define PGWRT_REG            SPMCSR
#define BLBSET_REG           SPMCSR
#define RWWSRE_REG           SPMCSR
#define RWWSB_REG            SPMCSR
#define SPMIE_REG            SPMCSR

/* PIM0 */
#define PEOPE0_REG           PIM0
#define PEVE0A_REG           PIM0
#define PEVE0B_REG           PIM0
#define PSEIE0_REG           PIM0

/* PIFR2 */
#define PEOP2_REG            PIFR2
#define PRN20_REG            PIFR2
#define PRN21_REG            PIFR2
#define PEV2A_REG            PIFR2
#define PEV2B_REG            PIFR2
#define PSEI2_REG            PIFR2

/* PORTB */
#define PORTB0_REG           PORTB
#define PORTB1_REG           PORTB
#define PORTB2_REG           PORTB
#define PORTB3_REG           PORTB
#define PORTB4_REG           PORTB
#define PORTB5_REG           PORTB
#define PORTB6_REG           PORTB
#define PORTB7_REG           PORTB

/* ADCL */
#define ADCL0_REG            ADCL
#define ADCL1_REG            ADCL
#define ADCL2_REG            ADCL
#define ADCL3_REG            ADCL
#define ADCL4_REG            ADCL
#define ADCL5_REG            ADCL
#define ADCL6_REG            ADCL
#define ADCL7_REG            ADCL

/* ADCH */
#define ADCH0_REG            ADCH
#define ADCH1_REG            ADCH
#define ADCH2_REG            ADCH
#define ADCH3_REG            ADCH
#define ADCH4_REG            ADCH
#define ADCH5_REG            ADCH
#define ADCH6_REG            ADCH
#define ADCH7_REG            ADCH

/* PSOC2 */
#define POEN2A_REG           PSOC2
#define POEN2C_REG           PSOC2
#define POEN2B_REG           PSOC2
#define POEN2D_REG           PSOC2
#define PSYNC2_0_REG         PSOC2
#define PSYNC2_1_REG         PSOC2
#define POS22_REG            PSOC2
#define POS23_REG            PSOC2

/* TIMSK0 */
#define TOIE0_REG            TIMSK0
#define OCIE0A_REG           TIMSK0
#define OCIE0B_REG           TIMSK0

/* TIMSK1 */
#define TOIE1_REG            TIMSK1
#define OCIE1A_REG           TIMSK1
#define OCIE1B_REG           TIMSK1
#define ICIE1_REG            TIMSK1

/* AMP0CSR */
#define AMP0TS0_REG          AMP0CSR
#define AMP0TS1_REG          AMP0CSR
#define AMP0G0_REG           AMP0CSR
#define AMP0G1_REG           AMP0CSR
#define AMP0IS_REG           AMP0CSR
#define AMP0EN_REG           AMP0CSR

/* UDR */
#define UDR0_REG             UDR
#define UDR1_REG             UDR
#define UDR2_REG             UDR
#define UDR3_REG             UDR
#define UDR4_REG             UDR
#define UDR5_REG             UDR
#define UDR6_REG             UDR
#define UDR7_REG             UDR

/* DACON */
#define DAEN_REG             DACON
#define DALA_REG             DACON
#define DATS0_REG            DACON
#define DATS1_REG            DACON
#define DATS2_REG            DACON
#define DAATE_REG            DACON

/* PINB */
#define PINB0_REG            PINB
#define PINB1_REG            PINB
#define PINB2_REG            PINB
#define PINB3_REG            PINB
#define PINB4_REG            PINB
#define PINB5_REG            PINB
#define PINB6_REG            PINB
#define PINB7_REG            PINB

/* AC0CON */
#define AC0M0_REG            AC0CON
#define AC0M1_REG            AC0CON
#define AC0M2_REG            AC0CON
#define AC0IS0_REG           AC0CON
#define AC0IS1_REG           AC0CON
#define AC0IE_REG            AC0CON
#define AC0EN_REG            AC0CON

/* PINE */
#define PINE0_REG            PINE
#define PINE1_REG            PINE
#define PINE2_REG            PINE

/* PIND */
#define PIND0_REG            PIND
#define PIND1_REG            PIND
#define PIND2_REG            PIND
#define PIND3_REG            PIND
#define PIND4_REG            PIND
#define PIND5_REG            PIND
#define PIND6_REG            PIND
#define PIND7_REG            PIND

/* OCR1AH */
#define OCR1AH0_REG          OCR1AH
#define OCR1AH1_REG          OCR1AH
#define OCR1AH2_REG          OCR1AH
#define OCR1AH3_REG          OCR1AH
#define OCR1AH4_REG          OCR1AH
#define OCR1AH5_REG          OCR1AH
#define OCR1AH6_REG          OCR1AH
#define OCR1AH7_REG          OCR1AH

/* OCR1AL */
#define OCR1AL0_REG          OCR1AL
#define OCR1AL1_REG          OCR1AL
#define OCR1AL2_REG          OCR1AL
#define OCR1AL3_REG          OCR1AL
#define OCR1AL4_REG          OCR1AL
#define OCR1AL5_REG          OCR1AL
#define OCR1AL6_REG          OCR1AL
#define OCR1AL7_REG          OCR1AL

/* TIFR0 */
#define TOV0_REG             TIFR0
#define OCF0A_REG            TIFR0
#define OCF0B_REG            TIFR0

/* pins mapping */
#define MISO_PORT PORTB
#define MISO_BIT 0
#define PSCOUT20_PORT PORTB
#define PSCOUT20_BIT 0

#define MOSI_PORT PORTB
#define MOSI_BIT 1
#define PSCOUT21_PORT PORTB
#define PSCOUT21_BIT 1

#define ADC5_PORT PORTB
#define ADC5_BIT 2
#define INT1_PORT PORTB
#define INT1_BIT 2

#define AMP0-_PORT PORTB
#define AMP0-_BIT 3

#define AMP0+_PORT PORTB
#define AMP0+_BIT 4

#define ADC6_PORT PORTB
#define ADC6_BIT 5
#define INT2_PORT PORTB
#define INT2_BIT 5

#define ADC7_PORT PORTB
#define ADC7_BIT 6
#define PSCOUT11_PORT PORTB
#define PSCOUT11_BIT 6
#define ICP1B_PORT PORTB
#define ICP1B_BIT 6

#define ADC4_PORT PORTB
#define ADC4_BIT 7
#define PSCOUT01_PORT PORTB
#define PSCOUT01_BIT 7
#define SCK_PORT PORTB
#define SCK_BIT 7

#define PSCOUT00_PORT PORTD
#define PSCOUT00_BIT 0
#define XCK_PORT PORTD
#define XCK_BIT 0
#define SSA_PORT PORTD
#define SSA_BIT 0

#define PSCIN0_PORT PORTD
#define PSCIN0_BIT 1
#define CLK0_PORT PORTD
#define CLK0_BIT 1

#define PSCIN2_PORT PORTD
#define PSCIN2_BIT 2
#define OC1A_PORT PORTD
#define OC1A_BIT 2
#define MISO_A_PORT PORTD
#define MISO_A_BIT 2

#define TXD_PORT PORTD
#define TXD_BIT 3
#define DALI_PORT PORTD
#define DALI_BIT 3
#define OC0A_PORT PORTD
#define OC0A_BIT 3
#define SS_PORT PORTD
#define SS_BIT 3
#define MOSI_A_PORT PORTD
#define MOSI_A_BIT 3

#define ADC1_PORT PORTD
#define ADC1_BIT 4
#define RXD_PORT PORTD
#define RXD_BIT 4
#define DALI_PORT PORTD
#define DALI_BIT 4
#define ICP1_PORT PORTD
#define ICP1_BIT 4
#define SCK_A_PORT PORTD
#define SCK_A_BIT 4

#define ADC2_PORT PORTD
#define ADC2_BIT 5
#define ACMP2_PORT PORTD
#define ACMP2_BIT 5

#define ADC3_PORT PORTD
#define ADC3_BIT 6
#define ACMPM_PORT PORTD
#define ACMPM_BIT 6
#define INT0_PORT PORTD
#define INT0_BIT 6

#define ACMP0_PORT PORTD
#define ACMP0_BIT 7

#define RESET_PORT PORTE
#define RESET_BIT 0
#define OCD_PORT PORTE
#define OCD_BIT 0

#define OC0B_PORT PORTE
#define OC0B_BIT 1
#define XTAL1_PORT PORTE
#define XTAL1_BIT 1

#define ADC0_PORT PORTE
#define ADC0_BIT 2
#define XTAL2_PORT PORTE
#define XTAL2_BIT 2


