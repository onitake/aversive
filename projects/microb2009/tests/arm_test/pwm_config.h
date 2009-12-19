/*  
 *  Copyright Droids Corporation, Microb Technology, Eirbot (2005)
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
 *  Revision : $Id: pwm_config.h,v 1.2 2009-01-23 23:10:25 zer0 Exp $
 *
 */

/* Droids-corp, Eirbot, Microb Technology 2005 - Zer0
 * Config for PWM
 */

#ifndef _PWM_CONFIG_
#define _PWM_CONFIG_

#define _PWM_CONFIG_VERSION_ 2

/* Which PWM are enabled ? */
#define PWM1A_ENABLED
#define PWM1B_ENABLED
/* #define PWM1C_ENABLED */
/* #define PWM3A_ENABLED */
/* #define PWM3B_ENABLED */
#define PWM3C_ENABLED

/** max value for PWM entry, default 12 bits > 4095 */
#define PWM_SIGNIFICANT_BITS 12

#define TIMER1_MODE     TIMER_16_MODE_PWM_10
#define TIMER1_PRESCALE TIMER1_PRESCALER_DIV_1

#define TIMER3_MODE     TIMER_16_MODE_PWM_10
#define TIMER3_PRESCALE TIMER3_PRESCALER_DIV_1


#define PWM1A_MODE       (PWM_SIGNED // | PWM_SIGN_INVERTED)
#define PWM1A_SIGN_PORT  PORTB
#define PWM1A_SIGN_BIT   0

#define PWM1B_MODE       (PWM_SIGNED)
#define PWM1B_SIGN_PORT  PORTB
#define PWM1B_SIGN_BIT   1

#define PWM3C_MODE       (PWM_SIGNED) /* left */
#define PWM3C_SIGN_PORT  PORTE
#define PWM3C_SIGN_BIT   4

/* #define PWM2_MODE       (PWM_SIGNED) */
/* #define PWM2_SIGN_PORT  PORTB */
/* #define PWM2_SIGN_BIT   2 */

#endif

