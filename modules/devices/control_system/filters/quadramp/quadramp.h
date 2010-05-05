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
 *  Revision : $Id: quadramp.h,v 1.3.4.4 2009-05-18 12:29:51 zer0 Exp $
 *
 */

#ifndef _QUADRAMP_H_
#define _QUADRAMP_H_

#include <aversive.h>

struct quadramp_filter
{
	double var_2nd_ord_pos;
	double var_2nd_ord_neg;
	double var_1st_ord_pos;
	double var_1st_ord_neg;

	double previous_var;
	double previous_out;
	int32_t previous_in;
};

/** Initialization of the filter */
void quadramp_init(struct quadramp_filter *q);

void quadramp_reset(struct quadramp_filter *q);

void quadramp_set_2nd_order_vars(struct quadramp_filter *q,
				 double var_2nd_ord_pos,
				 double var_2nd_ord_neg);

void quadramp_set_1st_order_vars(struct quadramp_filter *q,
				 double var_1st_ord_pos,
				 double var_1st_ord_neg);

/**
 * Return 1 when (filter_input == filter_output && 1st_ord variation
 * is 0 --speed is 0-- ).
 */
uint8_t quadramp_is_finished(struct quadramp_filter *q);

/**
 * Process the ramp
 *
 * \param data should be a (struct quadramp_filter *) pointer
 * \param in is the input of the filter
 */
int32_t quadramp_do_filter(void *data, int32_t in);

#endif
