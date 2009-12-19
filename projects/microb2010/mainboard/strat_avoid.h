/*  
 *  Copyright Droids Corporation, Microb Technology (2009)
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
 *  Revision : $Id: strat_avoid.h,v 1.5 2009-11-08 17:24:33 zer0 Exp $
 *
 */

void set_opponent_poly(poly_t *pol, int16_t w, int16_t l);
int8_t goto_and_avoid(int16_t x, int16_t y, uint8_t flags_intermediate,
		      uint8_t flags_final);
int8_t goto_and_avoid_backward(int16_t x, int16_t y,
			       uint8_t flags_intermediate,
			       uint8_t flags_final);
int8_t goto_and_avoid_forward(int16_t x, int16_t y,
			      uint8_t flags_intermediate,
			      uint8_t flags_final);
