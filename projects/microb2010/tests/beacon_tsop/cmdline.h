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
 *  Revision : $Id: cmdline.h,v 1.2 2009-02-27 22:23:37 zer0 Exp $
 *
 */

#define CMDLINE_UART 0

int cmdline_process(void);

static inline uint8_t cmdline_keypressed(void) {
	return (uart_recv_nowait(CMDLINE_UART) != -1);
}

static inline int16_t cmdline_getchar(void) {
	return uart_recv_nowait(CMDLINE_UART);
}

static inline uint8_t cmdline_getchar_wait(void) {
	return uart_recv(CMDLINE_UART);
}

void write_char(char c);
void valid_buffer(const char *buf, uint8_t size);
int8_t complete_buffer(const char *buf, char *dstbuf, uint8_t dstsize,
		       int16_t *state);

