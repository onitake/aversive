/*  
 *  Copyright Droids Corporation (2007)
 *  Olivier MATZ <zer0@droids-corp.org>
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
 *  Revision : $Id: vt100.c,v 1.1.2.1 2008-01-05 22:46:28 zer0 Exp $
 *
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <ctype.h>

#include <aversive/pgmspace.h>

#include "vt100.h"

static const char PROGMEM cmd0[] = vt100_up_arr;
static const char PROGMEM cmd1[] = vt100_down_arr;
static const char PROGMEM cmd2[] = vt100_right_arr;
static const char PROGMEM cmd3[] = vt100_left_arr;
static const char PROGMEM cmd4[] = "\177";
static const char PROGMEM cmd5[] = "\n";
static const char PROGMEM cmd6[] = "\001";
static const char PROGMEM cmd7[] = "\005";
static const char PROGMEM cmd8[] = "\013";
static const char PROGMEM cmd9[] = "\031";
static const char PROGMEM cmd10[] = "\003";
static const char PROGMEM cmd11[] = "\006";
static const char PROGMEM cmd12[] = "\002";
static const char PROGMEM cmd13[] = vt100_suppr;
static const char PROGMEM cmd14[] = vt100_tab;
static const char PROGMEM cmd15[] = "\004";
static const char PROGMEM cmd16[] = "\014";
static const char PROGMEM cmd17[] = "\r";
static const char PROGMEM cmd18[] = "\033\177";
static const char PROGMEM cmd19[] = vt100_word_left;
static const char PROGMEM cmd20[] = vt100_word_right;
static const char PROGMEM cmd21[] = "?";

const char * const PROGMEM vt100_commands[] = {
	cmd0, cmd1, cmd2, cmd3, cmd4, cmd5, cmd6, cmd7,
	cmd8, cmd9, cmd10, cmd11, cmd12, cmd13, cmd14,
	cmd15, cmd16, cmd17, cmd18, cmd19, cmd20,
	cmd21,
};

void
vt100_init(struct vt100 * vt)
{
	vt->state = VT100_INIT;
}


static int8_t
match_command(char * buf, uint8_t size)
{
	const char *cmd;
	uint8_t i = 0;
	
	for (i=0 ; i<sizeof(vt100_commands)/sizeof(const char *) ; i++) {
#ifdef HOST_VERSION
		cmd = *(vt100_commands + i);
#else
		cmd = (const char *) pgm_read_word (vt100_commands + i);
#endif

		if (size == strlen_P(cmd) &&
		    !strncmp_P(buf, cmd, strlen_P(cmd))) {
			return i;
		}
	}

	return -1;
}

int8_t
vt100_parser(struct vt100 *vt, char ch)
{
	uint8_t size;
	uint8_t c = (uint8_t) ch;

	if (vt->bufpos > VT100_BUF_SIZE) {
		vt->state = VT100_INIT;
		vt->bufpos = 0;
	}

	vt->buf[vt->bufpos++] = c;
	size = vt->bufpos;

	switch (vt->state) {
	case VT100_INIT:
		if (c == 033) {
			vt->state = VT100_ESCAPE;
		}
		else {
			vt->bufpos = 0;
			goto match_command;
		}
		break;

	case VT100_ESCAPE:
		if (c == 0133) {
			vt->state = VT100_ESCAPE_CSI;
		}
		else if (c >= 060 && c <= 0177) { /* XXX 0177 ? */
			vt->bufpos = 0;
			vt->state = VT100_INIT;
			goto match_command;
		}
		break;

	case VT100_ESCAPE_CSI:
		if (c >= 0100 && c <= 0176) {
			vt->bufpos = 0;
			vt->state = VT100_INIT;
			goto match_command;
		}
		break;
		
	default:
		vt->bufpos = 0;
		break;
	}

	return -2;
	
 match_command:
	return match_command(vt->buf, size);
}
