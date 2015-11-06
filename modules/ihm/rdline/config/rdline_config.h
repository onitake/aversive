/*  
 *  Copyright Gregor Riepl (2015)
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
 *  Revision : $Id: uart_config.h 2015-11-06 16:57:00 onitake Exp $
 *
 */

#ifndef RDLINE_CONFIG_H
#define RDLINE_CONFIG_H

/* Line buffer size, set this to the maximum number of characters per command line */
#define RDLINE_BUF_SIZE 64
/* Length of the command prompt buffer, the command prompt will be truncated if it is longer than this */
#define RDLINE_PROMPT_SIZE 16
/* Size of the VT100 code buffer - should be left at 8 */
#define RDLINE_VT100_BUF_SIZE 8
/* Total size of the history in characters */
#define RDLINE_HISTORY_BUF_SIZE 128
/* Maximum line length of a single history entry - should equal to RDLINE_BUF_SIZE */
#define RDLINE_HISTORY_MAX_LINE RDLINE_BUF_SIZE

#endif /*RDLINE_CONFIG_H*/
