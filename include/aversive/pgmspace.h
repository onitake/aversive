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
 *  Revision : $Id: pgmspace.h,v 1.1.2.4 2007-11-21 21:54:38 zer0 Exp $
 *
 */

/**
 * This file is used for compatibility between host and avr : with
 * this we can emulate pgmspace on a host.
 */

#ifndef _AVERSIVE_PGMSPACE_H_
#define _AVERSIVE_PGMSPACE_H_

#ifndef HOST_VERSION

#include <avr/pgmspace.h>
#define PGMS_FMT "%S"


#else

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>

typedef void prog_void;
typedef char prog_char;
typedef unsigned char prog_uchar;
typedef int8_t prog_int8_t;
typedef uint8_t prog_uint8_t;
typedef int16_t prog_int16_t;
typedef uint16_t prog_uint16_t;
typedef int32_t prog_int32_t;
typedef uint32_t prog_uint32_t;
typedef int64_t prog_int64_t;


static inline int memcmp_P(const void *s1,
			   const prog_void *s2, unsigned n)
{
	return memcmp(s1, s2, n);
}

static inline void *memcpy_P(void *s1,
			     const prog_void *s2, unsigned n)
{
	return memcpy(s1, s2, n);
}

static inline char *strcat_P(char *s1, const prog_char *s2)
{
	return strcat(s1, s2);
}

static inline char *strcpy_P(char *s1, const prog_char *s2)
{
	return strcpy(s1, s2);
}

static inline char *strncpy_P(char *s1, const prog_char *s2,
			      unsigned n)
{
	return strncpy(s1, s2, n);
}

static inline int strcmp_P(const char *s1, const prog_char *s2)
{
	return strcmp(s1, s2);
}

static inline int strncmp_P(const char *s1, const prog_char *s2,
			    unsigned n)
{
	return strncmp(s1, s2, n);
}

static inline unsigned strlen_P(const prog_char *s)
{
	return strlen(s);
}

static inline int vfprintf_P(FILE *stream,
			     const prog_char *s, va_list ap)
{
	return vfprintf(stream, s, ap);
}

static inline int vsprintf_P(char *buf, const prog_char *s,
			     va_list ap)
{
	return vsprintf(buf, s, ap);
}

#define PGM_P const char *
#define PSTR(x) x
#define PROGMEM
#define printf_P(args...) printf(args)
#define sprintf_P(buf, args...) sprintf(buf, args)
#define snprintf_P(buf, n, args...) snprintf(buf, n, args)

static inline uint32_t pgm_read_dword(const prog_void *x)
{
	return *(uint32_t *)x;
}

static inline uint16_t pgm_read_word(const prog_void *x)
{
	return *(uint16_t *)x;
}

static inline uint8_t pgm_read_byte(const prog_void *x)
{
	return *(uint8_t *)x;
}

#define PGMS_FMT "%s"

#endif /* HOST_VERSION */
#endif /* _AVERSIVE_PGMSPACE_H_ */


