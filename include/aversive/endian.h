/*
 *  Copyright Droids Corporation (2011)
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
 *  Revision : $Id: md5c.c,v 1.3.4.1 2006-11-26 21:06:02 zer0 Exp $
 *
 */

#ifndef _AVERSIVE_ENDIAN_H_
#define _AVERSIVE_ENDIAN_H_

static inline uint16_t bswap16(uint16_t x)
{
	return (uint16_t)(((x & 0x00ffU) << 8) |
			  ((x & 0xff00U) >> 8));
}

static inline uint32_t bswap32(uint32_t x)
{
	return  ((x & 0x000000ffUL) << 24) |
		((x & 0x0000ff00UL) << 8) |
		((x & 0x00ff0000UL) >> 8) |
		((x & 0xff000000UL) >> 24);
}

static inline uint64_t bswap64(uint64_t x)
{
	return  ((x & 0x00000000000000ffULL) << 56) |
		((x & 0x000000000000ff00ULL) << 40) |
		((x & 0x0000000000ff0000ULL) << 24) |
		((x & 0x00000000ff000000ULL) <<  8) |
		((x & 0x000000ff00000000ULL) >>  8) |
		((x & 0x0000ff0000000000ULL) >> 24) |
		((x & 0x00ff000000000000ULL) >> 40) |
		((x & 0xff00000000000000ULL) >> 56);
}

#if BYTE_ORDER == LITTLE_ENDIAN
#define ntohs(x) bswap16(x)
#define ntohl(x) bswap32(x)
#define ntohll(x) bswap64(x)
#else
#define ntohs(x) (x)
#define ntohl(x) (x)
#define ntohll(x) (x)
#endif

#define htons ntohs
#define htonl ntohl
#define htonll ntohll

#endif
