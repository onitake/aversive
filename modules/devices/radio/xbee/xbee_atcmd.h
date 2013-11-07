/*
 * Copyright (c) 2011, Olivier MATZ <zer0@droids-corp.org>
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of California, Berkeley nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE REGENTS AND CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _XBEE_ATCMD_H_
#define _XBEE_ATCMD_H_

#define XBEE_ATCMD_F_READ              0x001
#define XBEE_ATCMD_F_WRITE             0x002
#define XBEE_ATCMD_F_PARAM_NONE        0x004
#define XBEE_ATCMD_F_PARAM_U8          0x008
#define XBEE_ATCMD_F_PARAM_U16         0x010
#define XBEE_ATCMD_F_PARAM_S16         0x020
#define XBEE_ATCMD_F_PARAM_U32         0x040
#define XBEE_ATCMD_F_PARAM_STRING_20B  0x080
#define XBEE_ATCMD_F_PARAM_HEXBUF_16B  0x100

/* list of xbee at commands */
struct xbee_atcmd {
	PGM_P name;
	PGM_P desc;
	unsigned int flags;
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
	PGM_P help;
#endif
};

extern const struct xbee_atcmd PROGMEM xbee_atcmd_list[];

const struct xbee_atcmd *xbee_atcmd_lookup_name(const char *atcmd_str);
const struct xbee_atcmd *xbee_atcmd_lookup_desc(const char *desc);

#endif /* _xBEE_ATCMD_H_ */
