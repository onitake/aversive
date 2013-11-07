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

#ifndef _XBEE_H_
#define _XBEE_H_

#include "xbee_neighbor.h"
#include "xbee_atcmd.h"
#include "xbee_stats.h"
#include "xbee_rxtx.h"

/* Callback when receiving data on a specific channel. The arguments of the
 * function are the xbee device, the channel ID, the type of the frame (example:
 * XBEE_TYPE_ATRESP), the pointer to the frame, the length of the frame, and an
 * opaque pointer (reserved for user). The given length exludes the xbee_hdr
 * structure (delimiter, len, type, id) and the cksum.  */
typedef int8_t (xbee_rx_cb_t)(struct xbee_dev *dev, int channel, int type,
	void *frame, unsigned len, void *opaque);

/* an xbee queue */
struct xbee_channel {
	int registered;
	xbee_rx_cb_t *rx_cb;
	void *arg;
};

#define XBEE_DEFAULT_CHANNEL 0
#define XBEE_MAX_CHANNEL 16
#define XBEE_CHANNEL_ANY XBEE_MAX_CHANNEL

/* structure identifying a xbee device */
struct xbee_dev {
	FILE *file;
	struct xbee_channel channel[XBEE_MAX_CHANNEL];
	uint8_t frame_len;
	char frame[XBEE_MAX_FRAME_LEN];
	struct xbee_neigh_list neigh_list;
#ifdef CONFIG_MODULE_XBEE_STATS
	struct xbee_stats stats;
#endif
};

/* initialize xbee library */
int xbee_init(void);

/* open an xbee device */
int xbee_open(struct xbee_dev *dev, FILE *xbee_file);

/* closes an xbee device */
int xbee_close(struct xbee_dev *dev);

/* Register a channel, return the ID of the channel or a negative
 * value on error. The rx_cb is a pointer to a function that will be
 * called by xbee_read() when a frame is received for this channel. If
 * rx_cb is NULL, no callback will occur. The "channel" argument can
 * be XBEE_CHANNEL_ANY to let the library choose the channel, or a
 * channel number to request a specific one. */
int xbee_register_channel(struct xbee_dev *dev, int channel,
			  xbee_rx_cb_t *rx_cb, void *opaque);

/* This function (re)sets the opaque pointer on a registered
 * channel. The function returns 0 on success and -1 on error (channel
 * not registered). As the opaque pointer can already be set after a
 * call to xbee_register_channel(), this function is only useful if
 * the opaque pointer has to be modified. */
int xbee_set_opaque(struct xbee_dev *dev, int channel, void *opaque);

/* Unregister a channel, return 0 on success */
int xbee_unregister_channel(struct xbee_dev *dev, int channel_id);

/* read data from device fd and put it in queue */
int xbee_read(struct xbee_dev *dev);

/* process all data in queue */
int xbee_process_queue(struct xbee_dev *dev);

#endif /* _XBEE_H_ */
