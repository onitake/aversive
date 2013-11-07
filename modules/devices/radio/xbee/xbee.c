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

#include <aversive.h>
#include <aversive/queue.h>
#include <aversive/pgmspace.h>

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "xbee_neighbor.h"
#include "xbee_stats.h"
#include "xbee_rxtx.h"
#include "xbee.h"

int xbee_init(void)
{
	return 0;
}

int xbee_register_channel(struct xbee_dev *dev, int channel,
			  xbee_rx_cb_t *rx_cb, void *opaque)
{
	/* user asked for any channel */
	if (channel == XBEE_CHANNEL_ANY) {
		int ch;

		/* skip XBEE_DEFAULT_CHANNEL == 0 */
		for (ch = 1; ch < XBEE_MAX_CHANNEL; ch++) {
			if (dev->channel[ch].registered == 0) {
				channel = ch;
				break;
			}
		}
		/* no available channels */
		if (channel == XBEE_CHANNEL_ANY)
			return -1;
	}
	/* user requested a specific channel */
	else if (channel < 0 || channel >= XBEE_MAX_CHANNEL ||
		 dev->channel[channel].registered == 1)
		return -1; /* not available */

	dev->channel[channel].registered = 1;
	dev->channel[channel].rx_cb = rx_cb;
	dev->channel[channel].arg = opaque;
	return channel;
}

int xbee_unregister_channel(struct xbee_dev *dev, int channel)
{
	if (channel < 0 || channel >= XBEE_MAX_CHANNEL ||
	    dev->channel[channel].registered == 0)
		return -1;
	dev->channel[channel].registered = 0;
	dev->channel[channel].rx_cb = NULL;
	dev->channel[channel].arg = NULL;
	return 0;
}

int xbee_set_opaque(struct xbee_dev *dev, int channel, void *opaque)
{
	if (channel < 0 || channel >= XBEE_MAX_CHANNEL ||
	    dev->channel[channel].registered == 0)
		return -1;

	dev->channel[channel].arg = opaque;
	return 0;
}

int xbee_open(struct xbee_dev *dev, FILE *xbee_file)
{
	memset(dev, 0, sizeof(*dev));
	dev->file = xbee_file;
	xbee_neigh_init(dev);
	return 0;
}
