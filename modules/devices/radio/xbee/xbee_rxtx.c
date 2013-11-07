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
#include <aversive/pgmspace.h>
#include <aversive/queue.h>
#include <aversive/endian.h>

#include <uart.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#include "xbee_neighbor.h"
#include "xbee_stats.h"
#include "xbee_rxtx.h"
#include "xbee.h"

/* Return -1 if the frame is invalid. The arguments buf and len correspond to
 * the whole xbee frame, from delimiter to cksum.  */
static int xbee_parse_atresp(struct xbee_dev *dev, void *buf, unsigned len)
{
	struct xbee_atresp_hdr *atresp_hdr;

	dev->stats.rx_atresp++;

	if (len < sizeof(struct xbee_hdr) + sizeof(struct xbee_atresp_hdr)) {
		dev->stats.rx_frame_too_small++;
		return -1;
	}

	atresp_hdr = buf + sizeof(struct xbee_hdr);

	/* bad status, but let the frame continue */
	if (atresp_hdr->status != 0)
		dev->stats.rx_atresp_error++;

	return 0;
}

/* Return -1 if the frame is invalid. The arguments buf and len correspond to
 * the whole xbee frame, from delimiter to cksum.  */
static int xbee_parse_rmt_atresp(struct xbee_dev *dev, void *buf, unsigned len)
{
	struct xbee_rmt_atresp_hdr *rmt_atresp_hdr;

	dev->stats.rx_rmt_atresp++;

	if (len < sizeof(struct xbee_hdr) + sizeof(struct xbee_rmt_atresp_hdr)) {
		dev->stats.rx_frame_too_small++;
		return -1;
	}

	rmt_atresp_hdr = buf + sizeof(struct xbee_hdr);

	/* bad status, but let the frame continue */
	if (rmt_atresp_hdr->status != 0)
		dev->stats.rx_rmt_atresp_error++;

	return 0;
}

/* Parse the reception of a "xmit status message". The arguments buf and len
 * correspond to the whole xbee frame, from delimiter to cksum. Return -1 if the
 * frame is invalid */
static int xbee_parse_xmit_status(struct xbee_dev *dev, void *buf, unsigned len)
{
	struct xbee_xmit_status_hdr *xmit_status_hdr;

	dev->stats.rx_xmit_status++;

	if (len < sizeof(struct xbee_hdr) + sizeof(struct xbee_xmit_status_hdr)) {
		dev->stats.rx_frame_too_small++;
		return -1;
	}

	xmit_status_hdr = buf + sizeof(struct xbee_hdr);
	dev->stats.tx_xmit_retries += xmit_status_hdr->xmit_retry_cnt;

	/* bad status, but let the frame continue */
	if (xmit_status_hdr->delivery_status != 0)
		dev->stats.rx_xmit_status_error++;

	return 0;
}

/* parse the frame stored in the xbee_dev structure: return 0 if the frame is
 * valid, else a negative value */
static void xbee_parse_frame(struct xbee_dev *dev)
{
	void *buf = dev->frame;
	uint8_t len = dev->frame_len;
	uint8_t hdrlen;
	struct xbee_hdr *hdr = buf;
	int i;
	uint8_t cksum = 0;
	int channel = XBEE_DEFAULT_CHANNEL;

	dev->stats.rx_frame++;

	switch (hdr->type) {
		case XBEE_TYPE_MODEM_STATUS:
		case XBEE_TYPE_RECV:
		case XBEE_TYPE_EXPL_RECV:
			hdrlen = sizeof(struct xbee_hdr) - 1; /* no frame ID */
			break;
		default:
			hdrlen = sizeof(struct xbee_hdr);
			break;
	}

	/* check frame len */
	if (len < (hdrlen + 1)) {
		dev->stats.rx_frame_too_small++;
		fprintf_P(stderr, PSTR("Frame too small\r\n"));
		return;
	}

	/* validate the cksum */
	for (i = 3; i < (len - 1); i++)
		cksum += ((uint8_t *)buf)[i];
	cksum = 0xff - cksum;
	if (cksum != ((uint8_t *)buf)[len-1]) {
		fprintf_P(stderr, PSTR("Invalid cksum\r\n"));
		dev->stats.rx_invalid_cksum++;
		return;
	}

	/* dispatch */
	switch (hdr->type) {
		case XBEE_TYPE_MODEM_STATUS:
			dev->stats.rx_modem_status++;
			channel = XBEE_DEFAULT_CHANNEL;
			break;
		case XBEE_TYPE_ATRESP:
			if (xbee_parse_atresp(dev, buf, len) < 0)
				return;
			channel = hdr->id;
			break;
		case XBEE_TYPE_RMT_ATRESP:
			if (xbee_parse_rmt_atresp(dev, buf, len) < 0)
				return;
			channel = hdr->id;
			break;
		case XBEE_TYPE_XMIT_STATUS:
			if (xbee_parse_xmit_status(dev, buf, len) < 0)
				return;
			channel = hdr->id;
			break;
		case XBEE_TYPE_RECV:
			dev->stats.rx_data++;
			channel = XBEE_DEFAULT_CHANNEL;
			break;
		case XBEE_TYPE_EXPL_RECV:
			dev->stats.rx_expl_data++;
			channel = XBEE_DEFAULT_CHANNEL;
			break;
		case XBEE_TYPE_NODE_ID:
			dev->stats.rx_node_id++;
			channel = hdr->id; //XXX
			break;
			/* invalid commands */
		case XBEE_TYPE_ATCMD:
		case XBEE_TYPE_ATCMD_Q:
		case XBEE_TYPE_XMIT:
		case XBEE_TYPE_EXPL_XMIT:
		case XBEE_TYPE_RMT_ATCMD:
		default:
			dev->stats.rx_invalid_type++;
			break;
	}

	/* fallback to default channel if not registered */
	if (channel < 0 || channel >= XBEE_MAX_CHANNEL ||
	    dev->channel[channel].registered == 0)
		channel = XBEE_DEFAULT_CHANNEL;

	/* execute the callback if any */
	if (dev->channel[channel].rx_cb == NULL)
		return;
	if (dev->channel[channel].rx_cb(dev, channel, hdr->type,
					buf + hdrlen,
					len - hdrlen - 1,
					dev->channel[channel].arg) < 0)
		dev->stats.rx_usr_error++;
}

int xbee_tx_iovec(struct xbee_dev *dev, uint8_t channel_id, uint8_t type,
	const struct xbee_msg *msg)
{
	struct xbee_hdr hdr;
	unsigned i, j;
	uint8_t cksum = 0;
	unsigned len = 0;

	for (i = 0; i < msg->iovlen; i++)
		len += msg->iov[i].len;

	/* prepare an iovec to avoid a copy: prepend a header to the
	 * buffer and append a checksum */
	hdr.delimiter = XBEE_DELIMITER;
	hdr.len = htons(len + 2);
	hdr.type = type;
	hdr.id = channel_id;

	if (channel_id >= XBEE_MAX_CHANNEL ||
	    dev->channel[channel_id].registered == 0) {
		dev->stats.tx_invalid_channel ++;
		return -1;
	}

	/* calculate the cksum */
	cksum = hdr.type;
	cksum += hdr.id;
	for (i = 0; i < msg->iovlen; i++) {
		for (j = 0; j < msg->iov[i].len; j++)
			cksum += ((uint8_t *)msg->iov[i].buf)[j];
	}
	cksum = 0xff - cksum;
	dev->stats.tx_frame ++;

	/* some additional checks before sending */
	switch (hdr.type) {

		case XBEE_TYPE_ATCMD:
			// XXX some checks ?
			dev->stats.tx_atcmd ++;
			break;
		case XBEE_TYPE_ATCMD_Q:
			dev->stats.tx_atcmd_q ++;
			break;
		case XBEE_TYPE_XMIT:
			dev->stats.tx_data ++;
			break;
		case XBEE_TYPE_EXPL_XMIT:
			dev->stats.tx_expl_data ++;
			break;
		case XBEE_TYPE_RMT_ATCMD:
			dev->stats.tx_rmt_atcmd ++;
			break;

		/* invalid commands */
		case XBEE_TYPE_XMIT_STATUS:
		case XBEE_TYPE_MODEM_STATUS:
		case XBEE_TYPE_ATRESP:
		case XBEE_TYPE_RECV:
		case XBEE_TYPE_EXPL_RECV:
		case XBEE_TYPE_NODE_ID:
		case XBEE_TYPE_RMT_ATRESP:
		default:
			dev->stats.tx_invalid_type ++;
			fprintf_P(stderr, PSTR("unhandled xmit type=%x\r\n"),
				  hdr.type);
			return -1;
	}

	/* send the frame on the wire */
	fwrite(&hdr, 1, sizeof(hdr), dev->file);
	for (i = 0; i < msg->iovlen; i++)
		fwrite(msg->iov[i].buf, 1, msg->iov[i].len, dev->file);
	fwrite(&cksum, 1, 1, dev->file);

	return 0;
}

int xbee_tx(struct xbee_dev *dev, uint8_t channel_id, uint8_t type,
	char *buf, unsigned len)
{
	struct xbee_msg msg;
	msg.iovlen = 1;
	msg.iov[0].buf = buf;
	msg.iov[0].len = len;
	return xbee_tx_iovec(dev, channel_id, type, &msg);
}

void xbee_rx(struct xbee_dev *dev)
{
	uint8_t framelen;
	struct xbee_hdr *hdr = (struct xbee_hdr *)dev->frame;
	int c;

	while (1) {

		/* read from UART */
		c = fgetc(dev->file);
		if (c == EOF)
			break;

		/* frame too long XXX stats */
		if (dev->frame_len >= XBEE_MAX_FRAME_LEN) {
			dev->frame_len = 0;
			continue;
		}

		if (dev->frame_len == 0 && c != XBEE_DELIMITER)
			continue;

		dev->frame[dev->frame_len++] = c;

		/* not enough data to read len */
		if (dev->frame_len < sizeof(*hdr))
			continue;

		framelen = ntohs(hdr->len);
		framelen += 4; /* 1 for delimiter, 2 for len, 1 for cksum */

		/* frame too long XXX stats */
		if (framelen >= XBEE_MAX_FRAME_LEN) {
			dev->frame_len = 0;
			dev->stats.rx_frame++;
			dev->stats.rx_frame_too_large++;
			continue;
		}

		/* not enough data */
		if (dev->frame_len < framelen)
			continue;

		xbee_parse_frame(dev);
		dev->frame_len = 0;
	}
}
