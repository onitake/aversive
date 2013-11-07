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

#ifndef _XBEE_STATS_H_
#define _XBEE_STATS_H_

/* per-device statistics */
struct xbee_stats {
	uint32_t rx_frame;
	uint32_t rx_atresp;
	uint32_t rx_atresp_error;
	uint32_t rx_modem_status;
	uint32_t rx_xmit_status;
	uint32_t rx_xmit_status_error;
	uint32_t rx_data;
	uint32_t rx_expl_data;
	uint32_t rx_node_id;
	uint32_t rx_rmt_atresp;
	uint32_t rx_rmt_atresp_error;
	uint32_t rx_frame_too_small;
	uint32_t rx_frame_too_large;
	uint32_t rx_invalid_cksum;
	uint32_t rx_invalid_type;
	uint32_t rx_no_delim;
	uint32_t rx_usr_error;

	uint32_t tx_frame;
	uint32_t tx_atcmd;
	uint32_t tx_atcmd_q;
	uint32_t tx_data;
	uint32_t tx_expl_data;
	uint32_t tx_xmit_retries;
	uint32_t tx_rmt_atcmd;
	uint32_t tx_invalid_type;
	uint32_t tx_invalid_channel;
};

struct xbee_dev;

/* return pointer to device stats */
struct xbee_stats *xbee_get_stats(struct xbee_dev *dev);

/* reset statistics of device */
void xbee_reset_stats(struct xbee_dev *dev);

/* dump statistics on stdout */
void xbee_dump_stats(struct xbee_dev *dev);

#endif /* _XBEE_STATS_H_ */
