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

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "xbee_neighbor.h"
#include "xbee_stats.h"
#include "xbee_rxtx.h"
#include "xbee.h"

struct xbee_stats *xbee_get_stats(struct xbee_dev *dev)
{
#ifdef CONFIG_MODULE_XBEE_STATS
	return &dev->stats;
#else
	(void)dev;
	return NULL;
#endif
}

void xbee_reset_stats(struct xbee_dev *dev)
{
	(void)dev;
#ifdef CONFIG_MODULE_XBEE_STATS
	memset(&dev->stats, 0, sizeof(dev->stats));
#endif
}

void xbee_dump_stats(struct xbee_dev *dev)
{
#ifdef CONFIG_MODULE_XBEE_STATS
	printf_P(PSTR("statistics on xbee_dev %p:\r\n"), dev);
	printf_P(PSTR(" rx_frame: %"PRIu32"\r\n"), dev->stats.rx_frame);
	printf_P(PSTR(" rx_atresp: %"PRIu32"\r\n"), dev->stats.rx_atresp);
	printf_P(PSTR(" rx_atresp_error: %"PRIu32"\r\n"), dev->stats.rx_atresp_error);
	printf_P(PSTR(" rx_modem_status: %"PRIu32"\r\n"), dev->stats.rx_modem_status);
	printf_P(PSTR(" rx_xmit_status: %"PRIu32"\r\n"), dev->stats.rx_xmit_status);
	printf_P(PSTR(" rx_xmit_status_error: %"PRIu32"\r\n"), dev->stats.rx_xmit_status_error);
	printf_P(PSTR(" rx_data: %"PRIu32"\r\n"), dev->stats.rx_data);
	printf_P(PSTR(" rx_expl_data: %"PRIu32"\r\n"), dev->stats.rx_expl_data);
	printf_P(PSTR(" rx_node_id: %"PRIu32"\r\n"), dev->stats.rx_node_id);
	printf_P(PSTR(" rx_rmt_atresp: %"PRIu32"\r\n"), dev->stats.rx_rmt_atresp);
	printf_P(PSTR(" rx_rmt_atresp_error: %"PRIu32"\r\n"), dev->stats.rx_rmt_atresp_error);
	printf_P(PSTR(" rx_frame_too_small: %"PRIu32"\r\n"), dev->stats.rx_frame_too_small);
	printf_P(PSTR(" rx_frame_too_large: %"PRIu32"\r\n"), dev->stats.rx_frame_too_large);
	printf_P(PSTR(" rx_invalid_cksum: %"PRIu32"\r\n"), dev->stats.rx_invalid_cksum);
	printf_P(PSTR(" rx_invalid_type: %"PRIu32"\r\n"), dev->stats.rx_invalid_type);
	printf_P(PSTR(" rx_no_delim: %"PRIu32"\r\n"), dev->stats.rx_no_delim);
	printf_P(PSTR(" rx_usr_error: %"PRIu32"\r\n"), dev->stats.rx_usr_error);
	printf_P(PSTR(" tx_frame: %"PRIu32"\r\n"), dev->stats.tx_frame);
	printf_P(PSTR(" tx_atcmd: %"PRIu32"\r\n"), dev->stats.tx_atcmd);
	printf_P(PSTR(" tx_atcmd_q: %"PRIu32"\r\n"), dev->stats.tx_atcmd_q);
	printf_P(PSTR(" tx_data: %"PRIu32"\r\n"), dev->stats.tx_data);
	printf_P(PSTR(" tx_expl_data: %"PRIu32"\r\n"), dev->stats.tx_expl_data);
	printf_P(PSTR(" tx_xmit_retries: %"PRIu32"\r\n"), dev->stats.tx_xmit_retries);
	printf_P(PSTR(" tx_rmt_atcmd: %"PRIu32"\r\n"), dev->stats.tx_rmt_atcmd);
	printf_P(PSTR(" tx_invalid_type: %"PRIu32"\r\n"), dev->stats.tx_invalid_type);
	printf_P(PSTR(" tx_invalid_channel: %"PRIu32"\r\n"), dev->stats.tx_invalid_channel);
#else
	(void)dev;
	printf_P(PSTR("Statistics not enabled\r\n"));
#endif
}
