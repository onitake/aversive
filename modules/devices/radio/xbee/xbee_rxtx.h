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

#ifndef _XBEE_RXTX_H_
#define _XBEE_RXTX_H_

/* protocol headers */

#define XBEE_DELIMITER 0x7E
#define XBEE_MAX_FRAME_LEN 0x50

struct xbee_hdr {
	uint8_t delimiter;
	uint16_t len;
	uint8_t type;
	uint8_t id;
} __attribute__((packed));

#define XBEE_TYPE_ATCMD 0x08
struct xbee_atcmd_hdr {
	uint16_t cmd;
	uint8_t params[];
} __attribute__((packed));

#define XBEE_TYPE_ATCMD_Q 0x09
struct xbee_atcmd_q_hdr {
	uint16_t cmd;
	uint8_t params[];
} __attribute__((packed));

#define XBEE_TYPE_XMIT 0x10
struct xbee_xmit_hdr {
	uint64_t dstaddr;
	uint16_t reserved;
	uint8_t bcast_radius;
	uint8_t opts;
	uint8_t data[];
} __attribute__((packed));

#define XBEE_TYPE_EXPL_XMIT 0x11
struct xbee_expl_xmit_hdr {
	uint64_t dstaddr;
	uint16_t reserved;
	uint8_t src_endpoint;
	uint8_t dst_endpoint;
	uint16_t cluster_id;
	uint16_t profile_id;
	uint8_t bcast_radius;
	uint8_t opts;
	uint8_t data[];
} __attribute__((packed));

#define XBEE_TYPE_RMT_ATCMD 0x17
struct xbee_rmt_atcmd_hdr {
	uint64_t dstaddr;
	uint16_t reserved;
	uint8_t opts;
	uint16_t cmd;
	uint8_t params[];
} __attribute__((packed));

#define XBEE_TYPE_ATRESP 0x88
struct xbee_atresp_hdr {
	uint16_t cmd;
	uint8_t status;
	uint8_t data[];
} __attribute__((packed));

#define XBEE_TYPE_MODEM_STATUS 0x8A
struct xbee_modem_status_hdr {
	/* empty */
} __attribute__((packed));

#define XBEE_TYPE_XMIT_STATUS 0x8B
struct xbee_xmit_status_hdr {
	uint16_t reserved;
	uint8_t xmit_retry_cnt;
	uint8_t delivery_status;
	uint8_t discovery_status;
} __attribute__((packed));

#define XBEE_TYPE_RECV 0x90
struct xbee_recv_hdr {
	uint64_t srcaddr;
	uint16_t reserved;
	uint8_t opts;
	uint8_t data[];
} __attribute__((packed));

#define XBEE_TYPE_EXPL_RECV 0x91
struct xbee_expl_recv_hdr {
	uint64_t srcaddr;
	uint16_t reserved;
	uint8_t src_endpoint;
	uint8_t dst_endpoint;
	uint16_t cluster_id;
	uint16_t profile_id;
	uint8_t opts;
	uint8_t data[];
} __attribute__((packed));

#define XBEE_TYPE_NODE_ID 0x95
struct xbee_node_id_hdr {
	uint64_t srcaddr;
	uint16_t srcnetwork;
	uint8_t opts;
	uint16_t dstnetwork;
	uint64_t dstaddr;
	uint8_t ni_string[];
	/* uint16_t parentaddr; after variable field */
} __attribute__((packed));

#define XBEE_TYPE_RMT_ATRESP 0x97
struct xbee_rmt_atresp_hdr {
	uint64_t srcaddr;
	uint16_t reserved;
	uint16_t cmd;
	uint8_t status;
	uint8_t data[];
} __attribute__((packed));

struct xbee_dev;

struct xbee_iovec {
	void *buf;     /* Buffer */
	unsigned len;  /* Number of bytes in the buffer */
};

#define XBEE_MSG_MAXIOV 4
struct xbee_msg {
	struct xbee_iovec iov[XBEE_MSG_MAXIOV]; /* scatter/gather array */
	unsigned iovlen;                        /* number of valid iov */
};

/* send a frame */
int xbee_tx(struct xbee_dev *dev, uint8_t id, uint8_t type,
	char *buf, unsigned len);

/* send a frame using an iovec */
int xbee_tx_iovec(struct xbee_dev *dev, uint8_t id, uint8_t type,
	const struct xbee_msg *msg);

/* process the content of the rx buffer and decode incoming frames */
void xbee_rx(struct xbee_dev *dev);

#endif /* _XBEE_RXTX_H_ */
