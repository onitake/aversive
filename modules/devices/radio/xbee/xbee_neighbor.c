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

#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "xbee_neighbor.h"
#include "xbee_atcmd.h"
#include "xbee_stats.h"
#include "xbee_rxtx.h"
#include "xbee.h"

void xbee_neigh_init(struct xbee_dev *dev)
{
	LIST_INIT(&dev->neigh_list);
}

struct xbee_neigh *xbee_neigh_lookup(struct xbee_dev *dev, const char *name)
{
	struct xbee_neigh *neigh;

	LIST_FOREACH(neigh, &dev->neigh_list, next) {
		if (!strcmp(name, neigh->name))
			break;
	}

	return neigh;
}

struct xbee_neigh *xbee_neigh_rlookup(struct xbee_dev *dev, uint64_t addr)
{
	struct xbee_neigh *neigh;

	LIST_FOREACH(neigh, &dev->neigh_list, next) {
		if (addr == neigh->addr)
			break;
	}

	return neigh;
}

struct xbee_neigh *xbee_neigh_add(struct xbee_dev *dev, const char *name,
				  uint64_t addr)
{
	struct xbee_neigh *neigh;

	if (xbee_neigh_rlookup(dev, addr) != NULL)
		return NULL;

	if (xbee_neigh_lookup(dev, name) != NULL)
		return NULL;

	neigh = malloc(sizeof(*neigh));
	if (neigh == NULL)
		return NULL;

	neigh->addr = addr;
	snprintf_P(neigh->name, sizeof(neigh->name), PSTR("%s"), name);
	LIST_INSERT_HEAD(&dev->neigh_list, neigh, next);

	return neigh;
}

void xbee_neigh_del(struct xbee_dev *dev, struct xbee_neigh *neigh)
{
	dev = dev; /* silent compiler */
	LIST_REMOVE(neigh, next);
	free(neigh);
}
