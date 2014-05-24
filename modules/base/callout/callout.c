/*
 * Copyright (c) <2014>, Olivier Matz <zer0@droids-corp.org>
 * All rights reserved.
 *
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

/* Inspired from Intel DPDK rte_timer library */
/*-
 * Copyright (c) <2010>, Intel Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in
 *   the documentation and/or other materials provided with the
 *   distribution.
 *
 * - Neither the name of Intel Corporation nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>

#include <aversive.h>
#include <aversive/pgmspace.h>
#include <aversive/queue.h>

#include "callout.h"

/* allow to browse a list while modifying the current element */
#define LIST_FOREACH_SAFE(cur, next, head, field)			\
	for ((cur) = LIST_FIRST((head)),				\
		     (next) = ((cur) ? LIST_NEXT((cur), field) : NULL); \
	     (cur);							\
	     (cur) = (next),						\
		     (next) = ((cur) ? LIST_NEXT((cur), field) : NULL))

#ifdef CALLOUT_STATS
/* called with irq locked */
#define CALLOUT_STAT_ADD(cm, field, x) do {	\
	cm->stats.field += x;			\
	} while(0)
#else
#define CALLOUT_STAT_ADD(cm, field, x) do { } while(0)
#endif

#ifdef CALLOUT_DEBUG
#define callout_dprintf_P(fmt, ...) printf_P(PSTR("%s(): " fmt), __FUNCTION__, \
					 __VA_ARGS__)
#else
#define callout_dprintf_P(...) do { } while (0)
#endif

/* Initialize a callout manager */
void
callout_mgr_init(struct callout_mgr *cm, get_time_t *get_time)
{
	memset(cm, 0, sizeof(*cm));
	cm->get_time = get_time;
	LIST_INIT(&cm->sched_list);
}

/* Initialize the timer handle tim for use */
void
callout_init(struct callout *tim, callout_cb_t f, void *arg, uint8_t priority)
{
	memset(tim, 0, sizeof(*tim));
	tim->f = f;
	tim->arg = arg;
	tim->priority = priority;
}

/*
 * Add a timer in the scheduled list (timer must not already be in a list). The
 * timers are sorted in the list according the expire time (the closer timers
 * first).
 *
 * called with irq locked
 */
static void
callout_add_in_sched_list(struct callout_mgr *cm, struct callout *tim)
{
	struct callout *t, *prev_t;

	callout_dprintf_P("cm=%p tim=%p\r\n", cm, tim);
	tim->state = CALLOUT_STATE_SCHEDULED;

	/* list is empty */
	if (LIST_EMPTY(&cm->sched_list)) {
		LIST_INSERT_HEAD(&cm->sched_list, tim, next);
		return;
	}

	/* 'tim' expires before first entry */
	t = LIST_FIRST(&cm->sched_list);
	if ((int16_t)(tim->expire - t->expire) <= 0) {
		LIST_INSERT_HEAD(&cm->sched_list, tim, next);
		return;
	}

	/* find an element that will expire after 'tim' */
	LIST_FOREACH(t, &cm->sched_list, next) {
		if ((int16_t)(tim->expire - t->expire) <= 0) {
			LIST_INSERT_BEFORE(t, tim, next);
			return;
		}
		prev_t = t;
	}

	/* not found, insert at the end of the list */
	LIST_INSERT_AFTER(prev_t, tim, next);
}

/*
 * Add a timer in the local expired list (timer must not already be in a
 * list). The timers are sorted in the list according to the priority (high
 * priority first).
 *
 * called with irq locked
 */
static void
callout_add_in_expired_list(struct callout_mgr *cm,
	struct callout_list *expired_list, struct callout *tim)
{
	struct callout *t, *prev_t;

	(void)cm; /* avoid warning if debug is disabled */

	callout_dprintf_P("cm=%p tim=%p\r\n", cm, tim);
	tim->state = CALLOUT_STATE_EXPIRED;

	/* list is empty */
	if (LIST_EMPTY(expired_list)) {
		LIST_INSERT_HEAD(expired_list, tim, next);
		return;
	}

	/* 'tim' has a higher prio */
	t = LIST_FIRST(expired_list);
	if (tim->priority >= t->priority) {
		LIST_INSERT_HEAD(expired_list, tim, next);
		return;
	}

	/* find an element that will expire after 'tim' */
	LIST_FOREACH(t, expired_list, next) {
		if (tim->priority >= t->priority) {
			LIST_INSERT_BEFORE(t, tim, next);
			return;
		}
		prev_t = t;
	}

	/* not found, insert at the end of the list */
	LIST_INSERT_AFTER(prev_t, tim, next);
}

/*
 * del from list (timer must be in a list)
 */
static void
callout_del(struct callout_mgr *cm, struct callout *tim)
{
	(void)cm; /* avoid warning if debug is disabled */
	callout_dprintf_P("cm=%p tim=%p\r\n", cm, tim);
	LIST_REMOVE(tim, next);
}

/* Reset and start the timer associated with the timer handle tim */
static int
__callout_schedule(struct callout_mgr *cm, struct callout *tim,
	uint16_t expire)
{
	uint8_t flags;

	callout_dprintf_P("cm=%p tim=%p expire=%d\r\n",
			  cm, tim, expire);

	IRQ_LOCK(flags);
	CALLOUT_STAT_ADD(cm, schedule, 1);

	/* remove it from list */
	if (tim->state != CALLOUT_STATE_STOPPED) {
		/* stats */
		if (tim->state == CALLOUT_STATE_SCHEDULED)
			CALLOUT_STAT_ADD(cm, cur_scheduled, -1);
		else if (tim->state == CALLOUT_STATE_EXPIRED)
			CALLOUT_STAT_ADD(cm, cur_expired, -1);
		if (tim->state == CALLOUT_STATE_RUNNING)
			CALLOUT_STAT_ADD(cm, cur_running, -1);

		callout_del(cm, tim);
	}

	tim->expire = expire;
	CALLOUT_STAT_ADD(cm, cur_scheduled, 1);
	callout_add_in_sched_list(cm, tim);
	IRQ_UNLOCK(flags);

	return 0;
}

/* Reset and start the timer associated with the timer handle tim */
int
callout_schedule(struct callout_mgr *cm, struct callout *tim,
	uint16_t ticks)
{
	return __callout_schedule(cm, tim, cm->get_time() + ticks);
}

/* Reset and start the timer associated with the timer handle tim */
int
callout_reschedule(struct callout_mgr *cm, struct callout *tim,
	uint16_t ticks)
{
	return __callout_schedule(cm, tim, tim->expire + ticks);
}

/* Stop the timer associated with the timer handle tim */
void
callout_stop(struct callout_mgr *cm, struct callout *tim)
{
	uint8_t flags;

	callout_dprintf_P("cm=%p tim=%p\r\n", cm, tim);

	IRQ_LOCK(flags);
	if (tim->state != CALLOUT_STATE_STOPPED) {

		/* stats */
		if (tim->state == CALLOUT_STATE_SCHEDULED)
			CALLOUT_STAT_ADD(cm, cur_scheduled, -1);
		else if (tim->state == CALLOUT_STATE_EXPIRED)
			CALLOUT_STAT_ADD(cm, cur_expired, -1);
		if (tim->state == CALLOUT_STATE_RUNNING)
			CALLOUT_STAT_ADD(cm, cur_running, -1);
		CALLOUT_STAT_ADD(cm, stop, 1);

		/* remove it from list */
		callout_del(cm, tim);
		tim->state = CALLOUT_STATE_STOPPED;
	}
	IRQ_UNLOCK(flags);
}

/* must be called periodically, run all timer that expired */
void callout_manage(struct callout_mgr *cm)
{
	struct callout_list expired_list;
	struct callout_list reschedule_list;
	struct callout *tim, *tim_next;
	uint16_t cur_time;
	uint8_t old_prio;
	int16_t diff;

	CALLOUT_STAT_ADD(cm, manage, 1);
	callout_dprintf_P("cm=%p\r\n", cm);

	/* maximize the number of self-recursions */
	if (cm->nb_recursion >= CALLOUT_MAX_RECURSION) {
		CALLOUT_STAT_ADD(cm, max_recursion, 1);
		return;
	}

	cli();
	cm->nb_recursion++;
	LIST_INIT(&expired_list);
	LIST_INIT(&reschedule_list);
	cur_time = cm->get_time();
	old_prio = cm->cur_priority;

	/* move all expired timers in a local list */
	LIST_FOREACH_SAFE(tim, tim_next, &cm->sched_list, next) {

		diff = cur_time - tim->expire;

		/* check the expiration time (tasks are sorted) */
		if (diff < 0)
			break;

		callout_dprintf_P("cm=%p diff=%d\r\n", cm, diff);

		/* check the priority, if it's too low, inc stats */
		if (tim->priority <= cm->cur_priority) {
			if (diff < 16484)
				CALLOUT_STAT_ADD(cm, delayed, 1);
			else {
				/* reschedule to avoid an overflow */
				CALLOUT_STAT_ADD(cm, hard_delayed, 1);
				LIST_REMOVE(tim, next);
				tim->expire = cur_time;
				LIST_INSERT_HEAD(&reschedule_list, tim, next);
			}
			continue;
		}

		LIST_REMOVE(tim, next);
		callout_add_in_expired_list(cm, &expired_list, tim);
		CALLOUT_STAT_ADD(cm, cur_scheduled, -1);
		CALLOUT_STAT_ADD(cm, cur_expired, 1);
	}

	/* reschedule hard_delayed timers, this does not happen usually */
	while (!LIST_EMPTY(&reschedule_list)) {
		tim = LIST_FIRST(&reschedule_list);
		LIST_REMOVE(tim, next);
		callout_add_in_sched_list(cm, tim);
	}

	/* for each timer of 'expired' list, execute callback */
	while (!LIST_EMPTY(&expired_list)) {
		tim = LIST_FIRST(&expired_list);
		LIST_REMOVE(tim, next);

		/* execute callback function */
		CALLOUT_STAT_ADD(cm, cur_expired, -1);
		CALLOUT_STAT_ADD(cm, cur_running, 1);
		tim->state = CALLOUT_STATE_RUNNING;
		cm->cur_priority = tim->priority;
		sei();
		tim->f(cm, tim, tim->arg);
		cli();
	}

	cm->cur_priority = old_prio;
	cm->nb_recursion--;
	sei();
}

/* set the current priority level */
uint8_t callout_mgr_set_prio(struct callout_mgr *cm, uint8_t new_prio)
{
	uint8_t old_prio;

	old_prio = cm->cur_priority;
	if (new_prio <= old_prio)
		return old_prio;

	cm->cur_priority = new_prio;
	return old_prio;
}

/* restore the current priority level */
void callout_mgr_restore_prio(struct callout_mgr *cm, uint8_t old_prio)
{
	cm->cur_priority = old_prio;
}

/* dump statistics about timers */
void callout_dump_stats(struct callout_mgr *cm)
{
#ifdef CALLOUT_STATS
	printf_P(PSTR("Timer statistics:\r\n"));
	printf_P(PSTR("  schedule = %"PRIu32"\r\n"), cm->stats.schedule);
	printf_P(PSTR("  stop = %"PRIu32"\r\n"), cm->stats.stop);
	printf_P(PSTR("  manage = %"PRIu32"\r\n"), cm->stats.manage);
	printf_P(PSTR("  max_recursion = %"PRIu32"\r\n"), cm->stats.max_recursion);
	printf_P(PSTR("  delayed = %"PRIu32"\r\n"), cm->stats.delayed);
	printf_P(PSTR("  hard_delayed = %"PRIu32"\r\n"), cm->stats.hard_delayed);

	printf_P(PSTR("  cur_scheduled = %u\r\n"), cm->stats.cur_scheduled);
	printf_P(PSTR("  cur_expired = %u\r\n"), cm->stats.cur_expired);
	printf_P(PSTR("  cur_running = %u\r\n"), cm->stats.cur_running);
#else
	printf_P(PSTR("No timer statistics, CALLOUT_STATS is disabled\r\n"));
#endif
}
