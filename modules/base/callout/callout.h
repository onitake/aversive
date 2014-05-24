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

#ifndef _CALLOUT_H_
#define _CALLOUT_H_

#include <aversive/queue.h>

#define CALLOUT_STATS
/* #define CALLOUT_DEBUG */

/**
 * This module provides a timer service to Aversive similar to the old
 * "scheduler" module. The manager function callout_manage() can be called from
 * an interrupt (like the scheduler does) or from a standard function (usually a
 * main-loop).
 *
 * Each timer has a priority: the timers with higher priorities are scheduled
 * before the others. This feature is mostly useful when the manager is called
 * from an interrupt. Indeed, the callback function of a timer with a high
 * priority cannot be preempted by a timer with a lower priority.
 *
 * The module locks interrupts when doing critical operations, ensuring that
 * critical data are accessed atomically.
 *
 * State of timers:
 * - stopped: initial state after callout_init()
 * - scheduled: after a call to callout_schedule(), the timer is in the
 *   scheduled list of the callout manager
 * - expired: after a call to callout_manage(), if the expire time of a
 *   timer is reached, it is moved in a local list and its state is
 *   changed to "expired".
 * - before starting the callback, the timer goes in state "running".
 *
 * Once running, the associated timer is not touched anymore by
 * callout_manage(). As a result, the timer MUST be either reloaded or stopped
 * (and potentially freed).
 */

#define CALLOUT_MAX_RECURSION 5

#ifdef CALLOUT_STATS
/**
 * The structure that stores the timer statistics, mostly useful for debug
 * purposes.
 */
struct callout_debug_stats {
	uint32_t schedule;      /**< nb of calls to callout_(re)schedule() */
	uint32_t stop;          /**< nb of calls to callout_stop() */
	uint32_t manage;        /**< nb of calls to callout_manage() */
	uint32_t max_recursion; /** manage() skipped due to max recursion */
	uint32_t delayed;       /** task delayed a bit due to low prio */
	uint32_t hard_delayed;  /** task recheduled later due to low priority */

	uint8_t cur_scheduled;  /**< current number of scheduled timers */
	uint8_t cur_expired;    /**< current number of expired timers */
	uint8_t cur_running;    /**< current number of running timers */
};
#endif

struct callout;
struct callout_mgr;

/**
 * The tyoe of a callout callback function.
 */
typedef void (callout_cb_t)(struct callout_mgr *cm, struct callout *tim,
	void *arg);

/**
 * A callout structure, storing all data associated to a timer.
 */
struct callout {
	LIST_ENTRY(callout) next; /**< next/prev in list */

#define CALLOUT_STATE_STOPPED    0 /**< not scheduled */
#define CALLOUT_STATE_SCHEDULED  1 /**< pres*/
#define CALLOUT_STATE_EXPIRED    2
#define CALLOUT_STATE_RUNNING    3
	uint8_t state;             /**< stopped, scheduled, expired */
	uint8_t priority;          /**< the priority of the timer */
	uint16_t expire;           /**< time when timer should expire */

	callout_cb_t *f;           /**< callback function pointer */
	void *arg;                 /**< argument given to the cb function. */
};

/* define the callout list */
LIST_HEAD(callout_list, callout);

/* static initializer for a timer structure */
#define CALLOUT_INITIALIZER { }

/**
 * Type of the function used by a callout manager to get a time reference
 */
typedef uint16_t (get_time_t)(void);

/**
 * An instance of callout manager. It is possible to have several managers. A
 * callout is attached to one manager at a time.
 */
struct callout_mgr {
	get_time_t *get_time; /**< func used to get the time reference */
	uint16_t prev_time;   /**< time of previous call */
	uint8_t cur_priority; /** priority of running event */
	uint8_t nb_recursion; /** number of recursion */
	struct callout_list sched_list; /**< list of scheduled timers */

#ifdef CALLOUT_STATS
	struct callout_debug_stats stats; /**< stats */
#endif
};

/**
 * Initialize a callout manager
 *
 * The callout manager must be initialized before callout_add() or
 * callout_manage() can be called.
 *
 * @param cm
 *   Pointer to the uninitialized callout manager structure.
 * @param get_time
 *   Pointer to a function that returns a time reference (unsigned 16 bits).
 */
void callout_mgr_init(struct callout_mgr *cm, get_time_t *get_time);

/**
 * Initialize a callout structure and set callback function
 *
 * Before doing any operation on the callout structure, it has to be initialized
 * with this function. It is possible to reinitialize a timer that has been
 * previously scheduled, but it must be stopped.
 *
 * @param tim
 *   The timer to initialize.
 * @param priority
 *   The priority of the callout (high value means higher priority)
 * @param f
 *   The callback function of the timer.
 * @param arg
 *   The user argument of the callback function.
 */
void callout_init(struct callout *tim, callout_cb_t f, void *arg,
	uint8_t priority);

/**
 * Schedule a callout
 *
 * The callout_schedule() function adds the timer in the scheduled list. After
 * the specified amount of ticks are elapsed, the callback function of the timer
 * previously given to callout_init() will be invoked with its argument.
 *
 * The given "tick" value is relative to the current time, and is 16 bits
 * wide. As it internally uses signed 16 bits comparison, the max value for
 * ticks is 32767.
 *
 * @param cm
 *   The callout manager where the timer should be scheduled
 * @param tim
 *   The timer handle
 * @param ticks
 *   The number of ticks before the callback function is called, relative to now
 *   (the reference is given by the get_time() function of the callout manager).
 * @return
 *   0 on success, negative on error
 */
int callout_schedule(struct callout_mgr *cm, struct callout *tim,
	uint16_t ticks);

/**
 * Reschedule a callout
 *
 * This function does exactly the same than callout_schedule() except that
 * the given time "ticks" is not relative to the current time but to the
 * "expire" field of the timer.
 *
 * Using this function is advised to avoid drift if you want to have periodic
 * timers.
 *
 * This function should preferably be called from the callback function of
 * the timer. Indeed, if the "expire" field should be a known value or it
 * can result in an undefined behavior
 *
 * The given "tick" value is relative to the "expire" field of the timer, and is
 * 16 bits wide. As it internally uses signed 16 bits comparison, the max value
 * for ticks is 32767.
 *
 * @param cm
 *   The callout manager where the timer should be scheduled
 * @param tim
 *   The timer handle
 * @param ticks
 *   The number of ticks before the callback function is called, relative to
 *   the "expire" value of the timer
 * @return
 *   0 on success, negative on error
 */
int callout_reschedule(struct callout_mgr *cm, struct callout *tim,
	uint16_t ticks);


/**
 * Stop a timer.
 *
 * The callout_stop() function stops a timer associated with the
 * timer handle tim.
 *
 * If the timer is scheduled or expired, it is removed from the list: the
 * callback function won't be invoked. If the timer is stopped or running the
 * function does nothing.
 *
 * If a timer structure is dynamically allocated, invoking callout_stop() is
 * needed before freeing the structure, even if the freeing occurs in a
 * callback. Indeed, this function can be called safely from a timer
 * callback. If it succeeds, the timer is not referenced anymore by the callout
 * manager.
 *
 * @param cm
 *   The callout manager where the timer is or was scheduled
 * @param tim
 *   The timer
 * @return
 *   0 on success, negative on error
 */
void callout_stop(struct callout_mgr *cm, struct callout *tim);


/**
 * Return the state of a timer
 *
 * @param tim
 *   The timer
 * @return
 *   - CALLOUT_STATE_STOPPED: the timer is stopped
 *   - CALLOUT_STATE_SCHEDULED: the timer is scheduled
 *   - CALLOUT_STATE_EXPIRED: the timer was moved in a local list before
 *     execution
 */
static inline uint8_t callout_state(struct callout *tim)
{
	return tim->state;
}

/**
 * Manage the timer list and execute callback functions.
 *
 * This function must be called periodically, either from a loop of from an
 * interrupt. It browses the list of scheduled timers and runs all timers that
 * are expired.
 *
 * This function must be called at least every 16384 reference ticks of
 * cm->get_time(), but calling it more often is recommanded to avoid delaying
 * task abusively.
 *
 * The function must be called with IRQ allowed.
 */
void callout_manage(struct callout_mgr *cm);

/**
 * Dump statistics about timers.
 */
void callout_dump_stats(struct callout_mgr *cm);

/**
 * Set the current priority level
 *
 * Prevent callout with a priority lower than "new_prio" to be executed.
 * If the current priority of the callout manager is already lower higher
 * than "new_prio", the function won't change the running priority.
 *
 * The returned value should be stored by the caller and restored with
 * callout_mgr_restore_prio(), preferably in the same function.
 *
 * @param cm
 *   The callout manager
 * @param new_prio
 *   The new running priority
 *
 * @return
 *   The value of the running priority before the call og this function
 */
uint8_t callout_mgr_set_prio(struct callout_mgr *cm, uint8_t new_prio);

/**
 * Restore the current priority level
 *
 * Used after a call to callout_mgr_set_prio().
 *
 * @param cm
 *   The callout manager
 * @param old_prio
 *   The old running priority
 */
void callout_mgr_restore_prio(struct callout_mgr *cm, uint8_t old_prio);

#endif /* _CALLOUT_H_ */
