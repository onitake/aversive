/*  
 *  Copyright Droids Corporation (2009)
 * 
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *  Revision : $Id: scheduler_stats.h,v 1.1.2.1 2009-11-08 17:33:14 zer0 Exp $
 *
 * Olivier MATZ <zer0@droids-corp.org>
 */

#ifndef _SCHEDULER_STATS_H_
#define _SCHEDULER_STATS_H_

#ifdef CONFIG_MODULE_SCHEDULER_STATS
struct scheduler_stats {
	uint32_t alloc_fails;
	uint32_t add_event;
	uint32_t del_event;
	uint32_t max_stacking;
	uint32_t task_delayed[SCHEDULER_NB_MAX_EVENT];
	uint32_t task_scheduled[SCHEDULER_NB_MAX_EVENT];
};

extern struct scheduler_stats sched_stats;

#define SCHED_INC_STAT(x) do {			\
		uint8_t flags;			\
		IRQ_LOCK(flags);		\
		sched_stats.x++;		\
		IRQ_UNLOCK(flags);		\
	} while(0)

#define SCHED_INC_STAT2(x, i) do {		\
		uint8_t flags;			\
		IRQ_LOCK(flags);		\
		sched_stats.x[i]++;		\
		IRQ_UNLOCK(flags);		\
	} while(0)


#else /* CONFIG_MODULE_SCHEDULER_STATS */

#define SCHED_INC_STAT(x) do { } while(0)
#define SCHED_INC_STAT2(x, i) do { } while(0)

#endif /* CONFIG_MODULE_SCHEDULER_STATS */

void scheduler_stats_dump(void);

#endif /* _SCHEDULER_STATS_H_ */
