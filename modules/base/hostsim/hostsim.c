/*
 *  Copyright Droids Corporation
 *  Olivier Matz <zer0@droids-corp.org>
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
 *  Revision : $Id: main.c,v 1.10 2009-11-08 17:24:33 zer0 Exp $
 *
 */

#ifdef HOST_VERSION
/*
 * AVR simulator. This code is used on host (PC for instance) to
 * generate a signal that will call other aversive modules like
 * scheduler or uart. The goal is to simulate the behaviour of
 * hardware interrupts in a unix application.
 */

#include <aversive.h>

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/wait.h>

#ifdef CONFIG_MODULE_SCHEDULER
#include <scheduler.h>
#endif

static pthread_mutex_t mut = PTHREAD_MUTEX_INITIALIZER;
static volatile int cpt = 0;

#ifdef SA_SIGINFO
static void sigusr1(__attribute__((unused)) int sig,
		    __attribute__((unused)) siginfo_t *info,
		    __attribute__((unused)) void *uc)
#else
void sigusr1(__attribute__((unused)) int sig)
#endif
{
	pthread_mutex_unlock(&mut);

#ifdef CONFIG_MODULE_SCHEDULER
	scheduler_interrupt();
#endif
}

static int lock_count = 0;

void hostsim_lock(void)
{
	if (lock_count++)
		return;
	pthread_mutex_lock(&mut);
}

void hostsim_unlock(void)
{
	if (lock_count-- == 1)
		pthread_mutex_unlock(&mut);
}

int hostsim_islocked(void)
{
	return lock_count;
}

void host_wait_ms(int ms)
{
	struct timeval tv, tv2, diff;

	gettimeofday(&tv, NULL);
	diff.tv_sec = (1000 * ms) / 1000000;
	diff.tv_usec = (1000 * ms) % 1000000;
	timeradd(&tv, &diff, &tv);
	gettimeofday(&tv2, NULL);

	while (timercmp(&tv2, &tv, <)) {
		usleep(1000);
		gettimeofday(&tv2, NULL);
	}
}


/* sends signal to child */
void *parent(void *arg)
{
	pthread_t *thread = arg;
	struct timeval cur_tv, prev_tv, tv_millisec;
	int n;

	gettimeofday(&prev_tv, NULL);

	while (1) {
		usleep(1000);
		gettimeofday(&cur_tv, NULL);

		n = 0;
		while (timercmp(&prev_tv, &cur_tv, <)) {
			if (n > 5) {
				/* give some time between subsequent
				 * signals */
				usleep(100);
				n = 0;
			}
			pthread_mutex_lock(&mut);
			pthread_kill(*thread, SIGUSR1);

			/* signal was acked */
			tv_millisec.tv_sec = 0;
			tv_millisec.tv_usec = 1000;
			timeradd(&prev_tv, &tv_millisec, &prev_tv);
			n ++;
		}
	}

	pthread_exit(NULL);
	return NULL;
}

int hostsim_init(void)
{
	struct sigaction sigact;
	pthread_t parent_id, child_id;
	int ret;

	parent_id = pthread_self();

	pthread_mutex_lock(&mut);
	ret = pthread_create(&child_id, NULL, parent, (void *)parent_id);
	if (ret) {
		printf("pthread_create() returned %d\n", ret);
		pthread_mutex_unlock(&mut);
		return -1;
	}

	/* register a signal handler, which is interruptible */
	memset(&sigact, 0, sizeof(sigact));
	sigemptyset(&sigact.sa_mask);
	sigact.sa_flags |= SA_NODEFER;
	sigact.sa_sigaction = sigusr1;
	sigaction(SIGUSR1, &sigact, NULL);

	/* */
	if (siginterrupt (SIGUSR1, 0) != 0)
		return -1;

	printf("hostsim_init()\n", ret);
	pthread_mutex_unlock(&mut);

	return 0;
}
#endif /* HOST_VERSION */
