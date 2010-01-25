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
#include <fcntl.h>
#include <string.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <termios.h>

#ifdef CONFIG_MODULE_SCHEDULER
#include <scheduler.h>
#endif
#ifdef CONFIG_MODULE_UART
#include <uart_host.h>
#endif

static pthread_mutex_t mut = PTHREAD_MUTEX_INITIALIZER;
static volatile int cpt = 0;

static struct termios oldterm;
/* static */ int old_stdin, old_stdout;
static int stdin_pipe[2];
static int stdout_pipe[2];

enum msg_type {
	SCHED,
	UART_RCV,
	UART_SND,
};

struct message {
	enum msg_type type;
	char c;
};
static struct message g_msg;

#ifdef SA_SIGINFO
static void sigusr1(__attribute__((unused)) int sig,
		    __attribute__((unused)) siginfo_t *info,
		    __attribute__((unused)) void *uc)
#else
void sigusr1(__attribute__((unused)) int sig)
#endif
{
	struct message m;
	m = g_msg;

#ifdef CONFIG_MODULE_SCHEDULER
	if (m.type == SCHED) {
		pthread_mutex_unlock(&mut);
		scheduler_interrupt();
	}
#endif

#ifdef CONFIG_MODULE_UART
	if (m.type == UART_RCV) {
		uart_host_rx_event(m.c);
		pthread_mutex_unlock(&mut);
	}
	if (m.type == UART_SND) {
		uart_host_tx_event(m.c);
		pthread_mutex_unlock(&mut);
	}
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
	pthread_t thread = (pthread_t)arg;
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
			g_msg.type = SCHED;
			pthread_kill(thread, SIGUSR1);

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

void *hostsim_uart_stdin(void *arg)
{
	pthread_t thread = (pthread_t)arg;
	int n;
	char c;

	/* read on old stdin and put it in pipe */
	while (1) {
		n = read(old_stdin, &c, 1);
		if (n <= 0)
			break;

		pthread_mutex_lock(&mut);
		g_msg.type = UART_RCV;
		g_msg.c = c;
		pthread_kill(thread, SIGUSR1);

		write(stdin_pipe[1], &c, 1);
	}
	pthread_exit(NULL);
	return NULL;
}

void *hostsim_uart_stdout(void *arg)
{
	pthread_t thread = (pthread_t)arg;
	int n;
	char c;

	/* read on our pipe, and forward it to the old stdout */
	while (1) {
		n = read(stdout_pipe[0], &c, 1);
		if (n <= 0)
			break;

		pthread_mutex_lock(&mut);
		g_msg.type = UART_SND;
		g_msg.c = c;
		pthread_kill(thread, SIGUSR1);

		write(old_stdout, &c, 1);
	}
	pthread_exit(NULL);
	return NULL;
}

int hostsim_uart_init(void)
{
	struct termios term;

	tcgetattr(0, &oldterm);
	memcpy(&term, &oldterm, sizeof(term));
	term.c_lflag &= ~(ICANON | ECHO | ISIG);
	tcsetattr(0, TCSANOW, &term);

	/* duplicate stdin */
	old_stdin = dup(0);
	if (old_stdin < 0)
		return 1;

	/* duplicate stdout */
	old_stdout = dup(1);
	if (old_stdout < 0)
		return -1;

	/* create 2 pipes */
	if (pipe(stdin_pipe) < 0)
		return -1;
	if (pipe(stdout_pipe) < 0)
		return -1;

	/* replace file desc 0 (stdin) by our pipe */
	if (dup2(stdin_pipe[0], 0) < 0)
		return -1;
	close(stdin_pipe[0]);

	/* replace file desc 1 (stdout) by our pipe */
	if (dup2(stdout_pipe[1], 1) < 0)
		return -1;
	close(stdout_pipe[1]);
	setbuf(stdin, NULL);
	setbuf(stdout, NULL);

	return 0;
}

int hostsim_init(void)
{
	struct sigaction sigact;
	pthread_t parent_id, child_id, child2_id, child3_id;
	int ret;

	parent_id = pthread_self();

	pthread_mutex_lock(&mut);
	ret = pthread_create(&child_id, NULL, parent, (void *)parent_id);
	if (ret) {
		printf("pthread_create() returned %d\n", ret);
		pthread_mutex_unlock(&mut);
		return -1;
	}

#ifdef CONFIG_MODULE_UART
	if (hostsim_uart_init())
		return -1;

	ret = pthread_create(&child2_id, NULL, hostsim_uart_stdin, (void *)parent_id);
	if (ret) {
		printf("pthread_create() returned %d\n", ret);
		pthread_mutex_unlock(&mut);
		return -1;
	}
	ret = pthread_create(&child3_id, NULL, hostsim_uart_stdout, (void *)parent_id);
	if (ret) {
		printf("pthread_create() returned %d\n", ret);
		pthread_mutex_unlock(&mut);
		return -1;
	}
#endif

	/* register a signal handler, which is interruptible */
	memset(&sigact, 0, sizeof(sigact));
	sigemptyset(&sigact.sa_mask);
	sigact.sa_flags |= SA_NODEFER;
	sigact.sa_sigaction = sigusr1;
	sigaction(SIGUSR1, &sigact, NULL);

	/* */
	if (siginterrupt (SIGUSR1, 0) != 0)
		return -1;

	pthread_mutex_unlock(&mut);

	return 0;
}

int hostsim_exit(void)
{
#ifdef CONFIG_MODULE_UART
	tcsetattr(0, TCSANOW, &oldterm);
#endif
	return 0;
}
#endif /* HOST_VERSION */
