/*
 *  Copyright 2014 Olivier Matz <zer0@droids-corp.org>
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

/*
 * To simulate AVR interruptions, we use signals. The signals are
 * always allowed, and the handler is located in hostsim code.  In the
 * hostsim handler, we check if the IRQ are allowed (in this case we
 * call the handler) or not (in this case we ignore the IRQ). Indeed,
 * the sei() / cli() functions just set a flag located in hostsim.
 *
 * Note that a signal handler can be preempted by itself. It means
 * that
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
#include <sys/queue.h>
#include <termios.h>
#include <signal.h>

#ifdef CONFIG_MODULE_SCHEDULER
#include <scheduler.h>
#endif
#ifdef CONFIG_MODULE_UART
#include <uart_host.h>
#endif

/* list of scheduled ittimers */
LIST_HEAD(hostsim_ittimer_list, hostsim_ittimer);
struct hostsim_ittimer_list tim_list = LIST_HEAD_INITIALIZER(&tim_list);

/* structure describing an interruption timer */
struct hostsim_ittimer {
	LIST_ENTRY(hostsim_ittimer) next;
	uint32_t period_ns;
	uint64_t prev_tick;
	unsigned pending;
	void (*handler)(void);
};

/* tell if global emulated IRQ are locked (1) or allowed (0). The
 * default IRQ state is locked as on a real avr */
#define IRQ_LOCKED  0
#define IRQ_ALLOWED 1
static uint8_t irq_flags = IRQ_LOCKED;

static int old_stdin, old_stdout;
static int stdin_pipe[2];
static int stdout_pipe[2];

/* ring buffer to send from pthread to signal */
#define RG_SIZE 256 /* pow of 2 */
#define RG_MASK (RG_SIZE-1)
volatile char out_buf[RG_SIZE];
volatile unsigned out_tail = 0;
volatile unsigned out_head = 0;
pthread_t out_pthread;
volatile char in_buf[RG_SIZE];
volatile unsigned in_tail = 0;
volatile unsigned in_head = 0;
pthread_t in_pthread;

/* store the previous state of termios, restored in hostsim_exit() */
static struct termios oldterm;

void hostsim_sei(void)
{
	irq_flags = IRQ_ALLOWED;
}

void hostsim_cli(void)
{
	irq_flags = IRQ_LOCKED;
}

/* lock emulated IRQ and return the previous state */
uint8_t hostsim_irq_save(void)
{
	uint8_t old = irq_flags;
	hostsim_cli();
	return old;
}

/* returns a monotonic clock in nanoseconds */
static inline uint64_t get_clk(void)
{
	struct timespec ts;
	uint64_t t;

	if (clock_gettime(CLOCK_MONOTONIC_RAW, &ts) < 0) {
		printf("clock_getres() failed: %s\n", strerror(errno));
		exit(1);
	}
	t = ts.tv_sec;
	t *= 1000000000;
	t += ts.tv_nsec;
	return t;
}

static void handle_interrupts(void)
{
	struct hostsim_ittimer *tim;
	uint64_t clk;
	uint64_t cur, next;
	int c;
	unsigned head;

	clk = get_clk();

	/* do the uart events if data is available */
	if (irq_flags == IRQ_ALLOWED) {
		while ((int)(out_tail - out_head) > 0) {
			irq_flags = IRQ_LOCKED;
			head = out_head;
			c = out_buf[out_head & RG_MASK];
			if (__sync_val_compare_and_swap(&out_head,
					head, head + 1) != head)
				continue;
			uart_host_tx_event(c);
			irq_flags = IRQ_ALLOWED;
		}
		while ((int)(in_tail - in_head) > 0) {
			irq_flags = IRQ_LOCKED;
			head = in_head;
			c = in_buf[in_head & RG_MASK];
			uart_host_rx_event(c);
			if (__sync_val_compare_and_swap(&in_head,
					head, head + 1) != head)
				continue;
			irq_flags = IRQ_ALLOWED;
		}
	}

	/* browse all timers */
	LIST_FOREACH(tim, &tim_list, next) {

		/* Call the handler if it's time to. We use an atomic operation
		 * because we can be interrupted by our own signal any moment */
		do {
			cur = tim->prev_tick;
			next = tim->prev_tick + tim->period_ns;

			if (next > clk)
				break;

			if (__sync_val_compare_and_swap(&tim->prev_tick,
					cur, next) != cur)
				continue;

			/* if irq are disabled, just mark the timer as pending,
			 * it will be executed from hostsim_irq_restore(). We
			 * may loose interrupts if they stay locked too long,
			 * like on the real hw */
			if (irq_flags == IRQ_LOCKED)
				tim->pending = 1;
			else {
				irq_flags = IRQ_LOCKED;
				tim->pending = 0;
				tim->handler();
				irq_flags = IRQ_ALLOWED;
			}
		} while (1);

		/* also execute the irq if it was pending */
		if (irq_flags == IRQ_ALLOWED && tim->pending == 1) {
			irq_flags = IRQ_LOCKED;
			tim->pending = 0;
			tim->handler();
			irq_flags = IRQ_ALLOWED;
		}
	}
}

/* restore the state given as parameter  */
void hostsim_irq_restore(uint8_t flags)
{
	/* on transition "locked" -> "unlocked", call any pending interrupts
	 * before releasing IRQ lock. For other transitions, just set the
	 * irq_flags variable */

	if (irq_flags == IRQ_ALLOWED || flags == IRQ_LOCKED) {
		irq_flags = flags;
		return;
	}

	irq_flags = IRQ_ALLOWED;
	handle_interrupts();
}

/* return 1 if emulated IRQ are locked */
uint8_t hostsim_irq_locked(void)
{
	return irq_flags == IRQ_LOCKED;
}

/* replacement for wait_ms() */
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

/* add a new timer: loaded at init and cannot be unloaded. */
struct hostsim_ittimer *hostsim_ittimer_add(void (*handler)(void),
	unsigned period_ns)
{
	struct hostsim_ittimer *tim;

	tim = malloc(sizeof(*tim));
	if (tim == NULL) {
		printf("not enough memory: cannot allocate timer\n");
		exit(1);
	}

	tim->period_ns = period_ns;
	tim->prev_tick = get_clk();
	tim->handler = handler;
	LIST_INSERT_HEAD(&tim_list, tim, next);
	return tim;
}

/* the signal handler, preemptable by itself */
#ifdef SA_SIGINFO
static void sigtimer(__attribute__((unused)) int sig,
	__attribute__((unused)) siginfo_t *info,
	__attribute__((unused)) void *uc)
#else
	void sigtimer(__attribute__((unused)) int sig)
#endif
{
	static int recurs = 0;

	if (recurs > 10)
		return;
	recurs++;
	handle_interrupts();
	recurs--;
}

/* enable loaded ittimers */
int hostsim_ittimer_enable(unsigned timer_resolution_us)
{
	struct sigaction sigact;
	struct itimerval t;

	/* register a signal handler, which is interruptible */
	memset(&sigact, 0, sizeof(sigact));
	sigemptyset(&sigact.sa_mask);
	sigact.sa_flags |= SA_NODEFER;
	sigact.sa_sigaction = sigtimer;
	sigaction(SIGALRM, &sigact, NULL);

	/* do not interrupt syscalls */
	if (siginterrupt (SIGALRM, 0) != 0)
		return -1;

	memset(&t, 0, sizeof(t));
	t.it_value.tv_usec = timer_resolution_us;
	t.it_value.tv_sec = 0;
	t.it_interval.tv_usec = timer_resolution_us;
	t.it_interval.tv_sec = 0;

	if (setitimer(ITIMER_REAL, &t, NULL) < 0) {
		printf("setitimer failed\n");
		return -1;
	}

	return 0;
}

void *hostsim_uart_stdin(void *arg)
{
	int n;
	char c;
	static char prev_c;

	(void)arg;

	/* read on old stdin and put it in pipe */
	while (1) {
		if (in_tail - in_head >= (RG_SIZE - 1)) {
			/* no more room, wait (should we drop ?) */
			usleep(1000);
			continue;
		}

		n = read(old_stdin, &c, 1);
		if (n <= 0)
			break;

		if (prev_c == '\x01' /* ctrl-a */ && c == 'q')
			hostsim_uart_exit();

		prev_c = c;

		in_buf[in_tail & RG_MASK] = c;
		in_tail++;

		write(stdin_pipe[1], &c, 1);
	}
	pthread_exit(NULL);
	return NULL;
}

void *hostsim_uart_stdout(void *arg)
{
	int n;
	char c;

	(void)arg;

	/* read on our pipe, and forward it to the old stdout */
	while (1) {
		if (out_tail - out_head >= (RG_SIZE - 1)) {
			/* no more room, wait (should we drop ?) */
			usleep(1000);
			continue;
		}

		n = read(stdout_pipe[0], &c, 1);
		if (n <= 0) {
			printf("read failed: %s\n", strerror(errno));
			break;
		}

		out_buf[out_tail & RG_MASK] = c;
		out_tail++;

		write(old_stdout, &c, 1);
	}
	pthread_exit(NULL);
	return NULL;
}

/* initialize hostsim framework for uart */
int hostsim_uart_init(void)
{
	struct termios term;
	int ret;

	printf("hostsim/uart: type ctrl-a + q to exit\n");

	tcgetattr(0, &oldterm);
	memcpy(&term, &oldterm, sizeof(term));
	term.c_lflag &= ~(ICANON | ECHO | ISIG);
	tcsetattr(0, TCSANOW, &term);

	setbuf(stdin, NULL);
	setbuf(stdout, NULL);

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

	ret = pthread_create(&in_pthread, NULL, hostsim_uart_stdin, NULL);
	if (ret) {
		printf("pthread_create() returned %d\n", ret);
		return -1;
	}
	ret = pthread_create(&out_pthread, NULL, hostsim_uart_stdout, NULL);
	if (ret) {
		printf("pthread_create() returned %d\n", ret);
		return -1;
	}

	return 0;
}

int hostsim_uart_exit(void)
{
	tcsetattr(old_stdin, TCSANOW, &oldterm);
	exit(0);
	return 0;
}
#endif /* HOST_VERSION */
