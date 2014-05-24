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

#include <stdarg.h>

#include <aversive/wait.h>

#include <callout.h>
#include <timer.h>
#include <uart.h>
#include <hostsim.h>

static struct callout_mgr global_cm;
static volatile uint16_t time_ref;

#define C1_TIME_MS 500
#define C2_TIME_MS 500
#define C3_TIME_MS 1000
#define C4_TIME_MS 4000

/* return the current time reference (the variable is incremented in the
 * interuption */
static uint16_t get_time_ms(void)
{
	uint8_t flags;
	uint16_t ret;

	IRQ_LOCK(flags);
	ret = time_ref;
	IRQ_UNLOCK(flags);
	return ret;
}

/* printf with a timestamp */
static void ts_printf(const char *fmt, ...)
{
	va_list ap;

	printf("%d: ", get_time_ms());
	va_start(ap, fmt);
	vfprintf(stdout, fmt, ap);
	va_end(ap);
}

static void f1(struct callout_mgr *cm, struct callout *tim, void *arg)
{
	(void)arg;
	ts_printf("%s\n", __FUNCTION__);

	callout_reschedule(cm, tim, C1_TIME_MS);
}

static void f2(struct callout_mgr *cm, struct callout *tim, void *arg)
{
	(void)arg;
	ts_printf("%s\n", __FUNCTION__);

	callout_reschedule(cm, tim, C2_TIME_MS);
}

static void f3(struct callout_mgr *cm, struct callout *tim, void *arg)
{
	(void)arg;
	ts_printf("%s START\n", __FUNCTION__);
	wait_ms(600);
	ts_printf("%s END\n", __FUNCTION__);

	callout_reschedule(cm, tim, C3_TIME_MS);
}

static void supp(struct callout_mgr *cm, struct callout *tim, void *arg)
{
	struct callout *c2 = arg;

	ts_printf("stopping c1\n");
	callout_stop(cm, c2);
}

static void timer_interrupt(void)
{
#ifndef HOST_VERSION
	static uint16_t cycles;

	cycles += 256 * 8; /* 8 bits timer + 8 divisor */
	if (cycles > (CONFIG_QUARTZ/1000)) {
		cycles -= (CONFIG_QUARTZ/1000);
		time_ref++;
	}
#else
	time_ref++;
#endif

	sei(); /* callout_manage() must be called with irq allowed */
	callout_manage(&global_cm);
}

static void dump_stats(char c)
{
	if (c != 's')
		return;

	callout_dump_stats(&global_cm);
}

int main(void)
{
	struct callout c1, c2, c3, c4;
	uint8_t old_prio;

#ifdef HOST_VERSION
	hostsim_uart_init();
	hostsim_ittimer_add(timer_interrupt, 1 * 1000 * 1000); /* 1ms period */
	hostsim_ittimer_enable(100); /* 100 us */
#else
	uart_init();
	fdevopen(uart0_dev_send, uart0_dev_recv);
	timer_init();
	timer0_register_OV_intr(timer_interrupt);
#endif
	uart_register_rx_event(0, dump_stats);

	callout_mgr_init(&global_cm, get_time_ms);
	sei();

	printf("f1 every %d ms, high prio (200)\n", C1_TIME_MS);
	printf("f2 every %d ms, low prio (50)\n", C2_TIME_MS);
	printf("f3 every %d ms, med prio (100), the function lasts 600ms\n",
		C3_TIME_MS);
	printf("f4 only once after %d ms, very high prio (250), "
		"stop task f2\n", C4_TIME_MS);
	printf("type s to dump stats\n");

	callout_init(&c1, f1, NULL, 200);
	callout_init(&c2, f2, NULL, 50);
	callout_init(&c3, f3, NULL, 100);
	callout_init(&c4, supp, &c2, 250);

	callout_schedule(&global_cm, &c1, C1_TIME_MS);
	callout_schedule(&global_cm, &c2, C2_TIME_MS);
	callout_schedule(&global_cm, &c3, C3_TIME_MS);
	callout_schedule(&global_cm, &c4, C4_TIME_MS);

	while (get_time_ms() < 2900)
		;

	old_prio = callout_mgr_set_prio(&global_cm, 150);
	ts_printf("set prio 150\n");

	while (get_time_ms() < 3100)
		;

	ts_printf("set prio 0\n");
	callout_mgr_restore_prio(&global_cm, old_prio);

	while (get_time_ms() < 5000)
		;

	callout_stop(&global_cm, &c1);
	callout_stop(&global_cm, &c3);
	callout_stop(&global_cm, &c4);

	callout_dump_stats(&global_cm);
	wait_ms(10);

#ifdef HOST_VERSION
	hostsim_uart_exit();
#endif

	return 0;
}
