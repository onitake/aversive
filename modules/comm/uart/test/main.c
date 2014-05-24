/*
 *  Copyright Droids Corporation, Microb Technology, Eirbot (2005)
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
 *  Revision : $Id: main.c,v 1.15.10.5 2008-12-27 16:29:08 zer0 Exp $
 *
 */
#include <aversive/wait.h>
#include <aversive/pgmspace.h>

#include <uart.h>

#include <stdio.h>

/* sending "pop" on cmdline uart resets the robot */
void emergency(char c)
{
	static uint8_t i = 0;

	printf("%c\n", c);

	if ((i == 0 && c == 'p') ||
	    (i == 1 && c == 'o') ||
	    (i == 2 && c == 'p'))
		i++;
	else if ( !(i == 1 && c == 'p') )
		i = 0;
	if (i == 3) {
#ifdef HOST_VERSION
		hostsim_uart_exit();
#endif
		reset();
	}
}

/*
 * This code sends a counter value to uart.
 */
int main(void)
{
	int i;

#ifdef HOST_VERSION
	hostsim_uart_init();
	hostsim_ittimer_enable(100000);
#endif

	/* initialize uart with the default parameters ( see
	 * uart_config.h ) */
	uart_init();
	uart_register_rx_event(0, emergency);

	/* enable interrupts */
	sei();

	/* send some single characters */
	for (i=0; i<10; i++) {
		uart_send(0, 'x');
		uart_send(0, '0' + i);
		wait_ms(100);
	}
	uart_send(0, '\n');

#ifndef HOST_VERSION
	/* now we want to do a printf : we must register the
	 * uart0_send as stdout. Here no receive function is
	 * specified. */
	fdevopen(uart0_dev_send, NULL);
#endif

	/** ready to do a nice printf on the uart */
	printf("Uart is cool !!\n");

	/* one drawback of the previous printf is that the format
	 * chain is stored in RAM and this can take a huge size if
	 * there are many printf. To avoid this problem, please use
	 * printf_P together with PSTR, like in the next example. */
	while (1) {
		printf_P(PSTR("This format string takes no RAM "
			      "space. %i\n"), i++);
		wait_ms(1000);
	}

	return 0;
}
