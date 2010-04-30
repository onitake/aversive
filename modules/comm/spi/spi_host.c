/*  
 *  Copyright Droids Corporation (2008)
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
 */

/*
 * Host wrapper for SPI
 */

#include <aversive.h>
#include <aversive/parts.h>
#include <aversive/error.h>

#include <stdlib.h>
#include <string.h>

#include <spi.h>
#include <spi_config.h>

/* global vars */
static volatile uint8_t g_ss_number;
static volatile spi_mode_t g_spi_mode;
static volatile uint8_t g_slave_selected;
static volatile uint8_t g_spi_order;
static volatile spi_format_t g_spi_format;
static volatile spi_mode_t g_spi_mode;
static volatile spi_clk_rate_t g_spi_clk_rate;

/*
 * Register a pin as SS line
 * Returns a unique identifier, or -1 on error
 * There is always the physical SS line registered as 0
 */
int8_t spi_register_ss_line(volatile uint8_t *port, uint8_t bitnum)
{
	DEBUG(E_SPI, "Trying to register new SS line: port 0x%x, bitnum %d", port, bitnum);
	/* too much SS lines (try to change SPI_MAX_SLAVES) */
	if (g_ss_number >= SPI_MAX_SLAVES+1)
		return -1;

	NOTICE(E_SPI, "New Slave Line registered: %d", g_ss_number);
	return g_ss_number++;
}


/*
 *	Set data order (default: MSB first)
 */
void spi_set_data_order(uint8_t order)
{
	g_spi_order = order;
}

/*
 *	Get data order
 */
uint8_t spi_get_data_order(void)
{
	return g_spi_order;
}


/*
 *	Initialize SPI
 */
void spi_init(spi_mode_t mode, spi_format_t format, spi_clk_rate_t clk_rate)
{
	NOTICE(E_SPI, "Init SPI: mode %d, format %d, clk_rate %d",
	       mode, format, clk_rate);

	/* SS pin is not driven by SPI hardware 
	 * This is taken care of by spi_register_ss_line()
	 * EVEN for the "default" SS line */
	g_ss_number = 0;
	g_spi_format = format;
	g_spi_clk_rate = clk_rate;
	g_slave_selected = FALSE;
	g_spi_mode = SPI_MODE_MASTER;
	NOTICE(E_SPI, "Init done");
}

/*
 *	Returns the state of SPI
 */
inline spi_mode_t spi_get_mode(void)
{
	return g_spi_mode;
}

/*
 *	Send a byte (and receive one)
 *	Returns the received byte
 */
uint8_t spi_send_and_receive_byte(uint8_t byte)
{
	/* XXX */
	return 0;
}

/*
 *	Send a byte, discard the result
 */
inline void spi_send_byte(uint8_t byte)
{
	spi_send_and_receive_byte(byte);
}

/*
 *	Receives a byte (sends a NULL)
 */
uint8_t spi_receive_byte(void)
{
	return spi_send_and_receive_byte(0x00);
}

/*
 *	Activates the selected SS line
 */
uint8_t spi_slave_select(uint8_t slave)
{
	if (g_slave_selected) {
		ERROR(E_SPI, "A slave is already selected !");
		return EBUSY;
	}

	/* XXX */
	g_slave_selected = TRUE;
	return ESUCCESS;
}

/*
 *	Desactivates the selected SS line
 */
void spi_slave_deselect(uint8_t slave)
{
	/* XXX */
	g_slave_selected = FALSE;
}

/*
 *	Display SS lines
 */
void spi_display_ss_lines(void)
{
}
