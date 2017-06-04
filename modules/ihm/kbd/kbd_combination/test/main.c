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
#include <stdio.h>

#include <aversive.h>
#include <aversive/wait.h>
#include <lcd.h>
#include <kbd.h>
#include <aversive/wait.h>



int main(void)
{
wait_ms(100);

lcd_init(LCD_DISP_ON);
// with printf :)
fdevopen(lcd_dev_putc,NULL);

kbd_init();

sei();


while(1)
	{
	
	printf_P(PSTR("\f%c"), kbd_get_current_key());
	wait_ms(50);


	kbd_manage();
	}



  return 0;
}


}
