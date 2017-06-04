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
 *  Revision : $Id: menu.h,v 1.3.4.2 2007-05-23 17:18:15 zer0 Exp $
 *
 */

/* Droids-corp + Eirbot 2004 - Zer0 - tof - ta mere en short

   Standard interface for many keyboards
*/

/** \file kbd.h
    \brief this is the interface for all keyboard modules
    
    \todo write as many keyboards modules as possible, each device on earth who has a keyboard should be interfaced with our avr-modules
*/

/*
Module clavier
==============

* CDC

- Diffrents types de claviers au niveau hardware, que ce soit un
  petit clavier matriciel 4x4 ou un clavier de PC.
- Interface commune, c'est  dire qu'on veut pouvoir changer de
  hardware en conservant le mme code, dans la mesure du possible.
- Interface souple, c'est  dire facile  utiliser et avec
  possibilit de l'utiliser avec les fdevopen() de la libc AVR, mais
  aussi de manire plus simple.
- L'interface doit pouvoir permettre de coder simplement une sorte de
  "touche shift" : une touche qui n'ajoute rien dans le buffer de
  lecture du device mais qui change la signification des autres
  touches. 
*/

#ifndef _KBD_
#define _KBD_

#include <aversive.h>
#include <kbd_combination_config.h>




/** Initialisation function */
extern void kbd_init(void);


/** 
 * Return 0 if the key is NOT currently pressed, or 1 if
 * the key is pressed
 * key is the ascii value of the key without any shifts
 * this function can be called in a loop to test one key
 */
extern u08 kbd_key_state(kbd_type key);


/** 
 * returns the character corresponding to the actually active key
 */
kbd_type kbd_get_current_key(void)
{
	return g_current_key;
}

/** the needed event type */
enum kbd_event {KBD_RELEASED, KBD_PRESSED , KBD_REPEAT};
typedef enum kbd_event kbd_event;

/**
 * Add a function f(u08) which will be called on key event
 * 
 * the registered function is for example: 
 *   void event(u08 c, kbd_event event_type)
 * c is the key and event_type is KBD_RELEASED, KBD_PRESSED or KBD_REPEAT
 */
extern void kbd_register_event(void (*f)(kbd_type, kbd_event));


/** 
 * For each key event, call the function key_event(c, event_type)
 * has to be called periodically
 */ 
extern void kbd_manage(void);


#endif // _KBD_
