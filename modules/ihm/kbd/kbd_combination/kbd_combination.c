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
 *  Revision : $Id$
 *
 */

/** \file kbd_combination.h
    \brief keyboard module for kbds that produce a logical combination on ports
    
    with this kbd module you can interface kbds that produce a logical combination on ports.
    This can be a matrix keyboard with decoding HW, or a few simple pushbuttons.
*/

#include <avr/io.h>
#include <aversive.h>
#include <avr/pgmspace.h>
#include <stdlib.h> 

#include <kbd.h>

// config checking
#if ( _KBD_CONFIG_H_ != _COMB )
#error you have the wrong config file for your module
#endif



// mask
#define KBD0_MASK ( (0xFF >> (8 - KBD0_SIZE))  << KBD0_BIT0 )
#define KBD1_MASK ( (0xFF >> (8 - KBD1_SIZE))  << KBD1_BIT0 )
#define KBD2_MASK ( (0xFF >> (8 - KBD2_SIZE))  << KBD2_BIT0 )
#define KBD3_MASK ( (0xFF >> (8 - KBD3_SIZE))  << KBD3_BIT0 )




/** permet de s'assurer de la non imbrication d'un morceau de code dans lui meme */
#define EXEC_PROTECT()  do {		\
    static u08 exec_status = 0;	        \
    if ( exec_status == 0 )             \
      {                                 \
       exec_status = 1;                 

#define EXEC_UNPROTECT()                \
       exec_status = 0;                 \
      }                                 \
  }   while ( 0 )





volatile kbd_type g_current_key = KBD_NO_KEY;

void (*event)(kbd_type, kbd_event) = NULL;




/** init function */
void kbd_init(void)
{
  u08 flags;
#ifdef KBD_ENABLE_PULLUPS
  IRQ_LOCK(flags);


#ifdef KBD0_PORT
  KBD0_PORT |= KBD0_MASK;
#endif
#ifdef KBD1_PORT
  KBD1_PORT |= KBD1_MASK;
#endif
#ifdef KBD2_PORT
  KBD2_PORT |= KBD2_MASK;
#endif
#ifdef KBD3_PORT
  KBD3_PORT |= KBD3_MASK;
#endif

  IRQ_UNLOCK(flags);
#endif
}


/** 
 * Return 0 if the key is NOT currently pressed, or 1 if
 * the key is pressed
 * key is the ascii value of the key without any shifts
 * this function can be called in a loop to test one key
 */
u08 kbd_key_state(kbd_type key)
{
	if(g_current_key == key)
		return 1;
	else
		return 0;
}

/** 
 * returns the character corresponding to the actually active key
 */
kbd_type kbd_get_current_key(void)
{
	return g_current_key;
}



/**
 * Add a function f(u08) which will be called on key event
 * 
 * the registered function is for example: 
 *   void event(u08 c, kbd_event event_type)
 * c is the key and event_type is KBD_RELEASED, KBD_PRESSED or KBD_REPEAT
 */
void kbd_register_event(void (*f)(kbd_type,kbd_event))
{
	u08 flags;
	IRQ_LOCK(flags);
	event = f ;
	IRQ_UNLOCK(flags);
}


/** returns the actual pressed key, internal function */
static inline kbd_type kbd_get_key(void)
{

 
	u08 port;

	#ifdef KBD0_PORT
		port = PIN(KBD0_PORT);
		#ifdef KBD0_INVERSE
			port = ~port;
		#endif	
		port = (port  & KBD0_MASK)>> KBD0_BIT0;

		/* code gneneration */
			#define KBD_KEY(val, char)     \
		if (port == val)               \
			return char;                   \


			KBD0_MAP

			#undef KBD_KEY

	#endif


	#ifdef KBD1_PORT
		port = PIN(KBD1_PORT);
		#ifdef KBD1_INVERSE
			port = ~port;
		#endif	
		port = (port  & KBD1_MASK)>> KBD1_BIT0;

		/* code gneneration */
			#define KBD_KEY(val, char)     \
		if (port == val)               \
			return char;                   \


			KBD1_MAP

			#undef KBD_KEY

	#endif

	#ifdef KBD2_PORT
		port = PIN(KBD2_PORT);
		#ifdef KBD2_INVERSE
			port = ~port;
		#endif	
		port = (port  & KBD2_MASK)>> KBD2_BIT0;

		/* code gneneration */
			#define KBD_KEY(val, char)     \
		if (port == val)               \
			return char;                   \


			KBD2_MAP

			#undef KBD_KEY

	#endif


	#ifdef KBD3_PORT
		port = PIN(KBD3_PORT);
		#ifdef KBD3_INVERSE
			port = ~port;
		#endif	
		port = (port  & KBD3_MASK)>> KBD3_BIT0;

		/* code gneneration */
			#define KBD_KEY(val, char)     \
		if (port == val)               \
			return char;                   \


			KBD3_MAP

			#undef KBD_KEY

	#endif
	return KBD_NO_KEY;
}




/** manages the keyboard state machine */
void kbd_manage(void)
{
	kbd_type value;

	static u08 s_timer = 0;

   
	// test kbd
	value = kbd_get_key();


	// setting anti rebound timer on change
	if (s_timer == 0)
		{
		if ( value != g_current_key)
			s_timer = KBD_NOREBOUND_TIMER;
		}
		
	// decreasing anti rebound timer
	else if( s_timer > 1 )
		s_timer -- ;
	
	// end of count, we only take in account the new value here
	else if( s_timer == 1)  
		{

		// resetting timer
		s_timer = 0;

		// calling the event
		if ( (event) && (value != g_current_key) )
			{
			EXEC_PROTECT();  // no imbricated calls

			if(g_current_key != KBD_NO_KEY)
				event(g_current_key, KBD_RELEASED); // PREVIOUS key released
			if(value != KBD_NO_KEY)
				event(value, KBD_PRESSED); // key pressed

			EXEC_UNPROTECT();
			}
		
		// changing stored value
		g_current_key = value;

		}

}


