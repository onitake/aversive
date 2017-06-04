/*
	config file for kbd type "COMB"
*/

#ifndef _KBD_CONFIG_
#define _KBD_CONFIG_   _COMB // only for 4 ports type keyboards


//do not change for this line please
typedef u08 kbd_type;

// IO ports, first part
#define KBD0_PORT PORTB
#define KBD0_BIT0 3 // number of the first bit of your HW
#define KBD0_SIZE 5 // number of bits concerned
#define KBD0_INVERSE // this has to be commented if the keyboard is not inverted

// IO ports, second port (comment this if not used)
#define KBD1_PORT PORTC
#define KBD1_BIT0 2 // number of the first bit of your HW
#define KBD1_SIZE 6 // number of bits concerned
#define KBD1_INVERSE // this has to be commented if the keyboard is not inverted

// IO ports, second port (comment this if not used)
#define KBD2_PORT PORTD
#define KBD2_BIT0 7 // number of the first bit of your HW
#define KBD2_SIZE 1 // number of bits concerned
#define KBD2_INVERSE // this has to be commented if the keyboard is not inverted

// IO ports, second port (comment this if not used)
//#define KBD3_PORT PORTA
#define KBD3_BIT0 2 // number of the first bit of your HW
#define KBD3_SIZE 6 // number of bits concerned
#define KBD3_INVERSE // this has to be commented if the keyboard is not inverted

// define this for anabling pullups
#define KBD_ENABLE_PULLUPS 



// rebound timer to filter the keys. Response ime will be this value multiplied by the calling period of kbd_manage()
#define KBD_NOREBOUND_TIMER 3



// key map to define.
// use (1<<position) for simple keys
// or muxed value for complex schemes
// attention : value has already been shifted to the first bit.
#define KBD0_MAP                     \
                  KBD_KEY((1<<0), '.') \
                  KBD_KEY((1<<1), 'S') /* "no speaker" */\
                  KBD_KEY((1<<2), 'V') /* "v/k" */\
                  KBD_KEY((1<<3), 'D') /* "kanal -" */\
                  KBD_KEY((1<<4), 'U') /* "kanal +" */\


#define KBD1_MAP                     \
                  KBD_KEY((1<<0), '1') /* "T1" */\
                  KBD_KEY((1<<1), '2') /* "T2" */\
                  KBD_KEY((1<<2), 'F') \
                  KBD_KEY((1<<3), 'R') /* "Resistor" */\
                  KBD_KEY((1<<4), '+') \
                  KBD_KEY((1<<5), '-') \


#define KBD2_MAP                     \
                  KBD_KEY((1<<0), 'N') /* "T1" */\



//#define KBD3_MAP                     \
//                  KBD_KEY((1<<0), '1') /* "T1" */\



#define KBD_NO_KEY ' ' // has to be different than all other values



#endif

