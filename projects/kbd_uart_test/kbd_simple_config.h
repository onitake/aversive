#ifndef _KBD_CONFIG_H_
#define _KBD_CONFIG_H_

#define KBD_FIFO_SIZE 4

#define KBD_COL1ROW1 '1'
#define KBD_COL1ROW2 '4'
#define KBD_COL1ROW3 '7'
#define KBD_COL1ROW4 '*'
#define KBD_COL2ROW1 '2'
#define KBD_COL2ROW2 '5'
#define KBD_COL2ROW3 '8'
#define KBD_COL2ROW4 '0'
#define KBD_COL3ROW1 '3'
#define KBD_COL3ROW2 '6'
#define KBD_COL3ROW3 '9'
#define KBD_COL3ROW4 '#'


#define KBD_PORT PORTA
#define KBD_DDR DDRA
#define KBD_PIN PINA

#define KBD_COL1_BIT 4
#define KBD_COL2_BIT 5
#define KBD_COL3_BIT 6

#define KBD_ROW1_BIT 0
/* Implicit, don't define it
#define KBD_ROW2_BIT (KBD_ROW1_BIT+1)
#define KBD_ROW3_BIT (KBD_ROW1_BIT+2)
#define KBD_ROW4_BIT (KBD_ROW1_BIT+3)
*/


#endif
