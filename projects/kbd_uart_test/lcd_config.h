/* change these definitions to adapt setting */
// changed PIN in BIT to avoid confusion, don'use old version
#define LCD_PORT PORTC

/*  If the LCD module is a 1 line version with double addressing (8
    chars by segment), you need to define LCD_LINES to 2 and
    LCD_DOUBLE_ADDRESSING to 1. This is the case with for example
    SAMSUNG LTN 211 - N01.
*/

#define LCD_LINES           2     /* visible lines */
#define LCD_LINE_LENGTH  0x40     /* internal line length */
#define LCD_START_LINE1  0x00     /* DDRAM address of first char of line 1 */
#define LCD_START_LINE2  0x40     /* DDRAM address of first char of line 2 */
#define LCD_START_LINE3  0x14     /* DDRAM address of first char of line 3 */
#define LCD_START_LINE4  0x54     /* DDRAM address of first char of line 4 */

#define LCD_DOUBLE_ADDRESSING 1

#define LCD_DATA_PORT    LCD_PORT  /* port for 4bit data */
#define LCD_FIRST_DATA_BIT 3
#define LCD_RS_PORT      LCD_PORT  /* port for RS line */
#define LCD_RS_BIT       0
#define LCD_RW_PORT      LCD_PORT  /* port for RW line */
#define LCD_RW_BIT       1
#define LCD_E_PORT       LCD_PORT  /* port for Enable line */
#define LCD_E_BIT        2
