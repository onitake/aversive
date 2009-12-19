#include <stdio.h>
#include <string.h>

#include <base/fifo/fifo.h>
#include <comm/uart/uart.h>
#include <other/kbd/kbd.h>
#include <base/wait/wait.h>
#include <time/scheduler/scheduler.h>
#include <time/hour/hour.h>
//#include <other/menu/menu.h>
#include <other/lcd/lcd.h>

uint8_t global=16;
FILE * lcd;



void process(uint8_t c)
{
  char tab[16];
  uint16_t nb;
  int add;

  tab[0] = 0;

  uart0_send(c);

  if(c=='\r')
    {
      nb=scanf("%s %d",tab,&add);

      if( !strcmp(tab,"set"))
	{
	  time_set(add,0);
	  printf("\r\n>> time set to %d\r\n",add);
	}
      else if (!strcmp(tab,"get"))
	printf("\r\n>> %d secondes\r\n",time_get_s());
      else if (!strcmp(tab,"show"))
	printf("\r\n>> Value at 0x%X : %d\r\n",add,*(uint8_t *)add); 
      else
	printf("\r\n>> command not found\r\n");
    }
}




/* void menu_print(void) */
/* { */
/*   int8_t tmp=menu_current(); */

/*   menu_init_brother_list(); */

/*   lcd_clrscr(); */
/*   while(tmp != 0) */
/*     { */
/*       fprintf(lcd,"|"); */
/*       tmp=menu_up(tmp); */
/*     } */
/*   fprintf(lcd,"%s%s",(menu_is_leaf(menu_current())?"* ":"> "),menu_txt(menu_current())); */

/* } */



/* void menu_control(uint8_t c) */
/* { */
/*   if(c == '0') */
/*     menu_set(menu_left(menu_current())); */
/*   else if(c=='8') */
/*     menu_set(menu_right(menu_current())); */
/*   else if(c=='*') */
/*     { */
/*       if(menu_action_is_allowed()) */
/* 	menu_action_disallow(); */
/*       else */
/* 	menu_set(menu_up(menu_current()));	 */
/*     } */
/*   else if(c=='#') */
/*     { */
/*       if(menu_is_leaf(menu_current())) */
/* 	menu_action_allow(); */
/*       else */
/* 	menu_set(menu_down(menu_current())); */
/*     } */

/*   menu_print(); */
/* } */

FIFO_DECLARE_TYPE(g_read_fifo, uint8_t, 16);
FIFO_DEFINE(g_read_fifo);
FIFO_DEFINE_FUNCTIONS(uint8_t);

int kbd_get_next(void)
{
  uint8_t c;
  
  if(FIFO_IS_FULL(g_read_fifo))
    return -1;

  FIFO_DEL_ELT(&c, g_read_fifo);  

  return c;
}


/* void menu_add_char(uint8_t c) */
/* { */
/*   int val; */

/*   // exit */
/*   if(c=='*') */
/*     { */
/*       FIFO_ADD_ELT(' ', g_read_fifo, uint8_t); */
/*       menu_action_disallow(); */
/*       kbd_register_event(menu_control); */
/*       fscanf(lcd,"%d",&val); */
/*       time_set(val,0); */
/*       printf("\r\nLCD: time set to %d\r\n",val); */
/*       menu_print(); */
/*       return; */
/*     } */

/*   lcd_putc(c); */
/*   FIFO_ADD_ELT(c, g_read_fifo, uint8_t); */
/* } */




// PE1 PE3 PB3 PB4
void leds(void)
{
  static uint8_t a=0;

  PORTB=a++;
}

int main(void)
{
  /* LEDS */
  DDRB=0x18;

  uart_init();  
  //  kbd_init();
  scheduler_init();
  //  lcd_init(LCD_DISP_ON);
  //  menu_init();

  FIFO_INIT(g_read_fifo, uint8_t, 16);

  //  scheduler_add_periodical_event(leds, 20000l/SCHEDULER_UNIT);
  /* ajoute la scrutation du clavier */
  //  scheduler_add_periodical_event(kbd_manage, 100);
  
  /* envoie les caracteres du clavier vers le LCD */
  //  kbd_register_event(menu_control);

  /* appele la fonction process a chaque reception de caractere */
  //  uart0_register_rx_event(process);

  /* creation du device uart */
  fdevopen(uart0_dev_send,uart0_dev_recv);

  /* creation du device lcd */
  //  lcd=fdevopen(lcd_dev_putc,kbd_dev_get_next);

  sei();

  time_init();
  //  menu_print();
  time_set(10,0);
/*   printf_P(PSTR("\r\nWelcome to this demo\r\n")); */
/*   printf_P(PSTR("\r\n")); */
/*   printf_P(PSTR("                                                          \r\n")); */
/*   printf_P(PSTR("                      .__,.      ___.                     \r\n")); */
/*   printf_P(PSTR("                   _%i~`            -'i;_                 \r\n")); */
/*   printf_P(PSTR("                 _=Xr~                '{a__               \r\n")); */
/*   printf_P(PSTR("               <ln2                    :|2S=;             \r\n")); */
/*   printf_P(PSTR("              _xnxn                    :=oox>.            \r\n")); */
/*   printf_P(PSTR("             .nnvno      ..______..    :=oonss            \r\n")); */
/*   printf_P(PSTR("             .onvnn_. __s>ss%xixaii_,_ =Invv1n            \r\n")); */
/*   printf_P(PSTR("             :vnnnnns_+''^~-` ---^^'(;sxvnnnn1            \r\n")); */
/*   printf_P(PSTR("           .__onvnvnxn_,.          __svvvnnnnv_,          \r\n")); */
/*   printf_P(PSTR("       _i%vnn2nvnnvnvn2n2nss;.;<aIvo2nxxnnvnx1nnnas_.     \r\n")); */
/*   printf_P(PSTR("      i)nnvnnnnxnvnnnnvnnnn1(-=]SIvnnvnnvnnnnnnnnnnx>_    \r\n")); */
/*   printf_P(PSTR("    =innonn|=;::+<innnvvvvv    .:o2vvo2oo===;==)vnvvn2:.  \r\n")); */
/*   printf_P(PSTR("   _Jn!^~`    <xs;. -^11oc+      **v}!-` i<a;.   --'{lu(  \r\n")); */
/*   printf_P(PSTR("   Xc|        Ixoc|       .:   .:        vvn(.        nc| \r\n")); */
/*   printf_P(PSTR(" ..v;         ~)n1x     .)n21vvnoIl    ._XuI'         ~<i \r\n")); */
/*   printf_P(PSTR("  .i;          =ix2v;    =innnnnn=+   :)onv.           =i \r\n")); */
/*   printf_P(PSTR("   i;            -'o1v_,.=vnvnvxo+| =snxI~~            -+ \r\n")); */
/*   printf_P(PSTR("   +:              -)ol=;)nnvvnnvvv:)n+;              .   \r\n")); */
/*   printf_P(PSTR("                      .<xxnvnvv1nnn<,                .    \r\n")); */
/*   printf_P(PSTR("      ===.          :iooonnnnno2ovvn1l             |;     \r\n")); */
/*   printf_P(PSTR("        --'=<__s_asxuonx1}|^`---'**nnnnnaii_s__/+^^       \r\n")); */
/*   printf_P(PSTR("              ==+==;=;               .=;;::=..           \r\n")); */
/*   printf_P(PSTR("\r\n")); */
/*   printf_P(PSTR("                                      Microb Technology\r\n")); */
/*   printf_P(PSTR("\r\n")); */
//  printf("\r\n>> Time set to %d seconds\r\n",time_get_s());
  
  while(1)
    {
      
/*       wait_ms(1); */
/*       uart0_send(0x00); */
/*       wait_ms(1); */
/*       uart0_send(0x03); */
/*       wait_ms(1); */
/*       uart0_send(0xAA); */
/*       wait_ms(1); */
/*       uart0_send(0x55); */
/*       wait_ms(1); */
/*       uart0_send(0xFF); */
/*       wait_ms(1); */
/*       wait_ms(1); */
      printf("\r\n>> Time set to %d seconds\r\n",time_get_s());
      //      menu_action();
    }
  return 0;
}


