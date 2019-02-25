/**************************************************************
 *	Hitachi HD44780 LCD routines for AVR
 *						-Er.Sapin Bajracharya, October 2012
 *                      -edited by Rabin Giri
 **************************************************************
 *	The DDR bits associated with the various lines
 *	(LCD_EN, LCD_RS, LCD_D7..LCD_D0, LCD_DATA, LCD_CLK)
 *	must be set (output mode) in the main program 
 *	prior to calling the lcd_init() function.
 **************************************************************
 *		Uses 6 lines:
 *		LCD_EN		:	STROBE 			:	Strobe for LCD/LCD_EN
 *		LCD_RS		:	RegisterSelect	:	R/S line 
 *		LCD_D7...D4	:	Data lines		:	D7 to D4 lines 
 **************************************************************

	Available functions:
	lcd_init();
		LCD initialization function
		MUST be called first, after associated DDR bits
		have been configured as outputs
	lcd_putch(char c);
		Displays supplied character at the current position
	lcd_puts(const char *s);
		Displays supplied string(character array) starting
		at current position
	lcd_clear();
		Clears and homes the display
	lcd_goto(uint8_t pos)
		Sets cursor location to given address (0-127)
	lcd_unum(uint16_t num)
		Displays unsigned integer (0 - 65535)
	lcd_unum3(uint16_t num)
		Displays unsigned number, upto 3 digits(0-999)
	lcd_unum_hex(uint16_t num)
		Displays 16bit number in 4digit Hex
	lcd_cmd(uint8_t c);
		Writes supplied byte to LCD with RS = 0 (command)
	lcd_dat(uint8_t c);
		Writes supplied byte to LCD with RS = 1 (data)
 *************************************************************/
#ifndef F_CPU
#define F_CPU 16000000UL
#endif


#define LCD_EN_PORT	PORTA
#define LCD_EN_PIN	2 
	
#define	LCD_RS_PORT PORTA		//R/S line
#define	LCD_RS_PIN	3

#define LCD_D7_PORT PORTA
#define LCD_D7_PIN	7
#define LCD_D6_PORT PORTA
#define LCD_D6_PIN	6
#define LCD_D5_PORT PORTA
#define LCD_D5_PIN	5
#define LCD_D4_PORT PORTA
#define LCD_D4_PIN	4



/*************************************************/
/*****DO NOT MODIFY ANYTHING BELOW THIS POINT*****/
/*************************************************/


#include <util/delay.h>

#include <avr/pgmspace.h>

#include <stdarg.h>
#include <stdlib.h>


#define DDR(x) (*(&x - 1))      /* address of data direction register of port x */

/* instruction register bit positions, see HD44780U data sheet */
#define LCD_CLR               0      /* DB0: clear display                  */

#define LCD_HOME              1      /* DB1: return to home position        */

#define LCD_ENTRY_MODE        2      /* DB2: set entry mode                 */
#define LCD_ENTRY_INC         1      /*   DB1: 1=increment, 0=decrement     */
#define LCD_ENTRY_SHIFT       0      /*   DB2: 1=display shift on           */

#define LCD_ON                3      /* DB3: turn lcd/cursor on             */
#define LCD_ON_DISPLAY        2      /*   DB2: turn display on              */
#define LCD_ON_CURSOR         1      /*   DB1: turn cursor on               */
#define LCD_ON_BLINK          0      /*     DB0: blinking cursor ?          */

#define LCD_MOVE              4      /* DB4: move cursor/display            */
#define LCD_MOVE_DISP         3      /*   DB3: move display (0-> cursor) ?  */
#define LCD_MOVE_RIGHT        2      /*   DB2: move right (0-> left) ?      */

#define LCD_FUNCTION          5      /* DB5: function set                   */
#define LCD_FUNCTION_8BIT     4      /*   DB4: set 8BIT mode (0->4BIT mode) */
#define LCD_FUNCTION_2LINES   3      /*   DB3: two lines (0->one line)      */
#define LCD_FUNCTION_10DOTS   2      /*   DB2: 5x10 font (0->5x7 font)      */

#define LCD_CGRAM             6      /* DB6: set CG RAM address             */

#define LCD_DDRAM             7      /* DB7: set DD RAM address             */

#define LCD_BUSY              7      /* DB7: LCD is busy                    */

/* set entry mode: display shift on/off, dec/inc cursor move direction */
#define LCD_ENTRY_DEC            0x04   /* display shift off, dec cursor move dir */ //((1<<LCD_ENTRY_MODE) | (0<<LCD_ENTRY_INC) | (0<<LCD_ENTRY_SHIFT))
#define LCD_ENTRY_DEC_SHIFT      0x05   /* display shift on,  dec cursor move dir */ //((1<<LCD_ENTRY_MODE) | (0<<LCD_ENTRY_INC) | (1<<LCD_ENTRY_SHIFT))
#define LCD_ENTRY_INC_           0x06   /* display shift off, inc cursor move dir */ //((1<<LCD_ENTRY_MODE) | (1<<LCD_ENTRY_INC) | (0<<LCD_ENTRY_SHIFT))
#define LCD_ENTRY_INC_SHIFT      0x07   /* display shift on,  inc cursor move dir */ //((1<<LCD_ENTRY_MODE) | (1<<LCD_ENTRY_INC) | (1<<LCD_ENTRY_SHIFT))

/* display on/off, cursor on/off, blinking char at cursor position */
#define LCD_DISP_OFF             0x08   /* display off                            */ //((1<<LCD_ON) | (0<<LCD_ON_DISPLAY) | (0<<LCD_ON_CURSOR) | (0<<LCD_ON_BLINK))
#define LCD_DISP_ON              0x0C   /* display on, cursor off                 */ //((1<<LCD_ON) | (1<<LCD_ON_DISPLAY) | (0<<LCD_ON_CURSOR) | (0<<LCD_ON_BLINK))
#define LCD_DISP_ON_BLINK        0x0D   /* display on, cursor off, blink char     */ //((1<<LCD_ON) | (1<<LCD_ON_DISPLAY) | (0<<LCD_ON_CURSOR) | (1<<LCD_ON_BLINK))
#define LCD_DISP_ON_CURSOR       0x0E   /* display on, cursor on                  */ //((1<<LCD_ON) | (1<<LCD_ON_DISPLAY) | (1<<LCD_ON_CURSOR) | (0<<LCD_ON_BLINK))
#define LCD_DISP_ON_CURSOR_BLINK 0x0F   /* display on, cursor on, blink char      */ //((1<<LCD_ON) | (1<<LCD_ON_DISPLAY) | (1<<LCD_ON_CURSOR) | (1<<LCD_ON_BLINK))

/* move cursor/shift display */
#define LCD_MOVE_CURSOR_LEFT     0x10   /* move cursor left  (decrement)          */ //((1<<LCD_MOVE) | (0<<LCD_MOVE_DISP) | (0<<LCD_MOVE_RIGHT))
#define LCD_MOVE_CURSOR_RIGHT    0x14   /* move cursor right (increment)          */ //((1<<LCD_MOVE) | (0<<LCD_MOVE_DISP) | (1<<LCD_MOVE_RIGHT))
#define LCD_MOVE_DISP_LEFT       0x18   /* shift display left                     */ //((1<<LCD_MOVE) | (1<<LCD_MOVE_DISP) | (0<<LCD_MOVE_RIGHT))
#define LCD_MOVE_DISP_RIGHT      0x1C   /* shift display right                    */ //((1<<LCD_MOVE) | (1<<LCD_MOVE_DISP) | (1<<LCD_MOVE_RIGHT))

/* function set: set interface data length and number of display lines */
#define LCD_FUNCTION_4BIT_1LINE  0x20   /* 4-bit interface, single line, 5x7 dots */ //((1<<LCD_FUNCTION) | (0<<LCD_FUNCTION_8BIT) | (0<<LCD_FUNCTION_2LINES) | (0<<LCD_FUNCTION_10DOTS))
#define LCD_FUNCTION_4BIT_2LINES 0x28   /* 4-bit interface, dual line,   5x7 dots */ //((1<<LCD_FUNCTION) | (0<<LCD_FUNCTION_8BIT) | (1<<LCD_FUNCTION_2LINES) | (0<<LCD_FUNCTION_10DOTS))
#define LCD_FUNCTION_8BIT_1LINE  0x30   /* 8-bit interface, single line, 5x7 dots */ //((1<<LCD_FUNCTION) | (1<<LCD_FUNCTION_8BIT) | (0<<LCD_FUNCTION_2LINES) | (0<<LCD_FUNCTION_10DOTS))
#define LCD_FUNCTION_8BIT_2LINES 0x38   /* 8-bit interface, dual line,   5x7 dots */ //((1<<LCD_FUNCTION) | (0<<LCD_FUNCTION_8BIT) | (1<<LCD_FUNCTION_2LINES) | (0<<LCD_FUNCTION_10DOTS))


#define LCD_MODE_DEFAULT     ((1<<LCD_ENTRY_MODE) | (1<<LCD_ENTRY_INC) )


#define	LCD_STROBE()	((LCD_EN_PORT |= (1 << LCD_EN_PIN)),(LCD_EN_PORT &= ~(1 << LCD_EN_PIN)))




static void lcd_write(uint8_t c)
{
	_delay_us(40);
	//MS nibble
	if(c & 0x80)
		LCD_D7_PORT |=  (1 << LCD_D7_PIN);
	else
		LCD_D7_PORT &= ~(1 << LCD_D7_PIN);
	
	if(c & 0x40)
		LCD_D6_PORT |=  (1 << LCD_D6_PIN);
	else
		LCD_D6_PORT &= ~(1 << LCD_D6_PIN);
	
	if(c & 0x20)
		LCD_D5_PORT |=  (1 << LCD_D5_PIN);
	else
		LCD_D5_PORT &= ~(1 << LCD_D5_PIN);
		
	if(c & 0x10)
		LCD_D4_PORT |=  (1 << LCD_D4_PIN);
	else
		LCD_D4_PORT &= ~(1 << LCD_D4_PIN);
	
	LCD_STROBE();
	
	//LS nibble
	if(c & 0x08)
		LCD_D7_PORT |=  (1 << LCD_D7_PIN);
	else
		LCD_D7_PORT &= ~(1 << LCD_D7_PIN);
		
	if(c & 0x04)
		LCD_D6_PORT |=  (1 << LCD_D6_PIN);
	else
		LCD_D6_PORT &= ~(1 << LCD_D6_PIN);
		
	if(c & 0x02)
		LCD_D5_PORT |=  (1 << LCD_D5_PIN);
	else
		LCD_D5_PORT &= ~(1 << LCD_D5_PIN);
		
	if(c & 0x01)
		LCD_D4_PORT |=  (1 << LCD_D4_PIN);
	else
		LCD_D4_PORT &= ~(1 << LCD_D4_PIN);
	
	LCD_STROBE();
}




//Write command to LCD
void lcd_cmd(uint8_t cmd)
{
	LCD_RS_PORT &= ~(1 << LCD_RS_PIN);
	lcd_write(cmd);
}

//Write character data to LCD
void lcd_dat(uint8_t dat)
{
	LCD_RS_PORT |= (1 << LCD_RS_PIN);
	lcd_write(dat);
}


/*
 * 	Clear and home the LCD
 */

void lcd_clear(void)
{
	LCD_RS_PORT &= ~(1 << LCD_RS_PIN);
	lcd_write(1<<LCD_CLR);
	_delay_ms(2);
}

/* write a string of chars to the LCD */

void lcd_puts(const char * s)
{
	LCD_RS_PORT |= (1 << LCD_RS_PIN);	// write characters
	while(*s)
		lcd_write(*s++);
}

#define lcd_puts_P(__s)         lcd_puts_p(PSTR(__s))

/* print string from program memory on lcd (no auto linefeed) */

void lcd_puts_p(const char *progmem_s)
{
    register char c;
	
	LCD_RS_PORT |= (1 << LCD_RS_PIN);	// write characters
    while ( (c = pgm_read_byte(progmem_s++)) ) 
        lcd_write(c);
}
/* write one character to the LCD */

void lcd_putch(char c)
{
	LCD_RS_PORT |= (1 << LCD_RS_PIN);	// write characters
	lcd_write( c );
}

/*
 * Go to the specified position
 */

void lcd_goto(unsigned char pos)
{
	LCD_RS_PORT &= ~(1 << LCD_RS_PIN);
	lcd_write(0x80|pos);
}
	


void lcd_init()
{
	DDR(LCD_RS_PORT) |= (1 << LCD_RS_PIN);
	DDR(LCD_EN_PORT) |= (1 << LCD_EN_PIN);
	
	DDR(LCD_D7_PORT) |= (1 << LCD_D7_PIN);
	DDR(LCD_D6_PORT) |= (1 << LCD_D6_PIN);
	DDR(LCD_D5_PORT) |= (1 << LCD_D5_PIN);
	DDR(LCD_D4_PORT) |= (1 << LCD_D4_PIN);
	
	LCD_RS_PORT &= ~(1 << LCD_RS_PIN);
	LCD_EN_PORT &= ~(1 << LCD_EN_PIN);
	
	_delay_ms(15);	// wait 15mSec after power applied,

	LCD_D4_PORT |= (1 << LCD_D4_PIN);//0x3 & 0x01;				//bit0 000X
	LCD_D5_PORT |= (1 << LCD_D5_PIN);//(0x3>>1) & 0x01;		//bit1 00XY -> 000X
	LCD_D6_PORT &= ~(1 << LCD_D6_PIN);//(0x3>>2) & 0x01;		//bit2 0XYZ -> 000X
	LCD_D7_PORT &= ~(1 << LCD_D7_PIN);//(0x3>>3) & 0x01;		//bit3 XYZW -> 000X
	
	LCD_STROBE();
	_delay_ms(5);
	LCD_STROBE();
	_delay_us(200);
	LCD_STROBE();
	_delay_us(200);
	
	// Four bit mode 
	LCD_D4_PORT &= ~(1 << LCD_D4_PIN);	//2 & 0x01
	LCD_D5_PORT |=  (1 << LCD_D5_PIN);	//(2>>1) & 0x01
	LCD_D6_PORT &= ~(1 << LCD_D6_PIN);	//(2>>2) & 0x01
	LCD_D7_PORT &= ~(1 << LCD_D7_PIN);	//(2>>3) & 0x01
	
	LCD_STROBE();

	lcd_write(0x28);		// Set interface length: nibblemode, 2line, 5x7dot
	lcd_write(0b00001100);	// Display On, Cursor Off, Cursor Blink off
	lcd_clear();			// Clear screen
	lcd_write(0x6);			// Set entry Mode : increment, displayShiftOff
}




const char PROGMEM chex[] =			{	'0', '1', '2', '3', '4', '5', '6', '7',
									'8', '9', 'A', 'B', 'C', 'D', 'E', 'F'  };
const char PROGMEM chex_wBlanking[] = {	' ', '1', '2', '3', '4', '5', '6', '7',
									'8', '9', 'A', 'B', 'C', 'D', 'E', 'F'  };

//without blanking of leading zeros
void lcd_unum_hex(uint16_t num)
{
	lcd_putch(pgm_read_byte(&chex[(num>>12)&0x0F]));
	lcd_putch(pgm_read_byte(&chex[(num>>8)&0x0F]));
	lcd_putch(pgm_read_byte(&chex[(num>>4)&0x0F]));
	lcd_putch(pgm_read_byte(&chex[num&0x0F]));
}

void lcd_unum_hex_wBlanking(uint16_t num)
{
	lcd_putch(pgm_read_byte(&chex_wBlanking[(num>>12)&0x0F]));
	lcd_putch(pgm_read_byte(&chex_wBlanking[(num>>8)&0x0F]));
	lcd_putch(pgm_read_byte(&chex_wBlanking[(num>>4)&0x0F]));
	lcd_putch(pgm_read_byte(&chex[num&0x0F]));
}

void lcd_unum3(uint8_t num)
{
	lcd_putch(num/100 + '0');
	lcd_putch((num%100)/10 + '0');
	lcd_putch(num%10 + '0');
}

void lcd_unum(uint16_t num)
{
    uint16_t bcd = 0;
    uint8_t bcd4 = 0;
    for(uint8_t i=16; i!=0; --i)
    {
	    if((bcd&0x000F) >= 0x5)
            bcd+=0x3;
        if((bcd&0x00F0) >= 0x50)
            bcd+=0x30;
        if((bcd&0x0F00) >= 0x500)
            bcd+=0x300;
        if((bcd&0xF000) >= 0x5000)
            bcd+=0x3000;
            
        bcd4 <<= 1;
        bcd4 += ((bcd&0x8000)?1:0);
        bcd <<= 1;
        bcd += ((num&(1<<(i-1)))?1:0);
    }
	
    lcd_putch((bcd4)?(chex[bcd4]):' ');    
    lcd_putch((!bcd4 && bcd<0x1000)	?' ':(pgm_read_byte(&chex[bcd>>12])));
    lcd_putch((!bcd4 && bcd<0x100)	?' ':(pgm_read_byte(&chex[(bcd>>8)&0x000F])));
    lcd_putch((!bcd4 && bcd<0x10)	?' ':(pgm_read_byte(&chex[(bcd>>4)&0x000F])));
    lcd_putch(pgm_read_byte(&chex[bcd&0x000F]));	
}	

void lcd_num( int num, int radix )
{
	char str[6];
	itoa( num, str, radix );
	lcd_puts( str );
}

void lcd_gotoxy( unsigned char x, unsigned char y )
{
	lcd_goto( y*64+x );
}

void Printf( char *fmt,... )
{
	va_list aptr;
	char *p, *sval;
	int ival;
	float fval;
	va_start( aptr, fmt );
	
	for( p=fmt; *p ; ++p )
	if( *p=='%' )
	switch( * ++p )
	{
		case 'b':
		ival = va_arg( aptr, int );
		lcd_num( ival, 2 );
		break;
		
		case 'x':
		ival = va_arg( aptr, int );
		lcd_num( ival, 16 );
		break;
		
		case 'd':
		ival = va_arg( aptr, int );
		lcd_num( ival, 10 );
		break;
		
		case 's':
		for( sval=va_arg(aptr, char*); *sval; ++sval )
		lcd_putch( *sval );
		break;
		
		case 'f':
		fval=va_arg( aptr, double ); 
		{
			int d,m;
			m=10000*(fval=fval-(d=fval));
			if( m<0 )
			m*=-1;
			lcd_num( d, 10 );
			lcd_putch('.');
			lcd_num( m, 10 );
		}
		break;
		
		case 'o':
		ival = va_arg( aptr, int );
		lcd_num( ival, 8 );
		break;
		
		
		default:
		lcd_putch(*p);
		
		
	}
	else if( *p=='\t' )
	lcd_putch(' ');
	else if( *p=='\n' )
	lcd_gotoxy( 0,1 );		
	else
	lcd_putch( *p );		
}
