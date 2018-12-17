/*
 * zkhan003 Custom Project.c
 *
 * Created: 3/6/2018 3:17:04 PM
 * Author : Zohaib Khan
 */

#include "nokia5110.h"
#include "timer.h"
#include "Usart.h"
#include <avr/pgmspace.h>
#include <avr/io.h>
#define F_CPU 8000000UL
#include <util/delay.h>
#include "nokia5110_chars.h"
#include "scheduler.h"



static struct {
    /* screen byte massive */
    uint8_t screen[504];

    /* cursor position */
    uint8_t cursor_x;
    uint8_t cursor_y;

} nokia_lcd = {
    .cursor_x = 0,
    .cursor_y = 0
};

/**
 * Sending data to LCD
 * @bytes: data
 * @is_data: transfer mode: 1 - data; 0 - command;
 */
static void write(uint8_t bytes, uint8_t is_data)
{
    register uint8_t i;
    /* Enable controller */
    PORT_LCD &= ~(1 << LCD_SCE);

    /* We are sending data */
    if (is_data)
        PORT_LCD |= (1 << LCD_DC);
    /* We are sending commands */
    else
        PORT_LCD &= ~(1 << LCD_DC);

    /* Send bytes */
    for (i = 0; i < 8; i++) {
        /* Set data pin to byte state */
        if ((bytes >> (7-i)) & 0x01)
            PORT_LCD |= (1 << LCD_DIN);
        else
            PORT_LCD &= ~(1 << LCD_DIN);

        /* Blink clock */
        PORT_LCD |= (1 << LCD_CLK);
        PORT_LCD &= ~(1 << LCD_CLK);
    }

    /* Disable controller */
    PORT_LCD |= (1 << LCD_SCE);
}

static void write_cmd(uint8_t cmd)
{
    write(cmd, 0);
}

static void write_data(uint8_t data)
{
    write(data, 1);
}

/*
 * Public functions
 */

void nokia_lcd_init(void)
{
    register unsigned i;
    /* Set pins as output */
    DDR_LCD |= (1 << LCD_SCE);
    DDR_LCD |= (1 << LCD_RST);
    DDR_LCD |= (1 << LCD_DC);
    DDR_LCD |= (1 << LCD_DIN);
    DDR_LCD |= (1 << LCD_CLK);

    /* Reset display */
    PORT_LCD |= (1 << LCD_RST);
    PORT_LCD |= (1 << LCD_SCE);
    _delay_ms(10);
    PORT_LCD &= ~(1 << LCD_RST);
    _delay_ms(70);
    PORT_LCD |= (1 << LCD_RST);

    /*
     * Initialize display
     */
    /* Enable controller */
    PORT_LCD &= ~(1 << LCD_SCE);
    /* -LCD Extended Commands mode- */
    write_cmd(0x21);
    /* LCD bias mode 1:48 */
    write_cmd(0x13);
    /* Set temperature coefficient */
    write_cmd(0x06);
    /* Default VOP (3.06 + 66 * 0.06 = 7V) */
    write_cmd(0xC2);
    /* Standard Commands mode, powered down */
    write_cmd(0x20);
    /* LCD in normal mode */
    write_cmd(0x09);

    /* Clear LCD RAM */
    write_cmd(0x80);
    write_cmd(LCD_CONTRAST);
    for (i = 0; i < 504; i++)
        write_data(0x00);

    /* Activate LCD */
    write_cmd(0x08);
    write_cmd(0x0C);
}

void nokia_lcd_clear(void)
{
    register unsigned i;
    /* Set column and row to 0 */
    write_cmd(0x80);
    write_cmd(0x40);
    /*Cursor too */
    nokia_lcd.cursor_x = 0;
    nokia_lcd.cursor_y = 0;
    /* Clear everything (504 bytes = 84cols * 48 rows / 8 bits) */
    for(i = 0;i < 504; i++)
        nokia_lcd.screen[i] = 0x00;
}

void nokia_lcd_power(uint8_t on)
{
    write_cmd(on ? 0x20 : 0x24);
}

void nokia_lcd_set_pixel(uint8_t x, uint8_t y, uint8_t value)
{
    uint8_t *byte = &nokia_lcd.screen[y/8*84+x];
    if (value)
        *byte |= (1 << (y % 8));
    else
        *byte &= ~(1 << (y %8 ));
}

void nokia_lcd_write_char(char code, uint8_t scale)
{
    register uint8_t x, y;

    for (x = 0; x < 5*scale; x++)
        for (y = 0; y < 7*scale; y++)
            if (pgm_read_byte(&CHARSET[code-32][x/scale]) & (1 << y/scale))
                nokia_lcd_set_pixel(nokia_lcd.cursor_x + x, nokia_lcd.cursor_y + y, 1);
            else
                nokia_lcd_set_pixel(nokia_lcd.cursor_x + x, nokia_lcd.cursor_y + y, 0);

    nokia_lcd.cursor_x += 5*scale + 1;
    if (nokia_lcd.cursor_x >= 84) {
        nokia_lcd.cursor_x = 0;
        nokia_lcd.cursor_y += 7*scale + 1;
    }
    if (nokia_lcd.cursor_y >= 48) {
        nokia_lcd.cursor_x = 0;
        nokia_lcd.cursor_y = 0;
    }
}

void nokia_lcd_write_string(const char *str, uint8_t scale)
{
    while(*str)
        nokia_lcd_write_char(*str++, scale);
}

void nokia_lcd_set_cursor(uint8_t x, uint8_t y)
{
    nokia_lcd.cursor_x = x;
    nokia_lcd.cursor_y = y;
}

void nokia_lcd_render(void)
{
    register unsigned i;
    /* Set column and row to 0 */
    write_cmd(0x80);
    write_cmd(0x40);

    /* Write screen to display */
    for (i = 0; i < 504; i++)
        write_data(nokia_lcd.screen[i]);
}

void ADC_init()
{
    ADCSRA = 0xE7;
    //ADCSRA |= (1 << ADEN) | (1 << ADSC) | (1 << ADATE);
    // ADEN: setting this bit enables analog-to-digital conversion.
    // ADSC: setting this bit starts the first conversion.
    // ADATE: setting this bit enables auto-triggering. Since we are
    // In Free Running Mode, a new conversion will trigger whenever
    // The previous conversion completes.
}

unsigned char SetBit(unsigned char pin, unsigned char number, unsigned char bin_value)
{
    return (bin_value ? pin | (0x01 << number) : pin & ~(0x01 << number));
}


int Activated;
unsigned char bluetoothData;
const unsigned short sensor = 0x0320;

enum blueTooth_States{blueTooth_Init, blueTooth_Wait, blueTooth_On}blueTooth_State;
int blueTooth(){
   
    switch(blueTooth_State){
       
        case blueTooth_Init:
        blueTooth_State = blueTooth_Wait;
        break;
        case blueTooth_Wait:
        bluetoothData = USART_Receive(0) & 0x03;
            if (bluetoothData == 1){
                blueTooth_State = blueTooth_Wait;
            }
            else {
                blueTooth_State = blueTooth_On;
            }
        break;
      
        case blueTooth_On:
        bluetoothData = USART_Receive(0) & 0x03;
            if (bluetoothData == 1){
                blueTooth_State = blueTooth_Wait;
            }
            else{
                blueTooth_State = blueTooth_On;
            }
        break;
      
        default:
        blueTooth_State = blueTooth_Wait;
        break;
    }
    switch (blueTooth_State){
        case  blueTooth_Init:
        break;
      
        case blueTooth_Wait:
        Activated = 0;
        break;
      
        case blueTooth_On:
        Activated = 1;
        break;
      
        default:
        break;
    }
    return blueTooth_State;
}



enum Nokia_States{Nokia_Init,Nokia_Wait,Nokia_On}Nokia_State;
int lcdNokia(){
  
    switch(Nokia_State){
        case Nokia_Init:
        Nokia_State = Nokia_Wait;
        break;
      
        case Nokia_Wait:
            if (Activated == 0){
                Nokia_State = Nokia_Wait;
            }
            else{
                Nokia_State = Nokia_On;
            }
        break;
      
        case Nokia_On:
            if (Activated == 1){
                Nokia_State = Nokia_On;
            }
            else{
                Nokia_State = Nokia_Wait;
            }
        break;
      
        default:
        Nokia_State = Nokia_Init;
        break;
    }
    switch (Nokia_State){
        nokia_lcd_init();
        case Nokia_Init:
        break;
      
        case Nokia_Wait:
        nokia_lcd_init();
            nokia_lcd_clear();
            nokia_lcd_set_cursor(0,8);
            nokia_lcd_write_string("Input" , 1);
          
            nokia_lcd_set_cursor(35,8);
            nokia_lcd_write_string(" Motion", 1);
          
            nokia_lcd_render();
        break;
      
        case Nokia_On:
        nokia_lcd_init();
                nokia_lcd_clear();
                nokia_lcd_set_cursor(0,8);
                nokia_lcd_write_string("Result" , 1);
              
                nokia_lcd_set_cursor(35,8);
                nokia_lcd_write_string("-Shown", 1);
              
                nokia_lcd_render();                  
        break;

        default:
        break;
    }
    return Nokia_State;
}  


enum motionSensor_States{motionSensor_Init, motionSensor_Wait, motionSensor_Display}motionSensor_State;
int motionSensor(){
    switch(motionSensor_State){
        case motionSensor_Init:
        motionSensor_State = motionSensor_Wait;
        break;
        case motionSensor_Wait:
        if (Activated == 0){
         motionSensor_State = motionSensor_Wait;
        }
        else{
            motionSensor_State = motionSensor_Display;
        }
        break;
      
        case motionSensor_Display:
        if (Activated == 1){
            motionSensor_State = motionSensor_Display;
        }
        else{
            motionSensor_State = motionSensor_Wait;
        }
        break;
      
        default:
        motionSensor_State = motionSensor_Init;
        break;
    }
    switch (motionSensor_State){
        case motionSensor_Init:
        break;
      
        case motionSensor_Wait:
		if((ADC < sensor))          
		{
			PORTC= SetBit(PORTC,0,1);
			PORTC= SetBit(PORTC,1,0);
		;
		}
		else{
			PORTC= SetBit(PORTC,1,0);
			PORTC = SetBit(PORTC,0,1);
		}
	
        break;
      
        case motionSensor_Display:
       
     if((ADC < sensor))         
     {
         PORTC= SetBit(PORTC,1,1);
		 PORTC = SetBit(PORTC,0,0);
     }
     else{
         PORTC= SetBit(PORTC,1,0);
		 PORTC = SetBit(PORTC,0,1);
     }
        break;
      
        default:
        break;
    }
    return motionSensor_State;
}



int main(void)
{
    DDRA = 0x00; PORTA = 0xFF;
    DDRB = 0xFF; PORTB = 0x00;
    DDRC = 0xFF; PORTC = 0x00;
    DDRD = 0x00; PORTD = 0xFF;
  
    TimerSet(100);
    TimerOn();
  
    initUSART(0);
    nokia_lcd_init();
    ADC_init();

      
  
    unsigned tasksNum = 3;
    task tasks[3];
  
  
  
    tasks[0].state = blueTooth_Init;
    tasks[0].period = 300; 
    tasks[0].elapsedTime = tasks[0].period;
    tasks[0].TickFct = &blueTooth;
  
    tasks[1].state = Nokia_Init;
    tasks[1].period = 100; 
    tasks[1].elapsedTime = tasks[0].period;
    tasks[1].TickFct = &lcdNokia;
  
    tasks[2].state = motionSensor_Init;
    tasks[2].period = 300;
    tasks[2].elapsedTime = tasks[0].period;
    tasks[2].TickFct = &motionSensor;
  
  
    unsigned long GCD = findGCD(tasks[0].period, tasks[1].period);
  
    while(1) {
    
        for(unsigned char i = 0; i < tasksNum; ++i)
        {
          
            if(tasks[i].elapsedTime >= tasks[i].period)
            {
                tasks[i].state = tasks[i].TickFct(tasks[i].state);
                tasks[i].elapsedTime = 0;
              
            }
            tasks[i].elapsedTime += GCD;
        }
        
        while(!TimerFlag);
        TimerFlag = 0;
    }
}
