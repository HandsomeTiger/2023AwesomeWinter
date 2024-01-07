#include <iom644a.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>

#define byte			unsigned char
#define word			unsigned int
#define dward			unsigned long

#define CLI()			asm("cli")
#define SEI()			asm("sei")
#define NOP()			asm("nop")

#define sbi(PORTX, BitX) 	PORTX |= (1<<BitX) //command of bit set
#define cbi(PORTX , BitX)   	PORTX &= ~(1 << BitX) //command of bit clear

#define Mean_Count	10

#define FOSC 16000000 // Clock Speed
#define BAUD_0 38400
#define MYUBRR_0 FOSC/16/BAUD_0-1

///////////////////////////////////////////////////////////////////////
//			RS-232 Function
#define	BEL		0x07
#define	BS		0x08
#define	LF		0x0a
#define	CR		0x0d
#define	ESC		0x1b
///////////////////////////////////////////////////////////////////////

#define adc_delay           10
#define QTy_RX_Data         230
#define Set_Switch_Delay    200


//PORTD
#define _RX_EN	0x01
#define _TX_EN	0x02
#define Motor		0x04
#define LED		0x05

//PORTB
#define SW_1			0x08
#define SW_2			0x04
#define SW_3			0x02
#define SW_4			0x01

//PORTA
#define LCD_RS			0x05
#define LCD_RW			0x06
#define LCD_EN			0x07

//This is I/O PORT define.
void setup(){
  DDRA = 0xF0;
  PORTA = 0xE0;
  DIDR0 = 0x0F;
  
  DDRB = 0xB0;
  PORTB = 0xAF;
  
  //LCD
  DDRC = 0xFF;
  PORTC = 0xFF;
  
  DDRD = 0xFE;
  PORTD = 0x00;
  }

void Port_Init(void)
{
	PORTA = 0x00;
	PORTB = 0x00;
	PORTC = 0x00;
	PORTD = 0x00;
}

//------------------------------------------------------------------------------------
// Creating Delay functions
void Delay(unsigned int cnt)
{
	while(cnt--);
}

void delay_us( unsigned int p)//130us --> 124us
{
        unsigned int i;
	for(i=0; i<p; i++)
          NOP();
}
void delay_10us(unsigned int p)// 8.8 ~ 9.5us : Extenal 16MHz
{
        unsigned int i;
	for(i=0; i<p*15; i++)//15//3
	{
        	NOP();
	}
}
void delay_ms(unsigned int p)//100ms --> 100ms	
{
	unsigned int j;
	for(j=0; j<p; j++)
          delay_us(1075);
}

//-----------------------------------------------------------------------------------
//LCD define

#define LCD_Delay	100
#define BUSYFLAG	0x80
#define CLS		0x01	// 0000 0001 B clear
#define RTN		0x02	// 0000 0010 B retrun home

// entry mode set
#define POSITION_L_NO_SHIFT	0x04
#define POSITION_L_SHIFT	0x05	
#define POSITION_R_NO_SHIFT	0x06	
#define POSITION_R_SHIFT	0x07	

// display
#define NO_DISPLAY_NO_CSR_NO_BL		0x08	// 0000 1000 b D = 0, C = 0, B = 0
#define DISPLAY_NO_CSR_NO_BL		0x0C	// 0000 1100 b D = 1, C = 0, B = 0
#define DISPLAY_CSR_NO_BL		0x0E	// 0000 1110 b D = 1, C = 1, B = 0
#define DISPLAY_CSR_BL			0x0F	// 0000 1111 b D = 1, C = 1, B = 1

// cursor dor display shift
#define SHIFT_CSR_L	0x10	
#define SHIFT_CSR_R	0x14	
#define SHIFT_SCE_L	0x18	
#define SHIFT_SCE_R	0x1C	

//function set
#define DATA_4_LINE_1_FONT_S	0x20	// 0010 0000 
#define DATA_4_LINE_1_FONT_L	0x24	// 0010 0100 
#define DATA_4_LINE_2_FONT_S	0x28	// 0010 1000 
#define DATA_4_LINE_2_FONT_L	0x2C	// 0010 1100 
#define DATA_8_LINE_1_FONT_S	0x30	// 0011 0000 
#define DATA_8_LINE_1_FONT_L	0x34	// 0011 0100 
#define DATA_8_LINE_2_FONT_S	0x38	// 0011 1000 
#define DATA_8_LINE_2_FONT_L	0x3C	// 0011 1100 

#define LINE_1	0x80  // 1000 0000
#define LINE_2	0xC0  // 1100 0000 

//20x4
#define LINE_3	0x94  // 1001 0100
#define LINE_4	0xD4  // 1101 0100 

// 16x4
//#define LINE_3	0x90  // 1001 0000
//#define LINE_4	0xD0  // 1101 0000 
#define CURSOR	0x80



byte DataBusy = 0;
char LCD_BUF[50];
word pwm_rate; 

////////////////////////////////////////////////////////////////////////////////
LCD functions
unsigned char busyLCD( void )
{
	unsigned char data1;
	unsigned char foo;
	
	foo = 0;

	cbi(PORTA, LCD_EN);// = 0;
		Delay(LCD_Delay);
		
	cbi(PORTA, LCD_RS);// = 0;
	sbi(PORTA, LCD_RW);// = 1;
	sbi(PORTA, LCD_EN);// = 1;
		Delay(LCD_Delay);
		Delay(LCD_Delay);

	DDRC = 0x00; //?? 
	Delay(LCD_Delay);

	foo = PINC ;
	Delay(LCD_Delay);

	foo = foo;
	data1 = PINC ;
	Delay(LCD_Delay);
		
	DDRC = 0xff;
	PORTC = 0xff;
	Delay(LCD_Delay);
		
	cbi(PORTA, LCD_EN);// = 0;


	if( ( data1 &= BUSYFLAG) == BUSYFLAG )
		return 1;
	else 
		return 0;
}

void commandLCD( unsigned char command )
{
	while( DataBusy );			
	DataBusy = 1;	

	while( busyLCD() );
		Delay(LCD_Delay);

	cbi(PORTA, LCD_EN);// = 0;
		Delay(LCD_Delay);
		
	cbi(PORTA, LCD_RS);// = 0;
	cbi(PORTA, LCD_RW);// = 0;
	sbi(PORTA, LCD_EN);// = 1;
		Delay(LCD_Delay);

	PORTC = command;
		Delay(LCD_Delay);
	
	cbi(PORTA, LCD_EN);// = 0;
		Delay(LCD_Delay);
		Delay(LCD_Delay);

	DataBusy = 0;	
	return;
}

void writeLCD(unsigned char data1)
{
	while( DataBusy );		
	DataBusy = 1;		

	while( busyLCD() );
		
	cbi(PORTA, LCD_EN);// = 0;
		Delay(LCD_Delay);

	sbi(PORTA, LCD_RS);// = 1;
	cbi(PORTA, LCD_RW);// = 0;
	sbi(PORTA, LCD_EN);// = 1;
		Delay(LCD_Delay);

	PORTC = data1;
		Delay(LCD_Delay);

	cbi(PORTA, LCD_EN);// = 0;
		Delay(LCD_Delay);

	DataBusy = 0;
	return;
}

void stringLCD( const char *s_string, unsigned char line, unsigned char position )
{
	unsigned char i=0;
	unsigned char strnum=0;
	strnum = strlen( s_string );
	commandLCD( line + position );
	
	for( i = 0; i < strnum ; i++ )
	{
		writeLCD( s_string[i] );
		//put_hex_ascii( s_string[i] );
	}
	return;
}
			

void lcdInit(void)
{
	unsigned int  i;

	delay_ms(300);
	
	CLI();
	
	
	for( i =0; i < 3 ; i++ )
	{
		PORTC = 0xFF;

		cbi(PORTA, LCD_EN);// = 0;
			Delay(LCD_Delay);

		cbi(PORTA, LCD_RS);// = 0;
		cbi(PORTA, LCD_RW);// = 0;
		sbi(PORTA, LCD_EN);// = 1;
			delay_us(LCD_Delay);

		PORTC = DATA_8_LINE_2_FONT_S;
			Delay(LCD_Delay);

		cbi(PORTA, LCD_EN);// = 0;

	}
	commandLCD( DISPLAY_NO_CSR_NO_BL );	
	commandLCD( POSITION_R_NO_SHIFT );	
	commandLCD( SHIFT_CSR_R );	
	commandLCD( CLS );

	SEI();//EA = 1;	
	return;

}
//--------------------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////
//			ADC
///////////////////////////////////////////////////////////////////////
void ADC_Init(void)
{
	ADMUX = 0x40;		
	ADCSRA = 0xA6; // 1010 0110
	ADCSRB = 0x00;
	
	DIDR0 = 0x0F;
}

unsigned int Read_ADC( byte ch)
{
	unsigned int result = 0;


	if ( ch >= 8 )
	{
		ADMUX = 0x40 | (ch-8);
	}
	else 
	{
		ADMUX = 0x40 | ch;
	}

	
	
	sbi( ADCSRA, ADSC);		// Starting ADC Converter 
	delay_us(500);

	while ( ( ADCSRA & 0x10 ) == 0 )
		ADCSRA |= 0x10;

	cbi( ADCSRA, ADSC );		// End ADC Converter 
  
	result = ADC;
	
	return result;
}

unsigned int Mean_Read_ADC(char ch)
{

	unsigned long foo = 0;
	unsigned int result = 0;
	char i = 0;

	delay_us(1);

	for(i=0; i < Mean_Count; i++)
	{
		foo = foo + Read_ADC(ch);
	}

	result = foo / Mean_Count;

	return result;

}
//--------------------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////
//			UART
///////////////////////////////////////////////////////////////////////

void USART_0_Init( unsigned int ubrr)
{
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	
	UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00)|(1<<UPM01)|(1<<UPM00);//8data, 1stop bit, Odd
}


void put_0_char(byte data)
{
	// Wait for empty transmit buffer
	while ( !( UCSR0A & (1<<UDRE0)) );

	// Put data into buffer, sends the data
	UDR0 = data;
}


byte get_0_char(void)
{
	// Wait for data to be received
	while ( !(UCSR0A & (1<<RXC0)) );

	// Get and return received data from buffer
	return UDR0;
  
}

void put_0_string(const char *str)
{
	int i;
  
	for (i=0; *(str+i)!=0;i++)
	{
		if(*(str+i) == '\n')
			put_0_char(LF);
		put_0_char(*(str+i));
		delay_ms(5);
	}

}
void put_0_string_unsigned(unsigned char *str,byte ccc)
{
	int i;

	for (i=0; i<ccc ;i++)
	{
		if(*(str+i) == '\n')
			put_0_char(LF);
		put_0_char(*(str+i));
		delay_ms(5);    
	}
}

void put_0_hex_ascii(unsigned char ch)
{
        byte hnibble, lnibble;
        
        lnibble = ch & 0x0f;
        hnibble = ch >> 4;
        
        if ( hnibble < 0x0a )
                hnibble = hnibble + 0x30;
        else
                hnibble = hnibble + 0x37;
        
        put_0_char (hnibble);
        
        
        if ( lnibble < 0x0a )
                lnibble = lnibble + 0x30;
        else
                lnibble = lnibble + 0x37;
        
        put_0_char (lnibble);
	delay_ms(10);
      
}

char RX_Data_0[QTy_RX_Data];

void RX_Data_0_Init(void)
{
	extern volatile char RX_Data_0[QTy_RX_Data];
	
	unsigned char i;
	
	for ( i=0; i<QTy_RX_Data ;i++)
	{
		RX_Data_0[i] = 0;
	}
}

//--------------------------------------------------------------------------------------------
Creating functions for inputs and outputs
unsigned char Key_1 (void)
{
	byte iii = 0;
	byte foo = 0;
	
	iii = PINB & SW_1;
	
	if ( iii == SW_1 )
		foo = 1;
	
	return foo;
}

unsigned char Key_2 (void)
{
	byte iii = 0;
	byte foo = 0;
	
	iii = PINB & SW_2;
	
	if ( iii == SW_2 )
		foo = 1;
	
	return foo;
}

unsigned char Key_3 (void)
{
	byte iii = 0;
	byte foo = 0;
	
	iii = PINB & SW_3;
	
	if ( iii == SW_3 )
		foo = 1;
	
	return foo;
}

unsigned char Key_4 (void)
{
	byte iii = 0;
	byte foo = 0;
	
	iii = PINB & SW_4;
	
	if ( iii == SW_4 )
		foo = 1;
	
	return foo;
}

void LED_2(void){
	
	int i;
	
	for(i = 0; i<2; i++)
	{
	cbi(PORTD, LED);
	delay_ms(100);
	sbi(PORTD, LED);
	delay_ms(100);
	}
}
void Motor_2(void){
	int i;
	
	for (i = 0; i < 2; i++)
	{
	sbi(PORTD, Motor);
	delay_ms(500);
	cbi(PORTD, Motor);
	delay_ms(500);
	}
}


void InitTimer() {
   TCCR1A = 0xF3;
   TCCR1B = 0x03;
   TCNT1 = 0x00;
   OCR1A  = 0xFFFF;
   OCR1B  = 0xFFFF;
}


////////////////////////////////////////////////////////////////////////////////
// 	Routine:	Interrupt UART 0
////////////////////////////////////////////////////////////////////////////////

byte rs485_add = 0;
byte run_end = 0;
byte led_current [10];
byte led_status [10];

float smps_voltage = 0.0;


#pragma vector = USART0_RX_vect
__interrupt void USART0_RX_interrupt(void)
{
	extern volatile char RX_Data_0[QTy_RX_Data];
	
	extern volatile word pwm_rate; 

	//extern volatile byte led_status [10];
	
	byte receive_connection = 0 ;

	word i = 0;
	word k = 0;
	
	while(1)
	{
		if ( !(UCSR0A & 0x80)  == 0 )
		{
			RX_Data_0[i] = UDR0;			
			UCSR0A |= 0x80; 
			
			if ( RX_Data_0[0] == 0xA1   &&    RX_Data_0[2] == 0x0D   &&   RX_Data_0[3] == 0x0A  && i == 3 ) 
			{
				receive_connection = 1;								
				break;
			}

			i++;
			k = 0;
		}
			
		k++;
		if ( k >= 10000   ||   i > QTy_RX_Data)
		{
			RX_Data_0_Init();
			
			//Buzzer(1);
			break;
		}

	}
	
	
	if ( receive_connection == 1 )
	{
		put_0_char(RX_Data_0[1]);
		pwm_rate = RX_Data_0[1] * 4; 
	}
}



//------------------------------------------------------------------------------------------------------------------------------
int main(void)
//cbi(PORTD, LED); // LED HIGH
//sbi(PORTD, LED); // LED LOW

//sbi(PORTD, Motor); //Motor HIGH
//cbi(PORTD, Motor); //Motor LOW

{
  word ADC_data;  
  setup();
  Port_Init();
  lcdInit();
  
  ADC_Init();
  InitTimer();
  
  USART_0_Init(MYUBRR_0);
  RX_Data_0_Init();

sprintf(LCD_BUF, "Handsometiger");
stringLCD(LCD_BUF, LINE_1,0);

	 sbi(PORTD, LED); // LED LOW
pwm_rate = 0;
  
  while(1){

  	ADC_data = Mean_Read_ADC(0); 
	sprintf(LCD_BUF, "%4d %4d", ADC_data,pwm_rate);
	stringLCD(LCD_BUF, LINE_4,0);
	

	OCR1A = ADC_data;
	OCR1B = 1024 - pwm_rate;

	
	if  (Key_1() == 0 && Key_2() & Key_3() & Key_4()!= 0) {
	cbi(PORTD, LED);
	sprintf(LCD_BUF, "Key_1 is ON");
  	stringLCD(LCD_BUF, LINE_2,0);
	//OCR1A = 1023;
	
	}
	
	if  (Key_2() == 0 && Key_1() & Key_3() & Key_4() != 0) {
	sbi(PORTD, Motor);
	sprintf(LCD_BUF, "Key_2 is ON");
  	stringLCD(LCD_BUF, LINE_2,0);
	//OCR1A = 0;
	}
	
	if  (Key_3() == 0 && Key_2() & Key_1() & Key_4() != 0) {
	LED_2();
	sprintf(LCD_BUF, "Key_3 is ON");
  	stringLCD(LCD_BUF, LINE_2,0);
	}
	
	if  (Key_4() ==0 && Key_2() & Key_3() & Key_1() != 0) {
	Motor_2();
	sprintf(LCD_BUF, "Key_4 is ON");
  	stringLCD(LCD_BUF, LINE_2,0);
	}
	
	else
		sbi(PORTD, LED); //LED LOW
		cbi(PORTD, Motor); //Motor HIGH

  }

}
