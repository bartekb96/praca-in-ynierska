#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <util/delay.h>
#include <stdlib.h>

#define FOSC 8000000// Clock Speed
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD-1

#define TIMER_START   TIMSK |= (1<<TOIE0)
#define TIMER_STOP    TIMSK &= ~(1<<TOIE0)
#define TIMER_RESET   TCNT0 = 0;

const int bufferSize = 7;
volatile unsigned char receivingBuffer[7];
volatile unsigned int bufferIterator = 0;
volatile _Bool isReceivingCompleted = 1;

//------------PID elements---------------------------------
#define encoder_A PD2		//int0 pin
#define encoder_B PC0

volatile _Bool encoder_Last;
volatile int impPerCycleLewe;
volatile _Bool kierunekLewe;

float u;
float kp = 0.6;
float ki = 5;
float kd = 0.3;
float e = 0;
float e_sum = 0;
float e_last = 0;
float e_sum_max = 2000;
float T = 50.0; //ms ZMIEŃ TO TEZ W LOOPIE!!!
//---------------------------------------------------------

//------------PID2 elements---------------------------------
#define encoder2_A PD3		//int1 pin
#define encoder2_B PC1

volatile _Bool encoder2_Last;
volatile int impPerCyclePrawe;
volatile _Bool kierunekPrawe;

float u2;
float e2 = 0;
float e2_sum = 0;
float e2_last = 0;
//---------------------------------------------------------

double leftWheelSpeed = 0;
double rightWheelSpeed = 0;

//-------------------initialization------------------------
void USART_Init(unsigned int ubrr)
{
	UBRRH = (unsigned char)(ubrr>>8);
	UBRRL = (unsigned char)ubrr;
	UCSRB = (1<<RXEN)|(1<<TXEN);

	//UCSRC = (1<<URSEL)|(1<<USBS)|(3<<UCSZ0);		//8n2
	UCSRC = (1<<URSEL)|(3<<UCSZ0);					//8n1

	UCSRB |= (1 << RXCIE);	//enable RXC interrupt
}

void UART_Transmit (unsigned char data)
{
   while (!( UCSRA & (1<<UDRE))) ;
   UDR = data;
}

void Timer0_init(){
	//TCNT0 = 175;
	TCNT0 = 0;
	TCCR0 |= (1<CS00); //prescaler 1024
	TCCR0 |= (1<CS02);
	TIMSK |= (1<<TOIE0); //interrupt enable
}

void encoderInit(){
	kierunekLewe = 1;
	kierunekPrawe = 1;
	DDRC &= ~(1<<encoder_B); //encoder_B as input
	DDRD &= ~(1<<encoder_A); //encoder_A as input

	DDRC &= ~(1<<encoder2_B); //encoder2_B as input
	DDRD &= ~(1<<encoder2_A); //encoder2_A as input
}
//---------------------------------------------------------


void pid2_calculate(float set_val, int read_val)
{
  e2 = set_val - read_val;
  e2_sum += e2 *(T/1000.0);

  if (e2_sum > e_sum_max)
  {
    e2_sum = e_sum_max;
  }
  else if (e2_sum < -e_sum_max)
  {
    e2_sum = -e_sum_max;
  }

  u2 = kp*e2 + ki*e2_sum + kd*(e2 - e2_last);
  e2_last = e2;
}

void pid_calculate(float set_val, int read_val)
{
  e = set_val - read_val;
  e_sum += e *(T/1000.0);

  if (e_sum > e_sum_max)
  {
    e_sum = e_sum_max;
  }
  else if (e_sum < -e_sum_max)
  {
    e_sum = -e_sum_max;
  }

  u = kp*e + ki*e_sum + kd*(e - e_last);
  e_last = e;
}

float encoder_to_vel(int imp){
  return (float)(1000/T*(imp/1920.0 * 18.84));  //imp na sek -> cm na sek
}

int vel_to_pwm (float vel){
  if (vel <= 0)
    return 0;
  else
    return ceil(5.55 * vel + 50);
}


void fastPwmInitOnTimer1(){
	//PWM pin as output
	//PB1 is one channel and PB2 is another one//
	DDRB |= (1<<PB1);
	DDRB |= (1<<PB2);

	//fast PWM mode on timer1
	TCCR1B |= (1<<WGM12);
	TCCR1A |= (1<<WGM10);

	//clear OC1A and OC1B on compare match
	TCCR1A |= (1<<COM1A1);
	TCCR1A |= (1<<COM1B1);

	//set prescaler 8
	TCCR1B |= (1<<CS11);
}

void setChannelA (unsigned int fulfillment){
	OCR1A = fulfillment;
}

void setChannelB (unsigned int fulfillment){
	OCR1B = fulfillment;
}

void setStopOnBuffer(){
	bufferIterator = 0;
	receivingBuffer[0] = 'd';
	receivingBuffer[1] = '0';
	receivingBuffer[2] = '0';
	receivingBuffer[3] = '0';
	receivingBuffer[4] = '0';
	receivingBuffer[5] = '0';
	receivingBuffer[6] = '0';
}

int isDigit(char a){
	if(a=='1' || a=='2' || a=='3' || a=='4' || a=='5' || a=='6' || a=='7' || a=='8' || a=='9' || a=='0')
		return 1;
	else
		return 0;
}

void interpretSpeedSignal()
{
	if(receivingBuffer[0] == 'a')
	{
		rightWheelSpeed = ((double)receivingBuffer[1] - 48.0)*10.0 + ((double)receivingBuffer[2] - 48.0)*1.0 + ((double)receivingBuffer[3] - 48.0)*0.1;
		leftWheelSpeed = ((double)receivingBuffer[4] - 48.0)*10.0 + ((double)receivingBuffer[5] - 48.0)*1.0 + ((double)receivingBuffer[6] - 48.0)*0.1;
		PORTB |= (1<<PB0);	//motor 1 forward
		PORTD &= ~(1<<PD7);
		PORTD |= (1<<PD5);	//motor 2 forward
		PORTD &= ~(1<<PD6);
	}
	else if(receivingBuffer[0] == 'b')
	{
		rightWheelSpeed = ((double)receivingBuffer[1] - 48.0)*10.0 + ((double)receivingBuffer[2] - 48.0)*1.0 + ((double)receivingBuffer[3] - 48.0)*0.1;
		leftWheelSpeed = ((double)receivingBuffer[4] - 48.0)*10.0 + ((double)receivingBuffer[5] - 48.0)*1.0 + ((double)receivingBuffer[6] - 48.0)*0.1;
		PORTB |= (1<<PB0);	//motor 1 forward
		PORTD &= ~(1<<PD7);
		PORTD |= (1<<PD6);	//motor 2 backward
		PORTD &= ~(1<<PD5);
	}
	else if(receivingBuffer[0] == 'c')
	{
		rightWheelSpeed = ((double)receivingBuffer[1] - 48.0)*10.0 + ((double)receivingBuffer[2] - 48.0)*1.0 + ((double)receivingBuffer[3] - 48.0)*0.1;
		leftWheelSpeed = ((double)receivingBuffer[4] - 48.0)*10.0 + ((double)receivingBuffer[5] - 48.0)*1.0 + ((double)receivingBuffer[6] - 48.0)*0.1;
		PORTD |= (1<<PD7);	//motor 1 backward
		PORTB &= ~(1<<PB0);
		PORTD |= (1<<PD5);	//motor 2 forward
		PORTD &= ~(1<<PD6);
	}
	else if(receivingBuffer[0] == 'd') 		//STOP
	{
		PORTB |= (1<<PB0);	//motor 1 stop
		PORTD |= (1<<PD7);
		PORTD |= (1<<PD5);	//motor 2 stop
		PORTD |= (1<<PD6);
	}
	else
	{
		bufferIterator = 0;
	}
}

int main(void)
{
	USART_Init ( MYUBRR );
	fastPwmInitOnTimer1();
	encoderInit();
	//Timer0_init();

	DDRB |= (1<<PB1);	//PWM channel A as output
	DDRB |= (1<<PB2);	//PWM channel B as output

	DDRB |= (1<<PB0);	//dir motor1 as output
	DDRD |= (1<<PD7);	//dir motor1 as output

	DDRD |= (1<<PD5);	//dir motor2 as output
	DDRD |= (1<<PD6);	//dir motor2 as output

	PORTB |= (1<<PB0);	//motor 1 forward
	PORTD &= ~(1<<PD7);

	PORTD |= (1<<PD5);	//motor 2 forward
	PORTD &= ~(1<<PD6);

	cli();
	_delay_ms(35000);
	sei();

	while(1)
	{

		if (isReceivingCompleted)
		{
		pid_calculate(leftWheelSpeed, encoder_to_vel(2*impPerCycleLewe));
		pid2_calculate(rightWheelSpeed, encoder_to_vel(2*impPerCyclePrawe));

		setChannelB(vel_to_pwm(u));
		setChannelA(vel_to_pwm(u2));

		impPerCycleLewe = 0;
		impPerCyclePrawe = 0;
		}

	}
}

ISR(USART_RXC_vect)
{
	cli();
	while (!(UCSRA & (1<<RXC)));
	receivingBuffer[bufferIterator] = UDR;
	if (bufferIterator == 0)
	{
		if (receivingBuffer[bufferIterator] == 'a' || receivingBuffer[bufferIterator] == 'b' || receivingBuffer[bufferIterator] == 'c' || receivingBuffer[bufferIterator] == 'd')
		{
			isReceivingCompleted = 0;
			bufferIterator++;
		}
		else
		{
			setStopOnBuffer();
		}
	}
	else if (bufferIterator <= bufferSize-1)
	{
		if (isDigit(receivingBuffer[bufferIterator]))
		{
			if (bufferIterator == 6)
			{
				bufferIterator = 0;
				interpretSpeedSignal();
				isReceivingCompleted = 1;
			}
			else
			{
				bufferIterator ++;
			}
		}
		else
		{
			setStopOnBuffer();
		}
	}

sei();
}

ISR(INT0_vect)
{
	cli();

	_Bool Lstate = 0;
	if (!(PIND & (1<<encoder_A)))
		Lstate = 1;

	  if(!encoder_Last && Lstate)
	  {
	    _Bool val = 0;
	    if ( !(PINB & (1<<encoder_B)) )
	    		val = 1;

			if(!val && kierunekLewe)
			{
			  kierunekLewe = 0;
			}
			else if(val && !kierunekLewe)
			{
			  kierunekLewe = 1;
			}
	  }
	  encoder_Last = Lstate;

  if(!kierunekLewe)  impPerCycleLewe++;
  else  impPerCycleLewe--;

  sei();
}

ISR(INT1_vect)
{
	cli();

	_Bool Lstate2 = 0;
	if (!(PIND & (1<<encoder2_A)))
		Lstate2 = 1;

	  if(!encoder2_Last && Lstate2)
	  {
	    _Bool val2 = 0;
	    if ( !(PINB & (1<<encoder2_B)) )
	    		val2 = 1;

			if(!val2 && kierunekPrawe)
			{
				kierunekPrawe = 0;
			}
			else if(val2 && !kierunekPrawe)
			{
				kierunekPrawe = 1;
			}
	  }
	  encoder2_Last = Lstate2;

  if(!kierunekPrawe)  impPerCyclePrawe++;
  else  impPerCyclePrawe--;

  sei();
}

/*ISR(TIMER0_OVF_vect)
{
	TIMER_STOP;
	setStopOnBuffer();
	TIMER_RESET;
}*/
