/*
 * mpu_test2.c
 *
 * Created: 27-07-2019 21:15:26
 
 */ 

#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <avr/interrupt.h>

#include "mpu6050/mpu6050.h"

#define F_CPU 16000000UL
#include <util/delay.h>
#define baudrate 9600
#define FOSC 8000000UL
#define UBRR 51
float pitch,integral,p_error,t_error,Kp=1,Kd=0,Ki=0,differential,error,flag;



void uart_init (void)
{
	UBRR0H = (UBRR >> 8);
	UBRR0L = UBRR;
	UCSR0C |=(1<<UCSZ00)|(1<<UCSZ01);
	UCSR0B |=(1<<RXEN0)|(1<<TXEN0);
	
}

void uart_transmit (char data)
{
	while (!( UCSR0A & (1<<UDRE0)));
	UDR0 = data;
	
}

char uart_receive (void)
{
	while (!( UCSR0A & (1<<RXC0)));
	return UDR0;
}

void usart_transmit_string(char *str)
{

	while(*str > 0)
	{
		
		uart_transmit(*str);
		str++;
	}
}


//PID CODE

double pid_calc(double pitch)
{
	error=pitch;
	integral=integral+error;
	differential=p_error-error;
	t_error=Kp*error+Kd*differential+Ki*integral;
	p_error=error;
	return t_error;
}
  
  
  

void forward()
{
   PORTB|=(1<<PINB3);
}

void backward()
{
   PORTB&=(~(1<<PINB3));
}


float op_scaling(float output)
{
	output=(output/90)*255;
	
	
	if(output>255)
	{
		output=255;
	}
	if (output<-255)
	{
		output=-255;
	}
	return output;
}

//float stop()
//{
	//pitch=0;
//}



int main(void)
{
	mpu6050_init();
	uart_init();
	
	//timer 
	TIMSK |=(1<<TOIE0);
	TCNT0=0x00;
	TCCR0 |=(1<<CS02)|(1<<CS00) | (1<<CS01); 
	
	
	//PWM CODE
	TCCR2|=(1<<WGM21)|(1<<WGM20)|(1<<COM21)|(1<<CS21);
	//int duty;
	DDRB|=(1<<PINB7);
	
	
	//int intdata=179;
	
/*	int16_t ax = 0;
	int16_t ay = 0;
	int16_t az = 0;
	int16_t gx = 0;
	int16_t gy = 0;
	int16_t gz = 0;
	double qw,qx,qy,qz;
*/	double roll,pitch,yaw;
    char data[10] ;
	char data1[10] ;
//	char data2[10] ;
	float output,pwm;
	sei();
    while (1) 
	{
		mpu6050_getRollPitchYaw(&roll,&pitch,&yaw);
		pitch = pitch * 180/3.1415;
		output=pid_calc(pitch);
		
		if (pitch>0)
		{
			//usart_transmit_string("back");
			forward();
		}
		 if(pitch<0)
		{
			//usart_transmit_string("for");
			backward();
		}
		
		pwm=op_scaling(output);
		
		
		if (pwm<0)
		{
			OCR2=(-1)*(int)pwm;
		}
		else
		{
			OCR2=(int)pwm;
		}
			//mpu6050_getRawData(&ax, &ay, &az, &gx, &gy, &gz);
		
		//itoa(ax,data,10);				
		//usart_transmit_string(data);
		
		//mpu6050_updateQuaternion();
		//mpu6050_updateQuaternion();
		//mpu6050_getQuaternion(&qw,&qx,&qy,&qz);
		
		//itoa(a,data,10);
		
		//pitch print
		
		
		dtostrf(pitch,10,2,data1);
		
		usart_transmit_string(data1);
		usart_transmit_string("     ");
		
		//pwm print
		
		//dtostrf(pwm,10,2,data);
		
		//usart_transmit_string(data);
		
		//usart_transmit_string("     ");
		//_delay_us(50);
				
		
    }
}


ISR(TIMER0_OVF_vect) {
	mpu6050_updateQuaternion();
	//usart_transmit_string("as");
}
