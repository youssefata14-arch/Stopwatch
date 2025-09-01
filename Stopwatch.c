/*
=============================================================================
 * Project Name: Stopwatch
 * Created on  : Sep 1, 2025
 * Author Name : Youssef ATA
=============================================================================
 */
/*----------------- Libraries and Definitions ------------*/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
/*--------------------- Global Variables ------------------*/
unsigned char Flag_timer1= 0;
unsigned char Flag_count_mode = 0;
unsigned char flag_decrementseconds=0;
unsigned char flag_decrementminutes=0;
unsigned char flag_decrementhours=0;
unsigned char flag_incrementhours=0;
unsigned char flag_incrementminutes=0;
unsigned char flag_incrementseconds=0;
unsigned char paused=0;
unsigned char modeCountdown=0;
unsigned char count_up  = 1;
unsigned char Active_BUZZER =0;
unsigned char Seconds = 0 ;
unsigned char Minutes = 0 ;
unsigned char Hours   = 0 ;
/*------------ increment time -------------*/
void increment_time(void){
	Seconds++;
	if(Seconds >=60){
		Seconds=0;
		Minutes++;
		if(Minutes >=60){
			Minutes=0;
			Hours++;
			if(Hours>=100)  // two-digit hours (00TO99)
				Hours=0;
		}
	}
}
/*------------ Decrement time -------------*/
void decrement_time(void){
	if(Hours == 0 && Minutes == 0 && Seconds == 0){
		Active_BUZZER = 1;
		paused = 1;
		return;
	}
	if(Seconds > 0)
		Seconds--;
	else {
		if(Minutes > 0){
			Minutes--;
			Seconds = 59;
		} else {
			if(Hours > 0){
				Hours--;
				Minutes = 59;
				Seconds = 59;
			}
		}
	}
}
/*--------- Timer1 initialization ----------*/
void timer1(void){
	TCNT1  = 0 ; // initial value to count
	OCR1A =15624; //1hz=f_cpu/1024(1+top)  prescaler=1024
	TCCR1A = (1<<FOC1A) | (1<<FOC1B) ; /* non PWM mode */
	TCCR1B = (1<<WGM12) | (1<<CS12) | (1<<CS10); // CTC, prescaler 1024
	TIMSK|=(1<<OCIE1A);//Module interrupt Enable
	SREG  |= 1<<7 ; //  ACTIVE I_ BIT Interrupt
}
/*--------- Timer1 ISR ----------*/
ISR(TIMER1_COMPA_vect){
	Flag_timer1=1;
}
/*--------- INT0 for reset ----------*/
void INT0_INT(void){
	MCUCR|=(1<<ISC01);MCUCR &= ~(1<<ISC00);
	GICR  |= (1<<INT0);
	SREG  |= (1<<7);
}
/*---------ISR INT0-----------*/
ISR(INT0_vect){
	Hours   = 0 ;Minutes = 0 ;Seconds = 0 ;
	PORTD&=~(1<<0);
	Active_BUZZER=0;
	paused=0;
	count_up=1;
}
/*---------INT1 FOR stop Timer----------*/
void INT1_INT(void){
	MCUCR|=(1<<ISC11);MCUCR&=~(1<<ISC10);
	GICR  |= (1<<INT1);
	SREG  |= (1<<7);
}
/*---------ISR INT1-----------*/
ISR(INT1_vect){
	TCCR1B &=~((1<<CS12)|(1<<CS11)|(1<<CS10));
}
/*--------- INT2 Resume Timer ----------*/
void INT2_INT(void){
	MCUCSR&=~(1<<ISC2);
	GICR  |= (1<<INT2);
	SREG  |= (1<<7);
}
/*---------ISR INT2-----------*/
ISR(INT2_vect){
	TCCR1B = (1<<WGM12)|(1<<CS12)|(1<<CS10);
}

int main (void){
	DDRA = 0x3F;                  // FROM PA0 TO PA5 as digit enables outputs
	PORTA = 0x00;                 // Initially all 7segments are disable
	DDRC  =   0X0F  ;           // 4 outputs PINS decoder connected to first 4 pins in PORTC
	PORTC &= ~(0X0F) ;           // Initial value is zero
	DDRD =   0X31 ;             //PD0 ,PD4,PD5 OUTPUT AND PD2 , PD3 are input AND PD1,PD6,PD7 FLOATING
	PORTD |= (1<<PD2);         //activate the internal pull up resistorPD2
	DDRB  = 0X00 ;             //ALL pins in portB input pins
	PORTB = 0XFF ;            //activate the internal pull up resistor
	timer1();
	INT0_INT();
	INT1_INT();
	INT2_INT();
	while (1){
		/*---------------- Timer1 flag ----------------*/
		if(Flag_timer1){
			Flag_timer1 = 0;
			if(!paused){
				if(modeCountdown)
					decrement_time();
				else
					increment_time();
			}
		}

		/*-----------count mode-------------*/
		if(!(PINB & (1<<PB7))){
			_delay_ms(30);
			if(!(PINB & (1<<PB7))){
				if(Flag_count_mode==0){
					modeCountdown = !modeCountdown;
					Flag_count_mode = 1;
				}
			}
		} else Flag_count_mode = 0;
		/*---------------- LEDS ----------------*/
		if(modeCountdown){
			PORTD |= (1<<PD5); // counting down LED on
			PORTD &= ~(1<<PD4);
		} else {
			PORTD |= (1<<PD4); // counting up LED on
			PORTD &= ~(1<<PD5);
		}
		/*---------------- BUZZER----------------*/
		if(Active_BUZZER==1)
			PORTD |= (1<<PD0);
		else PORTD &= ~(1<<PD0);
		/*----------------incrementseconds-----------------*/
		if(!(PINB & (1<<PB6))){
			_delay_ms(30);
			if(!(PINB & (1<<PB6))){
				if(flag_incrementseconds==0){
					if(Seconds < 59)
						Seconds++;
					flag_incrementseconds = 1;
				}
			}
		} else flag_incrementseconds = 0;
		/*--------------incrementminutes----------------*/
		if(!(PINB & (1<<PB4))){
			_delay_ms(30);
			if(!(PINB & (1<<PB4))){
				if(flag_incrementminutes==0){
					if(Minutes < 59) Minutes++;
					flag_incrementminutes = 1;
				}
			}
		} else flag_incrementminutes = 0;
		/*-----------------incrementhours-------------------*/
		if(!(PINB & (1<<PB1))){
			_delay_ms(30);
			if(!(PINB & (1<<PB1))){
				if(flag_incrementhours==0){
					if(Hours < 99) Hours++;
					flag_incrementhours = 1;
				}
			}
		} else flag_incrementhours = 0;
		/*----------------decrementseconds---------------*/
		if(!(PINB & (1<<PB5))){
			_delay_ms(30);
			if(!(PINB & (1<<PB5))){
				if(flag_decrementseconds==0){
					if(Seconds > 0) Seconds--;
					flag_decrementseconds = 1;
				}
			}
		} else flag_decrementseconds = 0;

		/*---------------decrementminutes--------------------*/
		if(!(PINB & (1<<PB3))){
			_delay_ms(30);
			if(!(PINB & (1<<PB3))){
				if(flag_decrementminutes==0){
					if(Minutes > 0) Minutes--;
					flag_decrementminutes = 1;
				}
			}
		} else flag_decrementminutes = 0;

		/*-------------------decrementhours-----------------*/
		if(!(PINB & (1<<PB0))){
			_delay_ms(30);
			if(!(PINB & (1<<PB0))){
				if(flag_decrementhours==0){
					if(Hours > 0) Hours--;
					flag_decrementhours = 1;
				}
			}
		} else flag_decrementhours = 0;


		/************* multiplex display*****************/
		PORTA=0;
		PORTA|=(1<<5);//ACTIVE THE FIRSE SEVEN SEGMENT
		PORTC = (PORTC & 0xF0) | ((Seconds % 10) & 0x0F);//Display units of Seconds
		_delay_ms(2);
		PORTA=0;
		PORTA|=(1<<4);//ACTIVE THE SECOND SEVEN SEGMENT
		PORTC = (PORTC & 0xF0) | ((Seconds / 10) & 0x0F);//Display tens of Seconds
		_delay_ms(2);
		PORTA=0;
		PORTA|=(1<<3);//ACTIVE THE THIRD SEVEN SEGMENT
		PORTC = (PORTC & 0xF0) | ((Minutes % 10) & 0x0F);//Display units of Minutes
		_delay_ms(2);
		PORTA=0;
		PORTA|=(1<<2);//ACTIVE the fourth SEVEN SEGMENT
		PORTC = (PORTC & 0xF0) | ((Minutes / 10) & 0x0F);//Display tens of Minutes
		_delay_ms(2);
		PORTA=0;
		PORTA|=(1<<1);//ACTIVE the Fifth SEVEN SEGMENT
		PORTC = (PORTC & 0xF0) | ((Hours % 10) & 0x0F);//Display units of Hours
		_delay_ms(2);
		PORTA=0;
		PORTA|=(1<<0);//ACTIVE the Sixth SEVEN SEGMENT
		PORTC = (PORTC & 0xF0) | ((Hours / 10) & 0x0F);//Display tens of HOURS
		_delay_ms(2);

	}

}

