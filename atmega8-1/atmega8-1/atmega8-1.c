#define F_CPU 8000000L
#define FIRST_ID 0x08
#define SECOND_ID 0x01
#define WIRE1_DDR DDRB
#define WIRE1_PIN PINB
#define DMAX 20 
#define STEP 32767
#define sleep _delay_ms(10)

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include "../../rs485m.h"
#include "../../1-wire.h"

<<<<<<< HEAD
unsigned char	LOG[0xFF],LOGr,LOGw,LOGt;
unsigned char	MINUTE,HOUR;
unsigned int	PLAN,BLOCK;
unsigned int	CLOCK;
unsigned long	TEN[3];
unsigned char	ID_BOX[8],ID_IN[8],ID_OUT[8],ID_DOOR[8];
		 int	TM_BOX,   TM_IN,   TM_OUT,   TM_DOOR,  TM_LAST, TM_SET,BALANS;
unsigned char	BUS1WIRE[DMAX][8];
		 int	VALUE1WIRE[DMAX];
unsigned char	rscomand,AUTO;
=======
#define F_CPU 8000000L
#define MAX 20
#define MEM 1024
#define RMAX 32
#define LMAX 64
#define sleep _delay_us(1)

unsigned char  PLAN[RMAX];
unsigned char  BLOCK[RMAX];
unsigned short light[LMAX];
unsigned short limit[LMAX];
unsigned short l_fact[LMAX];
unsigned char  delay[LMAX];
unsigned char  cl_rg[LMAX];
unsigned char  clock[LMAX];
unsigned char  l_clock;

unsigned char number;
unsigned char rcount,lcount,rerr,lerr;
volatile unsigned short COUNT_COMAND;

volatile unsigned short CRC;
volatile unsigned char status;
volatile unsigned char BUF[MAX],COUNT;
ISR(TIMER2_OVF_vect){
	unsigned char i,j;
	for(i=0;i<l_clock;i++){
		if(clock[i]<LMAX){
			j=clock[i];
			cl_rg[j]++;
			if(cl_rg[j]>=delay[j]){
				cl_rg[j]=0;
				if(light[j]>l_fact[j]) l_fact[j]++;
				if(light[j]<l_fact[j]) l_fact[j]--;
				if(light[j]==l_fact[j]) clock[i]=0xFF;
			}
		}
	}	
}
ISR(TIMER0_OVF_vect)
{
	PORTD&=~(0x08);
	TCCR0=0x00;
}
ISR(USART_RXC_vect)
{
//	if(status&0x80) return;
	BUF[COUNT]=UDR;

//while ( !(UCSRA & (1<<UDRE)) );
//UDR = BUF[COUNT];			        

	
	if(UCSRA&0x1C)
		status|=0x10;
	if(status&0x20)
		status|=0x10;	
	if(COUNT>=MAX)
		status|=0x10;
	else
		COUNT++;
	TCNT1=0x00;
	TCCR1B=0x05;
}
ISR(TIMER1_COMPA_vect)
{
	if((status&0x10)==0){
		status|=0x80;	
	}
	else
	{
		status=0x00;
		COUNT=0;
	}		
	TCCR1B=0x00;
}	
ISR(TIMER1_COMPB_vect)
{
	status|=0x20;
}
void USART_Init( unsigned char i )
{
	unsigned int baudrate;
	switch (i){
		default: 
					baudrate=103;	OCR1A=0x1C;	OCR1B=0x0C;
	}
	UBRRH = (unsigned char) (baudrate>>8);
	UBRRL = (unsigned char) baudrate;
	UCSRA = (1<<U2X);
	UCSRB = (1<<RXEN)|(1<<TXEN)|(1<<RXCIE);
	UCSRC = (0<<USBS)|(1<<UCSZ0)|(1<<UCSZ1);
>>>>>>> 8996465a0cf4e2ed129160284fa35545e2bdac70

unsigned int read_registr_param(unsigned int address)
{
	if(address==0)  return number;
	if(address==2)	return AUTO;
	if(address==3)	return (LOGr<<8)|LOG[LOGr];
	if(address==4)	return (LOGw<<8)|LOGt;
	if(address==5)  return BALANS;
	if(address<8)   return 0;
	if(address==8)	return ((PORTB&0x08)>>3)|((PORTB&0x10)>>3);
	if(address<12)	return (unsigned short)TEN[address-9];
	if(address<32)	return 0;
	if(address<40)	return ID_BOX[address-32];
	if(address<48)	return ID_DOOR[address-40];
	if(address<56)	return ID_IN[address-48];
	if(address<64)	return ID_OUT[address-56];
	if(address==64)	return PLAN;
	if(address<68)	return 0;
	if(address==68)	return BLOCK;
	if(address<72)  return 0;
	if(address==72)	return TM_BOX;
	if(address==73)	return TM_DOOR;
	if(address==74)	return TM_IN;
	if(address==75)	return TM_OUT;
	if(address==76)	return TM_LAST;
	if(address==77)	return TM_SET;
	if(address<129) return 0;
	if(address<289) return BUS1WIRE[((address-129)>>3)][(address-129)&0x07];
	if(address<309) return VALUE1WIRE[address-289];
	if(address<512)	return 0;
	if(address<1024)return eeprom_read_byte(address-512);
	return 0;
}
void write_registr_param(unsigned int address, unsigned int data)
{
	if(address==0){
		number=(unsigned char)data;
		eeprom_write_byte(1,number);
		return;
	}		
	if(address==1){rscomand|=(unsigned char)data;return;}		
	if(address==2){AUTO=(unsigned char)data;return;}		
	if(address==3){
		if((unsigned char)data==0xFF) return;
		if(((data>>8)==LOGr)&&((unsigned char)data==LOG[LOGr])){		
			LOG[LOGr]=0xFF;
			LOGr++;
		}
		return;
		}
	if(address<8)return;
	if(address==8){
		if(data&0x01) PORTB|=0x08; 
		if(data&0x02) PORTB&=~(0x08);
		if(data&0x04) PORTB|=0x10;
		if(data&0x08) PORTB&=~(0x10);
		return;
	}
	if(address<32)return;
	if(address<40) {ID_BOX[address-32]=(unsigned char)data;eeprom_write_byte(address,(unsigned char)data);return;}
	if(address<48) {ID_DOOR[address-40]=(unsigned char)data;eeprom_write_byte(address,(unsigned char)data);return;}
	if(address<56) {ID_IN[address-48]=(unsigned char)data;eeprom_write_byte(address,(unsigned char)data);return;}
	if(address<64) {ID_OUT[address-56]=(unsigned char)data;eeprom_write_byte(address,(unsigned char)data);return;}
	if(address==64){
		PLAN=data&BLOCK;
		return;
	}		
	if(address<68)	return;
	if(address==68){
		BLOCK=data;
		PLAN&=data;
		return;
	}
	if(address<77)	return;
	if(address==77) {TM_SET=data;return;}
	if(address<512)	return;
	if(address<1024){
		eeprom_write_byte(address-512,(unsigned char)data);
		return;
	}		
}
void ten_on(){
	unsigned char i=3;
	unsigned int v=0xFFFF;
	if((TEN[0]<v)&&(!(PLAN&0x01))) {v=TEN[0];i=0;}
	if((TEN[1]<v)&&(!(PLAN&0x02))) {v=TEN[1];i=1;}
	if((TEN[2]<v)&&(!(PLAN&0x04))) {v=TEN[2];i=2;}
	if(i<3) PLAN|=(1<<i);
}
void ten_off(){
	unsigned char i=3;
	unsigned int v=0;
	if((TEN[0]>v)&&(PLAN&0x01)) {v=TEN[0];i=0;}
	if((TEN[1]>v)&&(PLAN&0x02)) {v=TEN[1];i=1;}
	if((TEN[2]>v)&&(PLAN&0x04)) {v=TEN[2];i=2;}
	if(i<3) PLAN&=~(1<<i);
}
ISR(TIMER2_OVF_vect){
	switch(CLOCK){
		case 0:		if(number==0xFF){if(PORTB&0x08)PORTB&=~(0x08);else PORTB|=0x08;}
					rscomand|=0x02&AUTO;
					break;
		case 305:	rscomand|=0x20&AUTO;
					break;
		case 610:	rscomand|=0x04&AUTO;
					break;
		case 915:	rscomand|=0x40&AUTO;
					break;
		case 1220:	rscomand|=0x80&AUTO;
					break;
	}
	CLOCK++;
	if (CLOCK>1831) {CLOCK=0;MINUTE++;HOUR++;}
}
void search1wire(void){
	unsigned char *p, i,c,d,k;
	onewire_enum_init();
	for(k=0;k<DMAX;k++){
		p=onewire_enum_next();
		if(!p) break;
		d=*(p++);
		c=0;
		for(i=0;i<8;i++){
			BUS1WIRE[k][i]=d;
			c=onewire_crc_update(c,d);
			d=*(p++);
		}
		if(c)
			BUS1WIRE[k][0]=0;
	}
}
void main(void)
{
	unsigned char i,j,e,ER;
	signed long TR,TD,TS;
	PORTB=eeprom_read_byte(4)&0x18;
	DDRB=0x1F;
	DDRC=0x0F;
	DDRD=0x0C;
	DDRD&=~(0x80);
	USART_Init();
	//Initial one wire
	PIN=5;
	DDRB&=~(1<<PIN);
	PORTB&=~(1<<PIN);
	//timer2
//	TCCR2=0x07;
	TCCR2=0x04;
	ASSR=0x00;
	TIMSK|=(1<<TOIE2);
	for(i=0;i<8;i++){
		ID_BOX[i]=eeprom_read_byte(32+i);
		ID_DOOR[i]=eeprom_read_byte(40+i);
		ID_IN[i]=eeprom_read_byte(48+i);
		ID_OUT[i]=eeprom_read_byte(56+i);
	}
	number=eeprom_read_byte(1);
	AUTO=eeprom_read_word(2);
	TM_SET=eeprom_read_word(5);
	PLAN=eeprom_read_word(64);
	BLOCK=eeprom_read_word(68);
	TEN[0]=(eeprom_read_word(9)<<6);
	TEN[1]=(eeprom_read_word(11)<<6);
	TEN[2]=(eeprom_read_word(13)<<6);
	rscomand=0x01;
	COUNT=0;
	CLOCK=0;
	HOUR=0;
	MINUTE=0;
	status=0;
	LOGr=0;
	LOGw=0;
	LOGt=0;
	TM_BOX=0x8000;
	TM_LAST=0x8000;
	TS=0;
	for(i=0;i<0xFF;i++) LOG[i]=0xFF;
	COUNT_COMAND=0;
	CRC=0xFFFF;
	sei();
	while(1){
		if(rscomand&0x01){search1wire();rscomand&=~(0x01);}
		if(rscomand&0x02){onewire_reset();onewire_send(0xCC);onewire_send(0x44);rscomand&=~(0x02);}
		if(rscomand&0x04){for(i=0;i<DMAX;i++)VALUE1WIRE[i]=read_temp(BUS1WIRE[i]); rscomand&=~(0x04);}
		if(rscomand&0x40){
			if(TM_BOX!=0x8000) TM_LAST=TM_BOX;
			TM_BOX=read_temp(ID_BOX);
			TM_IN=read_temp(ID_IN);
			TM_OUT=read_temp(ID_OUT);
			TM_DOOR=read_temp(ID_DOOR);
			rscomand&=~(0x40);
		}
		if(rscomand&0x20){
			if(PLAN&0x01) {LOGt++;TEN[0]++;}
			if(PLAN&0x02) {LOGt++;TEN[1]++;}
			if(PLAN&0x04) {LOGt++;TEN[2]++;}
			rscomand&=~(0x20);
		}
		if(rscomand&0x80){
			TR=TM_BOX-TM_LAST;
			TR=TR*TR*TR*512;
			TD=TM_BOX-TM_SET;
			TD=TD*TD*TD+TR;
			if(TD>STEP) TD=STEP;
			if(TD<-STEP) TD=-STEP;
			BALANS=(signed short)TD;
			TS+=TD;			
			if((TS>>1)>STEP) {ten_off();TS=0;}
			if((TS>>1)<-STEP) {ten_on();TS=0;}
			rscomand&=~(0x80);
		}
		if(HOUR>=60){
			LOG[LOGw]=LOGt;
			LOGt=0;
			LOGw++;
			HOUR=0;
		}
		if(MINUTE>=120){
			if((TEN[0]>>6)!=eeprom_read_word(9)) eeprom_write_word(9,(TEN[0]>>6));
			if((TEN[1]>>6)!=eeprom_read_word(11))eeprom_write_word(11,(TEN[1]>>6));
			if((TEN[2]>>6)!=eeprom_read_word(13))eeprom_write_word(13,(TEN[2]>>6));
			MINUTE=0;
		}
		if(ER>=0xF0)PORTB&=~(0x10);
		PORTB&=~(0x06);sleep;
		if(PLAN&(1<<j)){PORTB|=0x02;if(!PIND&0x80) e++;}else{if(PIND&0x80) e++;}
		sleep;PORTB|=0x04;
		j++;
		if(j>15){
			if(e>0) {ER+=8;} else {if(ER>0)ER--;PORTB&=~(0x01);}
			j=0;
			e=0;
		}
	sleep;PORTB|=0x01;
	}
}

/***********************************
adress	count		eer			name		done
0		1			1			number		*
1		1			-			comand		*
2		1			2-3			AUTO		*
3		1			-			LOG			*
4		1			-			LOGt		*
5		1			-			BALANS		

8		1			4			reley		*
9		3			9-14		TEN[3]		*
32		32			32-63		ID_..		*
64		2			64-65		PLAN		
68		2			68-69		BLOCK		
72		5			-			TM_..		*
77		1			5-6			TM_SET		*
129		20*8		-			BUS1WIRE	*
289		20			-			VALUE1WIRE	*
512		512			0-511		EEPROM		*
***********************************/