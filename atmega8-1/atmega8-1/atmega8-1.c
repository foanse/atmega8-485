#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

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

} 
void sendchar(unsigned char data){
	if(number!=BUF[0]) return;
    unsigned char i;
	CRC ^= data;
	for (i = 8; i != 0; i--) {
		if ((CRC & 0x0001) != 0) {
			CRC >>= 1;
			CRC ^= 0xA001;
		}
		else
			CRC >>= 1;
	}
	if(!(PORTD&0x04)){
		PORTD|=0x04;
		_delay_ms(15);
	}
	while ( !(UCSRA & (1<<UDRE)) );
	UDR = data;			        
}
void sendcrc(){
	if(number!=BUF[0]) return;
	while ( !(UCSRA & (1<<UDRE)) );
	UDR = (unsigned char)(CRC>>8);			        
	while ( !(UCSRA & (1<<UDRE)) );
	UDR = (unsigned char)CRC;			        
	while ( !(UCSRA & (1<<UDRE)) );
	_delay_ms(15);
	PORTD&=~(0x04);
	CRC=0xFFFF;
}
unsigned char CRCin(unsigned char count)
{
	unsigned short crc = 0xFFFF;
    unsigned char pos,i;
    for (pos = 0; pos < count; pos++) {
		crc ^= (unsigned char)BUF[pos];
		for (i = 8; i != 0; i--) {
			if ((crc & 0x0001) != 0) {
				crc >>= 1;
				crc ^= 0xA001;
			}
		else
			crc >>= 1;
		}
    }
	if((((unsigned char)(crc>>8))==BUF[count])&&(((unsigned char)crc)==BUF[count+1])) 
		return 1;
	else
		return 0;
}
unsigned short read_registr_param(unsigned short address)
{
	if(address==0)
		return number;
	if(address==1)
		return (UBRRH<<8)|UBRRL;
	if(address==2)
		return ((PORTB&0x08)>>3)|((PORTB&0x10)>>3);
	if(address==3)
		return (rerr&0xC0)|(rcount&0x3F);
	if(address==4)
		return (lerr&0xC0)|(lcount&0x3F);
	if(address==5)
		return TCCR2;
	if(address<127)
		return 0;
	if(address<(127+RMAX))
		return PLAN[address-127];
	if(address<191)
		return 0;
	if(address<(191+RMAX))
		return BLOCK[address-191];
	if(address<255)
		return 0;
	if(address<(255+LMAX))
		return light[address-255];
	if(address<319)
		return 0;
	if(address<(319+LMAX))
		return limit[address-319];
	if(address<383)
		return 0;
	if(address<(383+LMAX))
		return delay[address-383];
	if(address<511)
		return 0;
	if(address<1024)
		return eeprom_read_byte(address-511);
	return 0;
}
void write_registr_param(unsigned short address, unsigned short data)
{
	if(address==0){
		number=(unsigned char)data;
		eeprom_write_byte(1,number);
		return;
	}		
	if(address==1){
		USART_Init((unsigned char)data);
		return;
	}
	if(address==2){
		if(data&0x01) PORTB|=0x08; 
		if(data&0x02) PORTB&=~(0x08);
		if(data&0x04) PORTB|=0x10;
		if(data&0x08) PORTB&=~(0x10);
		return;
	}
	if(address==3){
		rcount=(unsigned char)data;
		rerr=0;
		DDRB|=0x07;	
		return;
	}		
	if(address==4){
		lcount=(unsigned char)data;
		lerr=0;
		DDRC|=0x0E;	
		return;
	}		
	if(address==5){
		TCCR2=(unsigned char)data&0x07;
		lerr=0;
		DDRC|=0x0E;	
		return;
	}		
	if(address<127)
		return;
	if(address<(127+RMAX)){
		PLAN[address-127]=(unsigned char)data&BLOCK[address-127];
		return;
	}		
	if(address<191)
		return;
	if(address<(191+RMAX)){
		BLOCK[address-191]=(unsigned char)data;
		PLAN[address-191]&=data;
		return;
	}
	if(address<255)
		return;
	if(address<(255+LMAX)){
		if(data>limit[address-255])
			light[address-255]=limit[address-255];
		else
			limit[address-255]=data;
		return;
	}
	if(address<319)
		return;
	if(address<(319+LMAX)){
		limit[address-319]=data;
		if(light[address-319]>limit[address-319])
			light[address-319]=data;
		return;
	}
	if(address<383)
		return;
	if(address<(383+LMAX)){
		delay[address-383]=(unsigned char)data;
		return;
	}

	if(address<511)
		return;
	if(address<1024){
		eeprom_write_byte(address-511,(unsigned char)data);
		return;
	}		
}
void swit(){
	sendchar(number);
	unsigned short B,E,D,C;
	switch(BUF[1]){
	case 0x01:
			if(COUNT!=8) goto er;
			B=(BUF[2]<<8)|BUF[3];
			if(B>(MEM*8-1)) goto er;
			sendchar(0x01);
			sendchar(0x01);
			D=B>>3;
			B&=0x0007;
			C=read_registr_param(D);
			if((C&(1<<B))>0)
				sendchar(0x01);
			else
				sendchar(0x00);
			goto re;
//				case 0x02:read_bit_input();break;
	case 0x03:
	case 0x04:
			if(COUNT!=8) goto er;
			B=(BUF[2]<<8)|BUF[3];
			if((B+BUF[5])>MEM) goto er;			
			sendchar(BUF[1]);
			sendchar(BUF[5]);
			for(D=B;D<(B+BUF[5]);D++){	
				C=read_registr_param(D);
				sendchar((unsigned char)(C>>8));
				sendchar((unsigned char)C);
			}
			goto re;
	case 0x05:
			if(COUNT!=8) goto er;
			B=(BUF[2]<<8)|BUF[3];
			if(B>(MEM*8-1))goto er;
			D=(B>>3);
			B&=0x0007;
			C=read_registr_param(D);
			if(BUF[4]==0xFF) 
				C|=(1<<B);
			if(BUF[5]==0xFF)
				C&=~(1<<B);
			write_registr_param(D,C);
			sendchar(0x05);
			sendchar(BUF[2]);
			sendchar(BUF[3]);
			sendchar(0x00);
			sendchar(0x01);
			goto re;
	case 0x06:		
			if(COUNT!=8) goto er;
			B=(BUF[2]<<8)|BUF[3];
			C=(BUF[4]<<8)|BUF[5];
			if(B>MEM) goto er;			
			sendchar(0x06);
			sendchar(BUF[2]);sendchar(BUF[3]);
			sendchar(BUF[4]);sendchar(BUF[5]);
			write_registr_param(B,C);
			goto re;
	case 0x0B:
			if(COUNT!=4) goto er;
			sendchar(0x0B);
			sendchar((unsigned char)(COUNT_COMAND>>8));
			sendchar((unsigned char)COUNT_COMAND);
			goto re;
//				case 0x0F:write_bits_output();break;
//				case 0x10:write_registrs_param();break;
	case 0x11:
			if(COUNT!=4) goto er;
			sendchar(0x11);sendchar(3);sendchar(0x08);sendchar(0x01);sendchar(0xFF);
			goto re;
	default:
			COUNT_COMAND--;
			goto er;
	}
er:
	sendchar(BUF[1]|0x80);
	sendchar(0x01);
re:	
	sendcrc();
}

void main(void)
{
	unsigned char ri,li,rj,lj,re,le;
	USART_Init(eeprom_read_byte(0));
	number=eeprom_read_byte(1);
	rcount=eeprom_read_byte(2);
	if(rcount>RMAX)
		rcount=RMAX;
	lcount=eeprom_read_byte(3);
	if(lcount>LMAX)
		lcount=LMAX;
	DDRD=0x0C;
	DDRB=0x1F;
	DDRC=0x0E;
	rerr=0;
	lerr=0;
	PORTB=eeprom_read_byte(4)&0x18;
	TCCR2=eeprom_read_byte(5)&0x07;
	for(COUNT=0;COUNT<RMAX;COUNT++){
		BLOCK[COUNT]=eeprom_read_byte(50+COUNT);
		PLAN[COUNT]=eeprom_read_byte(10+COUNT)&BLOCK[COUNT];
	}
	for(COUNT=0;COUNT<LMAX;COUNT++){
		clock[COUNT]=COUNT;
		cl_rg[COUNT]=0;
		l_fact[COUNT]=0;
		delay[COUNT]=eeprom_read_word(400+COUNT);
		limit[COUNT]=eeprom_read_word(250+COUNT*2);
		light[COUNT]=eeprom_read_word(100+COUNT*2);
		if(limit[COUNT]<light[COUNT])
			light[COUNT]=limit[COUNT];
	}
	l_clock=LMAX;
	COUNT=0;
	status=0;
	COUNT_COMAND=0;
	CRC=0xFFFF;
	TCCR0=0x00;
	ASSR=0x00;
	TCCR1A=0x00;
	TCCR1B=0x08;
	TIMSK=(1<<OCIE1A)|(1<<OCIE1B)|(1<<TOV0)|(1<<TOIE2);
	sei();
	while(1){
		if(status&0x80){
			if((number==BUF[0]|BUF[0]==0x00)){
				if(CRCin(COUNT-2)){
					PORTD|=0x08;
					TCCR0=0x05;
					COUNT_COMAND++;
					swit();
					}
			}	
			status=0;
			COUNT=0;
		}
		if(rerr>0xC0) DDRB&=~(0x07);
		if(lerr>0xC0) DDRC&=~(0x0E);
		PORTB&=~(0x01);
		PORTC&=~(0x02);
		sleep;
		if(PLAN[ri]&(1<<rj)){
			if(!(PIND&0x80)) re++;
			PORTB|=0x02;
		}else{
			if(PIND&0x80) re++;
			PORTB&=~(0x02);
		}				
		if(l_fact[li]&(1<<lj)){
			if(!(PINC&0x01)) le++;
			PORTC|=0x04;
		}else{
			if(PINC&0x01) le++;
			PORTC&=~(0x04);
		}
		sleep;
		PORTB|=0x01;
		PORTC|=0x02;
		sleep;
		rj++;
		lj++;
		if(rj>7){
			rj=0;
			ri++;
		}
		if(lj>11){			
			if(light[li]!=l_fact[li]) 
				clock[li]=li;
			lj=0;
			li++;
		}
		if(ri>=rcount){
			ri=0;
			if(re==0){
				PORTB&=~(0x04);
				rerr=0;
			}
			rerr++;
			re=0;
		}
		if(li>=lcount){
			li=0;
			if(le==0){
				PORTC&=~(0x08);
				lerr=0;
			}
			lerr++;
			le=0;
		}
		sleep;
		PORTB|=0x04;
		PORTC|=0x08;
	}

}
/***********************************
adress	count		name
0		1			number
1		1			baudrate
2		1			reley
3		1			rcount
4		1			lcount
5		1			timer2

127		64		PLAN
191		64		BLOCK
255		64		LIGHT
319		64		LIMIT
383		64		DELAY
511		512		EEPROM	
***********************************/
