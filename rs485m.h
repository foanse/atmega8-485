
#define MAX 20
volatile unsigned char number;
volatile unsigned int COUNT_COMAND, CRC;
volatile unsigned char status, BUF[MAX],COUNT;

unsigned int read_registr_param(unsigned int address);
void write_registr_param(unsigned int address, unsigned int data);

void crc(unsigned char data){
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
}
void sendchar(unsigned char data){
	crc(data);
	while ( !(UCSRA & (1<<UDRE)) );
	UDR = data;			        
}
void sendcrc(){
	while ( !(UCSRA & (1<<UDRE)) );
	UDR = (unsigned char)(CRC>>8);			        
	while ( !(UCSRA & (1<<UDRE)) );
	UCSRA|=(1<<TXC);
	UDR = (unsigned char)CRC;			        
	while ( !(UCSRA & (1<<TXC)) );
	PORTD&=~(0x04);
}
void USART_Init(void)
{
	TIMSK=0;
	//UART
	UBRRH = 0;
	UBRRL = 103;
	UCSRA = (1<<U2X);
	UCSRB = (1<<RXEN)|(1<<TXEN)|(1<<RXCIE);
	UCSRC = (0<<USBS)|(1<<UCSZ0)|(1<<UCSZ1);
	//timer0 
	TCCR0=0x00;
	TIMSK|=(1<<TOIE0);
	//timer1
	TCCR1A=0x00;
	TCCR1B=0x00;
//	TCCR1B=0x08;
	OCR1A=0x0C;
	OCR1B=0x1C;
	TIMSK|=(1<<OCIE1A)|(1<<OCIE1B);
} 
void swit(){
	CRC=0xFFFF;
	sendchar(number);
	unsigned int B,D,C;
	switch(BUF[1]){
	case 0x01:
			if(COUNT!=8) goto er;
			B=(BUF[2]<<8)|BUF[3];
			sendchar(0x01);
			sendchar(0x01);
			D=B>>3;
			B&=0x0007;
			C=read_registr_param(D);
			if(C&(1<<B))
				sendchar(0x01);
			else
				sendchar(0x00);
			goto re;
//				case 0x02:read_bit_input();break;
	case 0x03:
	case 0x04:
			if(COUNT!=8) goto er;
			sendchar(BUF[1]);
			sendchar(BUF[5]);
			B=(BUF[2]<<8)|BUF[3];
			for(D=B;D<(B+BUF[5]);D++){	
				C=read_registr_param(D);
				sendchar((unsigned char)(C>>8));
				sendchar((unsigned char)C);
			}
			goto re;
	case 0x05:
			if(COUNT!=8) goto er;
			B=(BUF[2]<<8)|BUF[3];
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
			sendchar(0x11);sendchar(3);sendchar(FIRST_ID);sendchar(SECOND_ID);sendchar(0xFF);
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
ISR(TIMER0_OVF_vect)
{
	PORTD&=~(0x08);
	TCCR0=0x00;
}
ISR(TIMER1_COMPA_vect)
{
	status|=0x20;
}	
ISR(TIMER1_COMPB_vect)
{
	TCCR1B=0x00;
	if(status&0x10) goto end;
	if((number!=BUF[0])&&(BUF[0]!=0x00)) goto end;
	if((((unsigned char)(CRC>>8))!=BUF[COUNT-2])||(((unsigned char)CRC)!=BUF[COUNT-1])) goto end;
	if(number==BUF[0])PORTD|=0x04;
	PORTD|=0x08;
	TCCR0=0x05;
	COUNT_COMAND++;
	swit();
	end:
	CRC=0xFFFF;
	status=0x00;
	COUNT=0;
}
ISR(USART_RXC_vect)
{
	unsigned char S=UCSRA;
	BUF[COUNT]=UDR;
	if((S&0x1C)||(status&0x20)||(COUNT>=MAX)){
		CRC=0xFFFF;
		COUNT=0;
		status|=0x10;
		return;
	}
	if(COUNT>1) crc(BUF[COUNT-2]);
	COUNT++;
	TCNT1=0x00;
	TCCR1B=0x05;
}