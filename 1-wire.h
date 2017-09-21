#include <avr/delay.h>
unsigned char  PIN;
unsigned char  WDEVICE[8], FORK;

void onewire_low(){WIRE1_DDR|=(1<<PIN);}
void onewire_hight(){WIRE1_DDR&=~(1<<PIN);}
unsigned char onewire_level(){return WIRE1_PIN&(1<<PIN);}
unsigned char onewire_reset(){
	unsigned char c;
	onewire_low();
	_delay_us(640);
	onewire_hight();
	_delay_us(2);	
	for(c=0;c<80;c++){
		if (!onewire_level()){
			while(!onewire_level()){_delay_us(1);}
			return 1;
		}
	_delay_us(1);
	}
	return 0;	
}	
void onewire_send_bit(unsigned char bit){
	onewire_low();
	if(bit){
		_delay_us(5);
		onewire_hight();
		_delay_us(90);
	}else{
		_delay_us(90);
		onewire_hight();
		_delay_us(5);
	}
}	
void onewire_send(unsigned char b){
	unsigned char p;
	for(p=8;p;p--){
		onewire_send_bit(b&1);
		b>>=1;
	}
}	
unsigned char onewire_read_bit(){
	unsigned char r;
	onewire_low();
	_delay_us(2);
	onewire_hight();
	_delay_us(8);
	r=onewire_level();
	_delay_us(80);
	return r;
}
unsigned char onewire_read(){
	unsigned char p,r=0;
	for(p=8;p;p--){
		r>>=1;
		if(onewire_read_bit()) r|=0x80;
	}
	return r;
}
unsigned char onewire_crc_update(unsigned char crc, unsigned char byte){
	unsigned char p;
	for (p=8;p;p--){
		crc=((crc^byte)&1)?(crc>>1)^0b10001100:(crc>>1);
		byte>>=1;
	}
	return crc;
}
void onewire_enum_init(){
	unsigned char i;
	for(i=0;i<8;i++)
		WDEVICE[i]=0;
	FORK=65;
}
unsigned char* onewire_enum_next(){
	if(!FORK) return 0;
	if(!onewire_reset()) return 0;
	unsigned char bp=8;
	unsigned char* pprev=&WDEVICE[0];
	unsigned char prev = *pprev;
	unsigned char next = 0;
	unsigned char p = 1;
	unsigned char newfork = 0;
	unsigned char not0,not1;
	onewire_send(0xF0);
	for(;;){
		not0=onewire_read_bit();
		not1=onewire_read_bit();
		if(!not0){
			if(!not1){
				if(p<FORK){
					if(prev & 1)
						next|=0x80;
					else
						newfork=p;
				}else{ 
					if(p==FORK)
						next|=0x80;
					else
						newfork=p;
				}
			}
		}else{
			if(!not1)
				next|=0x80;
			else
				return 0;
		}
		onewire_send_bit(next & 0x80);
		bp--;
		if(!bp){
			*pprev=next;
			if(p>=64) break;
			next=0;
			pprev++;
			prev=*pprev;
			bp=8;
		}else{
			if(p>=64) break;
			prev>>=1;
			next>>=1;
		}
		p++;
	}
	FORK=newfork;
	return &WDEVICE;	
}
signed short read_temp(unsigned char ID[8]){
	unsigned char i,D[8],c,j;
	if((ID[0]!=0x28)&&(ID[0]!=0x22)) return 0x8000;
	for(j=0;j<10;j++){
//		if(!onewire_reset()) return 0xFFFF;
		onewire_reset();
		onewire_send(0x55);
		for(i=0;i<8;i++)
			onewire_send(ID[i]);
		onewire_send(0xBE);
		c=0;
		for(i=0;i<8;i++){
			D[i]=onewire_read();
			c=onewire_crc_update(c,D[i]);
		}
		if(onewire_read()==c)
			return(D[1]<<8)|D[0];
	}			
	return 0x8000;
}
