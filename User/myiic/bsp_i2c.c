#include "bsp_i2c.h"
#include "delay.h"
#include "oled.h"
#include "gui.h"
#include "test.h"
#include "stdio.h"
#include "string.h"

uint8_t   ack_status=0;
uint8_t   readByte[6];
uint8_t   AHT20_status=0;

uint32_t  H1=0;  //Humility
uint32_t  T1=0;  //Temperature

uint8_t t1,t2,t3,t4;
uint8_t h1,h2,h3,h4;

uint8_t  AHT20_OutData[4];
uint8_t  AHT20sendOutData[10] = {0xFA, 0x06, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF};
char strTemp[30];  //声明字符数组strTemp,初始化元素30  
char strHumi[30];  //声明字符数组strHumi,初始化元素30
int t;
int h;
float a;
float b;
void IIC_Init(void)
{					     
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );	
	   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //í?íìê?3?
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
 
	IIC_SCL=1;
	IIC_SDA=1;
 
}
//2úéúIIC?eê?D?o?
void IIC_Start(void)
{
	SDA_OUT();     //sda??ê?3?
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	delay_us(4);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL=0;//?ˉ×?I2C×ü??￡?×?±?·￠?í?ò?óê?êy?Y 
}	  
//2úéúIICí￡?1D?o?
void IIC_Stop(void)
{
	SDA_OUT();//sda??ê?3?
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL=1; 
	IIC_SDA=1;//·￠?íI2C×ü???áê?D?o?
	delay_us(4);							   	
}
//μè′yó|′eD?o?μ?à′
//·μ???μ￡o1￡??óê?ó|′eê§°ü
//        0￡??óê?ó|′e3é1|
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDAéè???aê?è?  
	IIC_SDA=1;delay_us(1);	   
	IIC_SCL=1;delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;//ê±?óê?3?0 	   
	return 0;  
} 
//2úéúACKó|′e
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}
//2?2úéúACKó|′e		    
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}					 				     
//IIC·￠?íò???×??ú
//·μ??′ó?úóD?Tó|′e
//1￡?óDó|′e
//0￡??Tó|′e			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
		SDA_OUT(); 	    
    IIC_SCL=0;//à-μíê±?ó?aê?êy?Y′?ê?
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);   //??TEA5767?aèy???óê±??ê?±?D?μ?
		IIC_SCL=1;
		delay_us(2); 
		IIC_SCL=0;	
		delay_us(2);
    }	 
} 	    
//?á1??×??ú￡?ack=1ê±￡?·￠?íACK￡?ack=0￡?·￠?ínACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDAéè???aê?è?
  for(i=0;i<8;i++ )
	{
    IIC_SCL=0; 
    delay_us(2);
		IIC_SCL=1;
    receive<<=1;
    if(READ_SDA)receive++;   
		delay_us(1); 
  }					 
	if (!ack)
			IIC_NAck();//·￠?ínACK
	else
			IIC_Ack(); //·￠?íACK   
	return receive;
}
 
void IIC_WriteByte(uint16_t addr,uint8_t data,uint8_t device_addr)
{
	IIC_Start();  
	
	if(device_addr==0xA0) //eepromμ??·′óóú1×??ú
		IIC_Send_Byte(0xA0 + ((addr/256)<<1));//·￠?í??μ??·
	else
		IIC_Send_Byte(device_addr);	    //·￠?÷?tμ??·
	IIC_Wait_Ack(); 
	IIC_Send_Byte(addr&0xFF);   //·￠?íμíμ??·
	IIC_Wait_Ack(); 
	IIC_Send_Byte(data);     //·￠?í×??ú							   
	IIC_Wait_Ack();  		    	   
  IIC_Stop();//2úéúò???í￡?1ì??t 
	if(device_addr==0xA0) //
		delay_ms(10);
	else
		delay_us(2);
}
 
uint16_t IIC_ReadByte(uint16_t addr,uint8_t device_addr,uint8_t ByteNumToRead)  //?á??′??÷?ò?áêy?Y
{	
		uint16_t data;
		IIC_Start();  
		if(device_addr==0xA0)
			IIC_Send_Byte(0xA0 + ((addr/256)<<1));
		else
			IIC_Send_Byte(device_addr);	
		IIC_Wait_Ack();
		IIC_Send_Byte(addr&0xFF);   //·￠?íμíμ??·
		IIC_Wait_Ack(); 
 
		IIC_Start();  	
		IIC_Send_Byte(device_addr+1);	    //·￠?÷?tμ??·
		IIC_Wait_Ack();
		if(ByteNumToRead == 1)//LM75???èêy?Y?a11bit
		{
			data=IIC_Read_Byte(0);
		}
		else
			{
				data=IIC_Read_Byte(1);
				data=(data<<8)+IIC_Read_Byte(0);
			}
		IIC_Stop();//2úéúò???í￡?1ì??t	    
		return data;
}


/**********
*é???2?·??aIO?ú?￡?éI2C????
*
*′ó?aò????aê??aAHT20μ?????I2C
*oˉêy??óDIICoíI2Cμ???±e￡???×￠òa￡?￡?￡?￡?￡?
*
*2020/2/23×?oóDT??è??ú
*
***********/
void  read_AHT20_once(void)
{
	delay_ms(10);

	reset_AHT20();
	delay_ms(10);

	init_AHT20();
	delay_ms(10);

	startMeasure_AHT20();
	delay_ms(80);

	read_AHT20();
	delay_ms(5);
}


void  reset_AHT20(void)
{

	I2C_Start();

	I2C_WriteByte(0x70);
	ack_status = Receive_ACK();
	if(ack_status) printf("1");
	else printf("1-n-");
	I2C_WriteByte(0xBA);
	ack_status = Receive_ACK();
		if(ack_status) printf("2");
	else printf("2-n-");
	I2C_Stop();

	/*
	AHT20_OutData[0] = 0;
	AHT20_OutData[1] = 0;
	AHT20_OutData[2] = 0;
	AHT20_OutData[3] = 0;
	*/
}



void  init_AHT20(void)
{
	I2C_Start();

	I2C_WriteByte(0x70);
	ack_status = Receive_ACK();
	if(ack_status) printf("3");
	else printf("3-n-");	
	I2C_WriteByte(0xE1);
	ack_status = Receive_ACK();
	if(ack_status) printf("4");
	else printf("4-n-");
	I2C_WriteByte(0x08);
	ack_status = Receive_ACK();
	if(ack_status) printf("5");
	else printf("5-n-");
	I2C_WriteByte(0x00);
	ack_status = Receive_ACK();
	if(ack_status) printf("6");
	else printf("6-n-");
	I2C_Stop();
}



void  startMeasure_AHT20(void)
{
	//------------
	I2C_Start();

	I2C_WriteByte(0x70);
	ack_status = Receive_ACK();
	if(ack_status) printf("7");
	else printf("7-n-");
	I2C_WriteByte(0xAC);
	ack_status = Receive_ACK();
	if(ack_status) printf("8");
	else printf("8-n-");
	I2C_WriteByte(0x33);
	ack_status = Receive_ACK();
	if(ack_status) printf("9");
	else printf("9-n-");
	I2C_WriteByte(0x00);
	ack_status = Receive_ACK();
	if(ack_status) printf("10");
	else printf("10-n-");
	I2C_Stop();
}



void read_AHT20(void)
{
	uint8_t   i;
	for(i=0; i<6; i++)
	{
		readByte[i]=0;
	}

	//-------------
	I2C_Start();

	I2C_WriteByte(0x71);
	ack_status = Receive_ACK();
	readByte[0]= I2C_ReadByte();
	Send_ACK();

	readByte[1]= I2C_ReadByte();
	Send_ACK();

	readByte[2]= I2C_ReadByte();
	Send_ACK();

	readByte[3]= I2C_ReadByte();
	Send_ACK();

	readByte[4]= I2C_ReadByte();
	Send_ACK();

	readByte[5]= I2C_ReadByte();
	SendNot_Ack();
	//Send_ACK();

	I2C_Stop();

	//--------------
	if( (readByte[0] & 0x68) == 0x08 )
	{
		H1 = readByte[1];
		H1 = (H1<<8) | readByte[2];
		H1 = (H1<<8) | readByte[3];
		H1 = H1>>4;

		H1 = (H1*1000)/1024/1024;

		T1 = readByte[3];
		T1 = T1 & 0x0000000F;
		T1 = (T1<<8) | readByte[4];
		T1 = (T1<<8) | readByte[5];

		T1 = (T1*2000)/1024/1024 - 500;

		AHT20_OutData[0] = (H1>>8) & 0x000000FF;
		AHT20_OutData[1] = H1 & 0x000000FF;

		AHT20_OutData[2] = (T1>>8) & 0x000000FF;
		AHT20_OutData[3] = T1 & 0x000000FF;
	}
	else
	{
		AHT20_OutData[0] = 0xFF;
		AHT20_OutData[1] = 0xFF;

		AHT20_OutData[2] = 0xFF;
		AHT20_OutData[3] = 0xFF;
		printf("lyy");

	}
	printf("\r\n");
	
	printf("温度:%d%d.%d",T1/100,(T1/10)%10,T1%10);
	printf("湿度:%d%d.%d",H1/100,(H1/10)%10,H1%10);
	printf("\r\n");
	t=T1/10;
	t1=T1%10;
	a=(float)(t+t1*0.1);
	h=H1/10;
	h1=H1%10;
	b=(float)(h+h1*0.1);
	sprintf(strTemp,"%.1f",a);   //调用Sprintf函数把DHT11的温度数据格式化到字符串数组变量strTemp中  
  sprintf(strHumi,"%.1f",b);    //调用Sprintf函数把DHT11的湿度数据格式化到字符串数组变量strHumi中  
	//printf(strTemp);
	//printf("/r/n");
	GUI_ShowCHinese(16,00,16,"温湿度显示",1);
	GUI_ShowCHinese(16,20,16,"温度",1);
	GUI_ShowString(53,20,strTemp,16,1);
	GUI_ShowCHinese(16,38,16,"湿度",1);
	GUI_ShowString(53,38,strHumi,16,1);
	delay_ms(1500);		
	delay_ms(1500);
	delay_ms(1500);
	delay_ms(1500);
	
}




uint8_t  Receive_ACK(void)
{
	uint8_t result=0;
	uint8_t cnt=0;

	IIC_SCL = 0;
	SDA_IN(); 
	delay_us(4);

	IIC_SCL = 1;
	delay_us(4);

	while(READ_SDA && (cnt<100))
	{
		cnt++;
	}

	IIC_SCL = 0;
	delay_us(4);

	if(cnt<100)
	{
		result=1;
	}
	return result;
}



void  Send_ACK(void)
{
	SDA_OUT();
	IIC_SCL = 0;
	delay_us(4);

	IIC_SDA = 0;
	delay_us(4);

	IIC_SCL = 1;
	delay_us(4);
	IIC_SCL = 0;
	delay_us(4);

	SDA_IN();
}



void  SendNot_Ack(void)
{
	SDA_OUT();
	IIC_SCL = 0;
	delay_us(4);

	IIC_SDA = 1;
	delay_us(4);

	IIC_SCL = 1;
	delay_us(4);

	IIC_SCL = 0;
	delay_us(4);

	IIC_SDA = 0;
	delay_us(4);
}


void I2C_WriteByte(uint8_t  input)
{
	uint8_t  i;
	SDA_OUT();
	for(i=0; i<8; i++)
	{
		IIC_SCL = 0;
		delay_ms(5);

		if(input & 0x80)
		{
			IIC_SDA = 1;
			//delaymm(10);
		}
		else
		{
			IIC_SDA = 0;
			//delaymm(10);
		}

		IIC_SCL = 1;
		delay_ms(5);

		input = (input<<1);
	}

	IIC_SCL = 0;
	delay_us(4);

	SDA_IN();
	delay_us(4);
}	


uint8_t I2C_ReadByte(void)
{
	uint8_t  resultByte=0;
	uint8_t  i=0, a=0;

	IIC_SCL = 0;
	SDA_IN();
	delay_ms(4);

	for(i=0; i<8; i++)
	{
		IIC_SCL = 1;
		delay_ms(3);

		a=0;
		if(READ_SDA)
		{
			a=1;
		}
		else
		{
			a=0;
		}

		//resultByte = resultByte | a;
		resultByte = (resultByte << 1) | a;

		IIC_SCL = 0;
		delay_ms(3);
	}

	SDA_IN();
	delay_ms(10);

	return   resultByte;
}


void  set_AHT20sendOutData(void)
{
	/* --------------------------
	 * 0xFA 0x06 0x0A temperature(2 Bytes) humility(2Bytes) short Address(2 Bytes)
	 * And Check (1 byte)
	 * -------------------------*/
	AHT20sendOutData[3] = AHT20_OutData[0];
	AHT20sendOutData[4] = AHT20_OutData[1];
	AHT20sendOutData[5] = AHT20_OutData[2];
	AHT20sendOutData[6] = AHT20_OutData[3];

//	AHT20sendOutData[7] = (drf1609.shortAddress >> 8) & 0x00FF;
//	AHT20sendOutData[8] = drf1609.shortAddress  & 0x00FF;

//	AHT20sendOutData[9] = getXY(AHT20sendOutData,10);
}


void  I2C_Start(void)
{
	SDA_OUT();
	IIC_SCL = 1;
	delay_ms(4);

	IIC_SDA = 1;
	delay_ms(4);
	IIC_SDA = 0;
	delay_ms(4);

	IIC_SCL = 0;
	delay_ms(4);
}



void  I2C_Stop(void)
{
	SDA_OUT();
	IIC_SDA = 0;
	delay_ms(4);

	IIC_SCL = 1;
	delay_ms(4);

	IIC_SDA = 1;
	delay_ms(4);
}

