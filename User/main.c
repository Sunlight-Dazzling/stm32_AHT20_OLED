#include "delay.h"
#include "usart.h"
#include "bsp_i2c.h"
#include "sys.h"

#include "oled.h"
#include "gui.h"
#include "test.h"

int main(void)
{	
	delay_init();	    	       //��ʱ������ʼ��    	  
	uart_init(115200);	 
	IIC_Init();
		  
	NVIC_Configuration(); 	   //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ� 	
	OLED_Init();			         //��ʼ��OLED  
	OLED_Clear(0); 
	while(1)
	{
		//printf("�¶�ʪ����ʾ");
		read_AHT20_once();
		OLED_Clear(0); 
		delay_ms(1500);
  }
}
