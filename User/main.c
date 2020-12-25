#include "delay.h"
#include "usart.h"
#include "bsp_i2c.h"
#include "sys.h"

#include "oled.h"
#include "gui.h"
#include "test.h"

int main(void)
{	
	delay_init();	    	       //延时函数初始化    	  
	uart_init(115200);	 
	IIC_Init();
		  
	NVIC_Configuration(); 	   //设置NVIC中断分组2:2位抢占优先级，2位响应优先级 	
	OLED_Init();			         //初始化OLED  
	OLED_Clear(0); 
	while(1)
	{
		//printf("温度湿度显示");
		read_AHT20_once();
		OLED_Clear(0); 
		delay_ms(1500);
  }
}
