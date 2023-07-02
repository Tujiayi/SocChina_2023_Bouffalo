#include "as608.h"
#include "bflb_gpio.h"
#include "board.h"
#include "bflb_mtimer.h"
#include "keyboard.h"
#include "lcd.h"

extern uint8_t Lcd_Bufer[14];
extern SysPara AS608Para;
extern struct bflb_device_s *gpio;
extern uint16_t ValidN;
extern uint8_t PWM_Out_Flag;
extern uint32_t Audio_flag;

// 刷指纹
void press_FR(void)
{
	SearchResult seach;
	uint8_t ensure;
	char *str;
	ensure = PS_GetImage();
    // 获取图像成功 
	if(ensure == 0x00)
	{	
		ensure = PS_GenChar(CharBuffer1);
		if(ensure == 0x00) //生成特征成功
		{		
			ensure = PS_HighSpeedSearch(CharBuffer1, 0, AS608Para.PS_max, &seach);
			if(ensure == 0x00)//搜索成功
			{				
				Audio_flag = 1;
                printf("press_FR Successful.\r\n");
                printf("Have This People. ID:%d.", seach.pageID);
                printf("Match Grade:%d.\r\n", seach.mathscore);
				PWM_Out_Flag = 1;
				snprintf((char *)Lcd_Bufer, 14, "   Unclock   ");
    			lcd_draw_str_ascii16(11, 40, 0x0000, 0xffff, Lcd_Bufer, 14);
				snprintf((char *)Lcd_Bufer, 14, "  Successful ");
    			lcd_draw_str_ascii16(11, 70, 0x0000, 0xffff, Lcd_Bufer, 14);	
				bflb_mtimer_delay_ms(1000);
				PWM_Out_Flag = 0;
			}
			else 
			{
				Audio_flag = 2;
				printf("%s\r\n", EnsureMessage(ensure));
				snprintf((char *)Lcd_Bufer, 14, "   Unclock   ");
    			lcd_draw_str_ascii16(11, 40, 0x0000, 0xffff, Lcd_Bufer, 14);
				snprintf((char *)Lcd_Bufer, 14, "     Fail    ");
    			lcd_draw_str_ascii16(11, 70, 0x0000, 0xffff, Lcd_Bufer, 14);
			}						
	    }
		else
			printf("%s\r\n", EnsureMessage(ensure));
			
	bflb_gpio_set(gpio, GPIO_PIN_32);
	bflb_mtimer_delay_ms(600);
	}	
}

// 录指纹
void Add_FR(void)
{
	uint8_t i, ensure, processnum = 0;
	uint16_t ID = 0;
	uint16_t num_1 = 0;
	uint16_t num = 0;
	while(1)
	{
		switch (processnum)
		{
			case 0:
				i++;
				printf("Please, Press.\r\n");
				snprintf((char *)Lcd_Bufer, 14, " Please Press");
    		    lcd_draw_str_ascii16(11, 40, 0x0000, 0xffff, Lcd_Bufer, 14);
			    snprintf((char *)Lcd_Bufer, 14, "             ");
    		    lcd_draw_str_ascii16(11, 70, 0x0000, 0xffff, Lcd_Bufer, 14);
				ensure = PS_GetImage();
				if(ensure == 0x00) 
				{
					ensure = PS_GenChar(CharBuffer1);//生成特征
					if(ensure == 0x00)
					{
						snprintf((char *)Lcd_Bufer, 14, " FR is Normal");
    		    		lcd_draw_str_ascii16(11, 70, 0x0000, 0xffff, Lcd_Bufer, 14);
						printf("FR is Normal.\r\n");
						i = 0;
						processnum = 1;//跳到第二步						
					}
					else
					{
						printf("%s\r\n", EnsureMessage(ensure));	
					}						
				}
				else 
				{
					printf("%s\r\n", EnsureMessage(ensure));	
				}
				break;
			case 1:
				i++;
				printf("Please, Press again.\r\n");
				snprintf((char *)Lcd_Bufer, 14, " Press Again ");
    		    lcd_draw_str_ascii16(11, 40, 0x0000, 0xffff, Lcd_Bufer, 14);
			    snprintf((char *)Lcd_Bufer, 14, "             ");
    		    lcd_draw_str_ascii16(11, 70, 0x0000, 0xffff, Lcd_Bufer, 14);
				ensure = PS_GetImage();
				if(ensure == 0x00) 
				{
					ensure=PS_GenChar(CharBuffer2);//生成特征
					if(ensure == 0x00)
					{
						printf("FR is Normal.\r\n");
						snprintf((char *)Lcd_Bufer, 14, " FR is Normal");
    		    		lcd_draw_str_ascii16(11, 70, 0x0000, 0xffff, Lcd_Bufer, 14);
						i = 0;
						processnum = 2;//跳到第三步
					}
					else
					{
						printf("%s\r\n", EnsureMessage(ensure));	
					}						
				}
				else 
				{
					printf("%s\r\n", EnsureMessage(ensure));	
				}					
				break;
			case 2:
				printf("Compare FR.\r\n");
				snprintf((char *)Lcd_Bufer, 14, "  Compare FR ");
    		    lcd_draw_str_ascii16(11, 40, 0x0000, 0xffff, Lcd_Bufer, 14);
			    snprintf((char *)Lcd_Bufer, 14, "             ");
    		    lcd_draw_str_ascii16(11, 70, 0x0000, 0xffff, Lcd_Bufer, 14);
				ensure  = PS_Match();
				if(ensure == 0x00) 
				{
					printf("Compare success.\r\n");
					snprintf((char *)Lcd_Bufer, 14, "    success  ");
    		    	lcd_draw_str_ascii16(11, 70, 0x0000, 0xffff, Lcd_Bufer, 14);
					processnum = 3;//跳到第四步
				}
				else 
				{
					printf("Compare fail.\r\n");
					snprintf((char *)Lcd_Bufer, 14, "     Fail    ");
    		    	lcd_draw_str_ascii16(11, 70, 0x0000, 0xffff, Lcd_Bufer, 14);
					printf("%s\r\n", EnsureMessage(ensure));	
					i = 0;
					processnum = 0;//跳回第一步		
				}
				bflb_mtimer_delay_ms(1200);
				break;

			case 3:
				printf("Generate a fingerprint template.\r\n");
				ensure = PS_RegModel();
				if(ensure == 0x00) 
				{
					printf("Generate a fingerprint template successful.\r\n");
					processnum = 4;//跳到第五步
				}
				else 
				{
					processnum = 0;
					printf("%s\r\n", EnsureMessage(ensure));	
				}
				bflb_mtimer_delay_ms(1200);
				break;			
			case 4:	
				printf("Please enter the storage ID and press Enter to save.\r\n");
				printf("0=< ID <=299\r\n");
				snprintf((char *)Lcd_Bufer, 14, "   Enter ID  ");
    		    lcd_draw_str_ascii16(11, 40, 0x0000, 0xffff, Lcd_Bufer, 14);
			    snprintf((char *)Lcd_Bufer, 14, " 0=< ID <=299");
    		    lcd_draw_str_ascii16(11, 70, 0x0000, 0xffff, Lcd_Bufer, 14);
				
				do{
					num_1 = KEY_SCAN();
					if((num_1 != 0x00) && (num_1 != 0x0F)){
						if(num_1 == 0x0E){
							num_1 = 0x00;
						}
						printf("Value:%x\r\n", num_1);
						num = num * 10 + (uint16_t)(num_1);
						printf("Value:%d\r\n", num);
						snprintf((char *)Lcd_Bufer, 14, "   ID:%d   ", num);
    		    		lcd_draw_str_ascii16(11, 100, 0x0000, 0xffff, Lcd_Bufer, 14);
					}
				}while(num_1 != 0x0F);
				snprintf((char *)Lcd_Bufer, 14, "   ID:%d   ", num);
    		    lcd_draw_str_ascii16(11, 100, 0x0000, 0xffff, Lcd_Bufer, 14);
				do
					ID = num;
				while(!(ID < AS608Para.PS_max));//输入ID必须小于最大存储数值
				ensure = PS_StoreChar(CharBuffer2, num);//储存模板
				if(ensure == 0x00) 
				{			
					printf("Fingerprint entry successful.\r\n");
					PS_ValidTempleteNum(&ValidN);//读库指纹个数
					printf("Capacity:%d Grade:%d\r\n", AS608Para.PS_max - ValidN, AS608Para.PS_level);
					bflb_mtimer_delay_ms(1500);
					return ;
				}
				else
				{
					processnum = 0;
					lcd_draw_str_ascii16(10, 40, 0x0000, 0xffff, EnsureMessage(ensure), 14);
					printf("%s\r\n", EnsureMessage(ensure));	
				}					
				break;				
		}
		bflb_mtimer_delay_ms(1000);
		if(i == 5)//超过5次没有按手指则退出
		{
			break;	
		}				
	}
}

// 删除指纹
void Del_FR(void)
{
	uint8_t ensure;
	uint16_t num_1 = 0;
	uint16_t num = 0;
	printf("Delete the fingerprint.\r\n");
	printf("Please enter the fingerprint ID and press Enter to send.\r\n");
	printf("0=< ID <=299\r\n");
	snprintf((char *)Lcd_Bufer, 14, "   Enter ID  ");
    lcd_draw_str_ascii16(11, 40, 0x0000, 0xffff, Lcd_Bufer, 14);
	snprintf((char *)Lcd_Bufer, 14, " 0=< ID <=299");
    lcd_draw_str_ascii16(11, 70, 0x0000, 0xffff, Lcd_Bufer, 14);
	bflb_mtimer_delay_ms(50);
	// ensure = PS_Empty();//清空指纹库
	
	do{
		num_1 = KEY_SCAN();
		if((num_1 != 0x00) && (num_1 != 0x0F)){
			if(num_1 == 0x0E){
				num_1 = 0x00;
			}
			printf("Value:%x\r\n", num_1);
			num = num * 10 + (uint16_t)num_1;
			printf("Value:%d\r\n", num);
			snprintf((char *)Lcd_Bufer, 14, "   ID:%d   ", num);
    		lcd_draw_str_ascii16(11, 100, 0x0000, 0xffff, Lcd_Bufer, 14);
		}
	}while(num_1 != 0x0F);
	snprintf((char *)Lcd_Bufer, 14, "   ID:%d   ", num);
    lcd_draw_str_ascii16(11, 100, 0x0000, 0xffff, Lcd_Bufer, 14);
	ensure = PS_DeletChar(num, 1); //删除单个指纹
	if(ensure == 0x00)
	{
		printf("The fingerprint is deleted successfully.\r\n");	
	}
    else
		printf("%s\r\n", EnsureMessage(ensure));	
	bflb_mtimer_delay_ms(1200);
	// 读库指纹个数
	PS_ValidTempleteNum(&ValidN); 
	printf("Capacity:%d Grade:%d\r\n", AS608Para.PS_max - ValidN, AS608Para.PS_level);
}
